/**
 * This file is part of DSM.
 *
 * Copyright (C) 2019 CEIT (Universidad de Navarra) and Universidad de Zaragoza
 * Developed by Jon Zubizarreta,
 * for more information see <https://github.com/jzubizarreta/dsm>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with DSM. If not, see <http://www.gnu.org/licenses/>.
 */
#include "utils/CvoPointCloud.hpp"
#include "ActivePoint.h"
#include "CandidatePoint.h"
#include "Frame.h"
#include "Utils/Settings.h"
#include "Utils/GlobalCalibration.h"
#include "Utils/Interpolation.h"
#include "Optimization/PointParameterBlock.h"
#include "Optimization/PhotometricResidual.h"

namespace dsm
{
  ActivePoint::ActivePoint(int32_t creationID, const std::unique_ptr<CandidatePoint>& candidate, float filteredIdepth) :
    ActivePoint(creationID, candidate) {
    iDepth_ = filteredIdepth;
  }
  
  
  ActivePoint::ActivePoint(int32_t creationID, const std::unique_ptr<CandidatePoint>& candidate) :
    currentID_(creationID), status_(ISOLATED)
  {
    const auto& calib = GlobalCalibration::getInstance();
    const auto& settings = Settings::getInstance();
    const float weightConstant = settings.weightConstant;

    // resize
    const int numPyramids = settings.optMaxLevel + 1;
    this->u_.resize(numPyramids);
    this->v_.resize(numPyramids);
    this->color_.resize(numPyramids);
    this->weights_.resize(numPyramids);
    this->validity_.resize(numPyramids);

    // copy values from candidate point
    this->refFrame_ = candidate->refFrame_;
    this->iDepth_ = candidate->iDepth_;
    this->ray_ = candidate->ray_;

    // pyramidal values
    for (int lvl = 0; lvl < numPyramids; ++lvl)
    {
      // set as valid initially
      this->validity_[lvl] = true;

      // image pixel location
      this->u_[lvl] = ((candidate->u0_[0] + 0.5f) / ((int)1 << lvl)) - 0.5f;
      this->v_[lvl] = ((candidate->u0_[1] + 0.5f) / ((int)1 << lvl)) - 0.5f;

      const float* const image = this->refFrame_->image(lvl);
      const float* const grad = this->refFrame_->gradient(lvl);
      const int width = calib.width(lvl);	
      const int height = calib.height(lvl);

      this->color_[lvl].resize(Pattern::size());
      this->weights_[lvl].resize(Pattern::size());
      for (int idx = 0; idx < Pattern::size(); ++idx)
      {
        const float uj = this->u_[lvl] + (float)Pattern::at(idx, 0);
        const float vj = this->v_[lvl] + (float)Pattern::at(idx, 1);

        if (!(uj > 1.1f && uj < (width - 2.1f) && vj > 1.1f && vj < (height - 2.1f)))
        {
          // mark as not valid for this level
          this->validity_[lvl] = false;
          break;
        }

        // color
        this->color_[lvl][idx] = bilinearInterpolation(image, uj, vj, width);

        // weight = sqrt( c^2 / (c^2 + gradj^2) )
        const float gradj = bilinearInterpolation(grad, uj, vj, width);				
        this->weights_[lvl][idx] = sqrtf(weightConstant / (weightConstant + gradj));
      }
    }

    // optimization data
    this->pointBlock_ = std::make_unique<PointParameterBlock>(this->iDepth_);
    this->iDepthHessian_ = std::numeric_limits<float>::max();
    this->parallax_ = 0.f;


    // cvo data
    this->features_ = candidate->features();
    this->semantics_ = candidate->semantics();
    this->geometric_type_ = candidate->geometricType();
    // voxel set to nullptr
    voxel_ = nullptr;
                  
  }

  Eigen::Vector3f ActivePoint::xyz() const  {

    const auto& calib = GlobalCalibration::getInstance();
    const Eigen::Matrix3f& invK = calib.invMatrix3f(0);

    Eigen::Vector3f xyz;
    xyz << u_[0], v_[0], 1;
    xyz = (xyz / iDepth_).eval();
    xyz = (invK * xyz).eval();

    return xyz;
  }

  ActivePoint::~ActivePoint()
  {}

  int32_t ActivePoint::currentID() const
  {
    return this->currentID_;
  }

  float ActivePoint::u(int32_t lvl) const
  {
    return this->u_[lvl];
  }

  float ActivePoint::v(int32_t lvl) const
  {
    return this->v_[lvl];
  }

  Frame* ActivePoint::reference() const
  {
    return this->refFrame_;
  }

  Eigen::Vector3f ActivePoint::pt3d() const
  {
    return (1.f / this->iDepth_)*this->ray_;
  }

  const Eigen::VecXf& ActivePoint::colors(int32_t lvl) const
  {
    return this->color_[lvl];
  }

  const Eigen::VecXf& ActivePoint::weights(int32_t lvl) const
  {
    return this->weights_[lvl];
  }

  const bool ActivePoint::valid(int32_t lvl) const
  {
    return this->validity_[lvl];
  }

  float ActivePoint::iDepth() const
  {
    return this->iDepth_;
  }

  void ActivePoint::setIDepth(float value)
  {
    this->iDepth_ = value;
  }

  Visibility ActivePoint::visibility(int keyframeID) const
  {
    if ((int)this->visibility_.size() <= keyframeID)
    {
      return Visibility::UNINITIALIZED;
    }

    return this->visibility_[keyframeID];
  }

  void ActivePoint::setVisibility(int idx, Visibility vis)
  {
    // resize if the historical is older
    if ((int)this->visibility_.size() <= idx)
    {
      this->visibility_.resize(idx + 1, Visibility::UNINITIALIZED);
    }

    // change value
    this->visibility_[idx] = vis;
  }

  const Eigen::Vector2f& ActivePoint::centerProjection() const
  {
    return this->centerProjection_;
  }

  void ActivePoint::setCenterProjection(const Eigen::Vector2f& center)
  {
    this->centerProjection_ = center;
  }

  const std::unique_ptr<PointParameterBlock>& ActivePoint::pointBlock() const
  {
    return this->pointBlock_;
  }

  const std::unordered_map<Frame*, std::unique_ptr<PhotometricResidual>>& ActivePoint::observations() const
  {
    return this->observations_;
  }

  void ActivePoint::addObservation(Frame* const kf, std::unique_ptr<PhotometricResidual>& obs)
  {
    if (this->observations_.find(kf) == this->observations_.end())
    {
      this->observations_[kf] = std::move(obs);
    }
  }

  void ActivePoint::eraseObservation(Frame* const kf)
  {
    this->observations_.erase(kf);
  }

  bool ActivePoint::observationExists(Frame* const kf)
  {
    return (this->observations_.find(kf) != this->observations_.end());
  }

  int32_t ActivePoint::numObservations() const
  {
    return (int32_t)this->observations_.size();
  }

  int32_t ActivePoint::age(int newKeyframeID) const
  {
    return newKeyframeID - this->currentID_;
  }

  void ActivePoint::mergeOptimizationResult()
  {
    this->iDepth_ = (float)this->pointBlock_->getIDepth();
  }

  float ActivePoint::iDepthHessian() const
  {
    return this->iDepthHessian_;
  }

  void ActivePoint::setIDepthHessian(float H)
  {
    this->iDepthHessian_ = H;
  }

  float ActivePoint::parallax() const
  {
    return this->parallax_;
  }

  void ActivePoint::setParallax(float p)
  {
    this->parallax_ = p;
  }

  const Voxel<ActivePoint>* ActivePoint::voxel() const
  {
    return this->voxel_;
  }

  void ActivePoint::setVoxel(const Voxel<ActivePoint>* voxelIn)
  {
    this->voxel_ = voxelIn;
  }

  ActivePoint::Status ActivePoint::status() const {
    return this->status_;
  }
  void ActivePoint::setStatus(ActivePoint::Status stat) {
    this->status_ = stat;
  }


  template <typename PointPtr>
  void ActivePoint::activePointsToCvoPointCloud(const std::vector<PointPtr> & activePoints_,
                                                cvo::CvoPointCloud & output) {
    int num_points = activePoints_.size();
    if (num_points < 1) return;
    int num_features = activePoints_[0]->features().size();
    int num_semantics = activePoints_[0]->semantics().size();
    output.reserve(num_points, num_features, num_semantics);

    for (int i = 0; i < num_points; i++) {
      auto & p = *activePoints_[i];
      Eigen::Vector3f xyz = p.xyz();
      auto features = p.features();
      auto semantics = p.semantics();
      Eigen::VectorXf geoType = p.geometricType();
      output.add_point(i, xyz, features, semantics, geoType);
    }
  
  }


  template void ActivePoint::activePointsToCvoPointCloud<const ActivePoint *>(const std::vector<const ActivePoint *> & activePoints_,
                                                                     cvo::CvoPointCloud & output);
  template void ActivePoint::activePointsToCvoPointCloud<std::unique_ptr<ActivePoint>>(const std::vector<std::unique_ptr<ActivePoint>> & activePoints_,
                                                                              cvo::CvoPointCloud & output);
  

  
}

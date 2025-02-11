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

#include "Frame.h"
#include "ActivePoint.h"
#include "CandidatePoint.h"
#include "Statistics/TDistribution.h"
#include "Statistics/RobustNormalDistribution.h"
#include "Utils/Settings.h"
#include "Utils/GlobalCalibration.h"
#include "Memory/BufferPool.h"
#include "Optimization/FrameParameterBlock.h"

#include "dsm/BuildFlags.h"
#include "utils/CvoPointCloud.hpp"
#include "utils/ImageStereo.hpp"
#include "utils/ImageRGBD.hpp"
#include "utils/RawImage.hpp"
#include "utils/CvoPoint.hpp"
#include <iostream>

namespace dsm
{
  Frame::Frame(int id, double timestamp, unsigned char* image) :
    frameID_(id),
    timestamp_(timestamp),
    trackingParent_(nullptr),
    affineLight_(0.f, 0.f),
    thisToParentLight_(0.f, 0.f),
    type_(Type::FRAME),
    status_(Status::INACTIVE),
    depthType(DepthType::MONO),
    depthScale(1.0)
  {
    const auto& settings = Settings::getInstance();
    const auto& calib = GlobalCalibration::getInstance();

    // image pyramids
    this->images_ = std::make_unique<ImagePyramid<float>>(calib.levels(), image);

    // gradient pyramids
    this->gradients_ = std::make_unique<GradientPyramid<float>>(calib.levels(), *this->images_);

    // identity poses
    this->thisToParentPose_ = Sophus::SE3f();
    this->camToWorld_ = Sophus::SE3f();

    this->graphNode = nullptr;

    this->flaggedToDrop_ = false;

    // error distribution
    std::shared_ptr<IDistribution> errorDist;
    if (settings.useTDistribution)
    {
      errorDist = std::make_shared<TDistribution>(settings.defaultNu, settings.defaultMu, settings.defaultSigma);
    }
    else
    {
      errorDist = std::make_shared<RobustNormalDistribution>(settings.defaultMu, settings.defaultSigma);
    }

    this->setErrorDistribution(errorDist);
  }

  Frame::Frame(int id, double timestamp, unsigned char* image, std::shared_ptr<cvo::ImageStereo> color_img,  pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd, std::shared_ptr<cvo::CvoPointCloud> full_pc) :
    frameID_(id),
    timestamp_(timestamp),
    trackingParent_(nullptr),
    affineLight_(0.f, 0.f),
    thisToParentLight_(0.f, 0.f),
    type_(Type::FRAME),
    status_(Status::INACTIVE),
    trackingPoints_(new_frame_pcd),
    rawImg(color_img),
    //stereoDisparity(color_img.disparity().begin(), disparity.end()),
    depthType(DepthType::STEREO),
    depthScale(1.0),
    fullPoints_(full_pc)
  {
    const auto& settings = Settings::getInstance();
    const auto& calib = GlobalCalibration::getInstance();

    // image pyramids
    this->images_ = std::make_unique<ImagePyramid<float>>(calib.levels(), image);

    // gradient pyramids
    this->gradients_ = std::make_unique<GradientPyramid<float>>(calib.levels(), *this->images_);

    // identity poses
    this->thisToParentPose_ = Sophus::SE3f();
    this->camToWorld_ = Sophus::SE3f();

    this->graphNode = nullptr;

    this->flaggedToDrop_ = false;

    // error distribution
    std::shared_ptr<IDistribution> errorDist;
    if (settings.useTDistribution)
    {
      errorDist = std::make_shared<TDistribution>(settings.defaultNu, settings.defaultMu, settings.defaultSigma);
    }
    else
    {
      errorDist = std::make_shared<RobustNormalDistribution>(settings.defaultMu, settings.defaultSigma);
    }

    this->setErrorDistribution(errorDist);
  }


  Frame::Frame(int id, double timestamp, unsigned char* image, std::shared_ptr<cvo::ImageRGBD<float>> color_img,  pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd,  float depth_scale, std::shared_ptr<cvo::CvoPointCloud> full_pc) :

    frameID_(id),
    timestamp_(timestamp),
    trackingParent_(nullptr),
    affineLight_(0.f, 0.f),
    thisToParentLight_(0.f, 0.f),
    type_(Type::FRAME),
    status_(Status::INACTIVE),
    trackingPoints_(new_frame_pcd),
    rawImg(color_img),
    //rgbdDepth(color_img->depth_image().begin(), color_img->depth_image().end()),
    depthType(DepthType::RGBD_FLOAT),
    depthScale(depth_scale),
    fullPoints_(full_pc)
  {
    const auto& settings = Settings::getInstance();
    const auto& calib = GlobalCalibration::getInstance();

    // image pyramids
    this->images_ = std::make_unique<ImagePyramid<float>>(calib.levels(), image);

    // gradient pyramids
    this->gradients_ = std::make_unique<GradientPyramid<float>>(calib.levels(), *this->images_);

    // identity poses
    this->thisToParentPose_ = Sophus::SE3f();
    this->camToWorld_ = Sophus::SE3f();

    this->graphNode = nullptr;

    this->flaggedToDrop_ = false;

    // error distribution
    std::shared_ptr<IDistribution> errorDist;
    if (settings.useTDistribution)
    {
      errorDist = std::make_shared<TDistribution>(settings.defaultNu, settings.defaultMu, settings.defaultSigma);
    }
    else
    {
      errorDist = std::make_shared<RobustNormalDistribution>(settings.defaultMu, settings.defaultSigma);
    }

    this->setErrorDistribution(errorDist);

  }

  Frame::Frame(int id, double timestamp, unsigned char* image, std::shared_ptr<cvo::ImageRGBD<uint16_t>> color_img,  pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd,  float depth_scale, std::shared_ptr<cvo::CvoPointCloud> full_pc) :

    frameID_(id),
    timestamp_(timestamp),
    trackingParent_(nullptr),
    affineLight_(0.f, 0.f),
    thisToParentLight_(0.f, 0.f),
    type_(Type::FRAME),
    status_(Status::INACTIVE),
    trackingPoints_(new_frame_pcd),
    rawImg(color_img),
    //rgbdDepth(color_img->depth_image().begin(), color_img->depth_image().end()),
    depthType(DepthType::RGBD_UINT16),
    depthScale(depth_scale),
    fullPoints_(full_pc)
  {
    const auto& settings = Settings::getInstance();
    const auto& calib = GlobalCalibration::getInstance();

    // image pyramids
    this->images_ = std::make_unique<ImagePyramid<float>>(calib.levels(), image);

    // gradient pyramids
    this->gradients_ = std::make_unique<GradientPyramid<float>>(calib.levels(), *this->images_);

    // identity poses
    this->thisToParentPose_ = Sophus::SE3f();
    this->camToWorld_ = Sophus::SE3f();

    this->graphNode = nullptr;

    this->flaggedToDrop_ = false;

    // error distribution
    std::shared_ptr<IDistribution> errorDist;
    if (settings.useTDistribution)
    {
      errorDist = std::make_shared<TDistribution>(settings.defaultNu, settings.defaultMu, settings.defaultSigma);
    }
    else
    {
      errorDist = std::make_shared<RobustNormalDistribution>(settings.defaultMu, settings.defaultSigma);
    }

    this->setErrorDistribution(errorDist);

  }

  /*  
  template 
  Frame::Frame(int id, double timestamp, unsigned char* image, std::shared_ptr<cvo::ImageRGBD<float>> color_img,  pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd,  float depth_scale, std::shared_ptr<cvo::CvoPointCloud> full_pc);
  template 
  Frame::Frame(int id, double timestamp, unsigned char* image, std::shared_ptr<cvo::ImageRGBD<uint16_t>> color_img,  pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd,  float depth_scale, std::shared_ptr<cvo::CvoPointCloud> full_pc);
  */
  
  
  Frame::~Frame()
  {
  }

  void Frame::activate()
  {
    this->status_ = Status::ACTIVE;
  }

  void Frame::deactivate()
  {
    this->status_ = Status::INACTIVE;

    if (Settings::getInstance().minimizeMemory)
    {
      this->minimizeMemory();
    }
  }

  bool Frame::isActive()
  {
    return (this->status_ == Status::ACTIVE);
  }

  int Frame::frameID() const
  {
    return this->frameID_;
  }

  int Frame::keyframeID() const
  {
    return this->keyframeID_;
  }

  void Frame::setKeyframeID(int id)
  {
    this->keyframeID_ = id;
  }

  int Frame::activeID() const
  {
    return this->activeID_;
  }

  void Frame::setActiveID(int id)
  {
    this->activeID_ = id;
  }

  double Frame::timestamp() const
  {
    return this->timestamp_;
  }

  Frame::Type Frame::type() const
  {
    return this->type_;
  }

  void Frame::evolveToKeyframe()
  {
    // change flag
    this->type_ = Frame::Type::KEYFRAME;

    // obtain global variables
    if (this->trackingParent_)
    {
      this->camToWorld_ = this->trackingParent_->camToWorld() * this->thisToParentPose_;

      this->affineLight_ = AffineLight::calcGlobal(this->trackingParent_->affineLight(),
                                                   this->thisToParentLight_);
    }	
    else
    {
      // first keyframe
      this->camToWorld_ = this->thisToParentPose_;
      this->affineLight_ = this->thisToParentLight_;
    }

    // initialize optimization data
    this->frameBlock_ = std::make_unique<FrameParameterBlock>(this->camToWorld_.cast<double>(), this->affineLight_);
  }

  void Frame::minimizeMemory()
  {
    // free memory
    this->images_->freeMemory(true);
    this->gradients_->freeMemory();
  }

  const float* Frame::image(int32_t level) const
  {
    return this->images_->image(level);
  }

  const float* Frame::gx(int32_t level) const
  {
    return this->gradients_->gx(level);
  }

  const float* Frame::gy(int32_t level) const
  {
    return this->gradients_->gy(level);
  }

  const float* Frame::gradient(int32_t level) const
  {
    return this->gradients_->gradient(level);
  }

  Frame* const Frame::parent() const
  {
    return this->trackingParent_;
  }

  const Sophus::SE3f& Frame::thisToParentPose() const
  {
    return this->thisToParentPose_;
  }

  const AffineLight& Frame::thisToParentLight() const
  {
    return this->thisToParentLight_;
  }

  void Frame::dump_candidates_to_pcd(const std::string & fname) {
	  if (candidates_.size() < 1) return;
    cvo::CvoPointCloud to_dump(candidates_[0]->features().size() ,
		    candidates_[0]->semantics().size()
		    );
    candidatesToCvoPointCloud(to_dump);
    to_dump.write_to_color_pcd(fname);
  }

  void Frame::dump_active_points_to_pcd(const std::string & fname) {
	  if (activePoints_.size() < 1) return;
    cvo::CvoPointCloud to_dump(activePoints_[0]->features().size(), 
		    activePoints_[0]->semantics().size());
    activePointsToCvoPointCloud(to_dump);
    to_dump.write_to_color_pcd(fname);
  }
  

  void Frame::setTrackingResult(Frame* const parent, const Sophus::SE3f& thisToParentPose,
                                const AffineLight& thisToParentAffineLight)
  {
    // only valid for frames
    assert(this->type_ == Type::FRAME);

    this->trackingParent_ = parent;
    this->thisToParentPose_ = thisToParentPose;
    this->thisToParentLight_ = thisToParentAffineLight;
  }	

  void Frame::setTrackingResult(Frame* const parent, const Sophus::SE3f& thisToParentPose)
  {
    // only valid for frames
    assert(this->type_ == Type::FRAME);

    this->trackingParent_ = parent;
    this->thisToParentPose_ = thisToParentPose;
    //this->thisToParentLight_ = thisToParentAffineLight;
  }	

  
  void Frame::setCamToWorld(const Sophus::SE3f& pose)
  {
    // only valid for keyframes
    assert(this->type_ == Type::KEYFRAME);

    std::lock_guard<std::mutex> lock(this->globalParamMutex_);
    this->camToWorld_ = pose;
  }

  const Sophus::SE3f& Frame::camToWorld()
  {
    // only valid for keyframes
    assert(this->type_ == Type::KEYFRAME);

    std::lock_guard<std::mutex> lock(this->globalParamMutex_);
    return this->camToWorld_;
  }

  void Frame::setAffineLight(const AffineLight& newAffineLight)
  {
    // only valid for keyframes
    assert(this->type_ == Type::KEYFRAME);

    std::lock_guard<std::mutex> lock(this->globalParamMutex_);
    this->affineLight_ = newAffineLight;
  }

  const AffineLight& Frame::affineLight()
  {
    // only valid for keyframes
    assert(this->type_ == Type::KEYFRAME);

    std::lock_guard<std::mutex> lock(this->globalParamMutex_);
    return this->affineLight_;
  }

  void Frame::setFlaggedToDrop(bool flag)
  {
    this->flaggedToDrop_ = flag;
  }

  bool Frame::flaggedToDrop() const
  {
    return this->flaggedToDrop_;
  }

  void Frame::setErrorDistribution(const std::shared_ptr<IDistribution>& dist)
  {
    const auto& settings = Settings::getInstance();
    this->errorDistribution_ = dist;
    this->energyThreshold_ = this->errorDistribution_->icdf(settings.inlierPercentile);
  }

  const std::shared_ptr<IDistribution>& Frame::errorDistribution()
  {
    return this->errorDistribution_;
  }

  float Frame::energyThreshold() const
  {
    assert(this->errorDistribution_);

    return this->energyThreshold_;
  }

  const std::vector<std::unique_ptr<CandidatePoint>>& Frame::candidates() const
  {
    return this->candidates_;
  }

  std::vector<std::unique_ptr<CandidatePoint>>& Frame::candidates()
  {
    return this->candidates_;
  }

  const std::vector<std::unique_ptr<ActivePoint>>& Frame::activePoints() const
  {
    return this->activePoints_;
  }

  std::vector<std::unique_ptr<ActivePoint>>& Frame::activePoints()
  {
    return this->activePoints_;
  }

  const std::unique_ptr<FrameParameterBlock>& Frame::frameBlock() const
  {
    return this->frameBlock_;
  }

  void Frame::mergeOptimizationResult()
  {
    this->setCamToWorld(this->frameBlock_->getPose().cast<float>());
    this->setAffineLight(this->frameBlock_->getAffineLight());
  }

  void Frame::candidatesToCvoPointCloud(cvo::CvoPointCloud & output) {
    int num_points = candidates_.size();
    if (num_points < 1) return;
    int num_features = candidates_[0]->features().size();
    int num_semantics = candidates_[0]->semantics().size();
    output.reserve(num_points, num_features, num_semantics);

    for (int i = 0; i < num_points; i++) {
      auto & p = *candidates_[i];
      Eigen::Vector3f xyz = p.xyz();
      Eigen::VectorXf features = p.features();
      Eigen::VectorXf semantics = p.semantics();
      Eigen::VectorXf geoType = p.geometricType();
      output.add_point(i, xyz, features, semantics, geoType);
    }
      
    
  }

  void Frame::initCandidateQualityFlag() {
    int num_candidates = this->candidates_.size();
    //this->candidatesHighQuaity_.resize(num_candidates, CandidatePoint::PointStatus::UNINITIALIZED);
  }
  
  void Frame::activePointsToCvoPointCloud(cvo::CvoPointCloud & output
                                          ) {

    ActivePoint::activePointsToCvoPointCloud(activePoints_, output);
    
  }

}

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

#include "LMCW.h"
#include "Log.h"
#include "DataStructures/Frame.h"
#include "DataStructures/CandidatePoint.h"
#include "DataStructures/ActivePoint.h"
#include "DataStructures/VoxelMap.h"
#include "DataStructures/Pattern.h"
#include "DataStructures/CovisibilityGraph.h"
#include "Optimization/PhotometricResidual.h"
#include "FullSystem/DistanceTransform.h"
#include "Visualizer/IVisualizer.h"
#include "Utils/Projection.h"
#include "Utils/Settings.h"
#include "Utils/UtilFunctions.h"
#include "cvo/CvoGPU.hpp"

#include <fstream>

namespace dsm
{
  LMCW::LMCW(int width, int height, IVisualizer *visualizer) :
    LMCW(width, height, nullptr, visualizer)
  {
  }

  LMCW::LMCW(int width, int height,  const cvo::CvoGPU * align, IVisualizer *visualizer)
    : cvo_align(align) ,     temporalWindowIndex(0), numActivePoints(0), outputWrapper_(visualizer) {
    const auto& settings = Settings::getInstance();
    const int levels = settings.pyramidLevels;

    // distance transformation
    this->distanceMap_ = std::make_unique<DistanceTransform>(width, height);
    this->minDistToActivate = settings.minDistToActivate;

    // covisibility graph
    this->covisibilityGraph_ = std::make_unique<CovisibilityGraph>();

    
  }

  LMCW::LMCW(int width, int height,  const cvo::CvoGPU * align, IVisualizer *visualizer, float voxelSize)
    : cvo_align(align) ,     temporalWindowIndex(0), numActivePoints(0), outputWrapper_(visualizer), 
    voxelMap_(new VoxelMap(voxelSize)) {
    const auto& settings = Settings::getInstance();
    const int levels = settings.pyramidLevels;

    // distance transformation
    this->distanceMap_ = std::make_unique<DistanceTransform>(width, height);
    this->minDistToActivate = settings.minDistToActivate;

    // covisibility graph
    this->covisibilityGraph_ = std::make_unique<CovisibilityGraph>();

    //this->voxelMap_ = std::make_unique<VoxelMap>(voxelSize);
    }

  LMCW::~LMCW()
  {}

  void LMCW::clear()
  {
    const auto& settings = Settings::getInstance();

    this->activeKeyframes_.clear();
    this->allKeyframes_.clear();

    this->temporalWindowIndex = 0;
    this->numActivePoints = 0;
    this->minDistToActivate = settings.minDistToActivate;

    this->covisibilityGraph_->clear();
  }

  void LMCW::insertNewKeyframe(const std::shared_ptr<Frame>& newKeyframe)
  {
    // convert to keyframe and activate
    newKeyframe->evolveToKeyframe();
    newKeyframe->activate();

    // insert into the LMCW
    newKeyframe->setKeyframeID((int)this->allKeyframes_.size());
    this->allKeyframes_.push_back(newKeyframe);
    newKeyframe->setActiveID((int)this->activeKeyframes_.size());
    this->activeKeyframes_.push_back(newKeyframe);

    // insert into covisibility graph
    // TODO: search through all the voxels that this new frame has observed, then create the covisibitiliy edges as well
    this->covisibilityGraph_->addNode(newKeyframe);
  }

  void LMCW::selectWindow(const std::unique_ptr<CeresPhotometricBA>& photometricBA)
  {
    const auto& settings = Settings::getInstance();

    // temporal window
    //this->selectTemporalWindowC(photometricBA);
    this->selectTemporalWindowCvo();

    // covisibility window
    if (!settings.doOnlyTemporalOpt)
    {
      // TODO: choose a subset of covisible frames with the maximum overlap
      // this->selectCovisibleWindow(photometricBA);
      this->selectCovisibleWindowCvo2();
    }
  }

  void LMCW::dropFlaggedKeyframes()
  {
    const auto& settings = Settings::getInstance();

    // remove covisible keyframes
    if (!settings.doOnlyTemporalOpt)
    {
      for (int i = 0; i < this->temporalWindowIndex; ++i)
      {
        this->activeKeyframes_.front()->deactivate();
        this->activeKeyframes_.erase(this->activeKeyframes_.begin());
      }
      this->temporalWindowIndex = 0;
    }

    // drop from temporal keyframes
    for (auto it = this->activeKeyframes_.begin(); it != this->activeKeyframes_.end(); )
    {
      if ((*it)->flaggedToDrop()) // TODO: if exceeds the max temporal kf number
      {
        (*it)->deactivate();
        (*it)->setFlaggedToDrop(false);

        // remove from keframe list
        (*it)->setActiveID(-1);
        it = this->activeKeyframes_.erase(it);
      }
      else
      {
        ++it;
      }
    }

    // set active id again
    for (int i = 0; i < this->activeKeyframes_.size(); ++i)
    {
      this->activeKeyframes_[i]->setActiveID(i);
    }
  }

  void LMCW::selectTemporalWindow(const std::unique_ptr<CeresPhotometricBA>& photometricBA)
  {
    // const configs
    const auto& settings = Settings::getInstance();
    const auto& calib = GlobalCalibration::getInstance();
    const Eigen::Matrix3f& K = calib.matrix3f(0);
    const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
    const int w = calib.width(0);
    const int h = calib.height(0);

    const std::shared_ptr<Frame>& lastKeyframe = this->activeKeyframes_.back();
    const int lastKeyframeID = lastKeyframe->keyframeID();
    const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();
    std::cout<<"selectTemporalWindow: lastKeyframe;s worldToLast is "<<worldToLast.matrix()<<std::endl;

    this->numActivePoints = 0;

    int numFlaggedToDrop = 0;

    // prepare points for new active keyframes
    for (const std::shared_ptr<Frame>& kf : this->activeKeyframes_)
    {
      //if (kf == lastKeyframe) continue;

      // relative pose
      const Sophus::SE3f relPose = worldToLast * kf->camToWorld();
      const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
      const Eigen::Vector3f Kt = K * relPose.translation();

      const AffineLight light = AffineLight::calcRelative(kf->affineLight(), lastKeyframe->affineLight());

      int numVisible = 0;

      for (const auto& point : kf->activePoints())
      {
        // project point to new image
        Eigen::Vector2f pt2d;
        if (!Utils::project(point->u(0), point->v(0), point->iDepth(),
                            w, h, KRKinv, Kt, pt2d))
        {
          point->setVisibility(lastKeyframeID, Visibility::OOB);
          continue;
        }

        // it is visible!
        point->setCenterProjection(pt2d);
        point->setVisibility(lastKeyframeID, Visibility::VISIBLE);

        // create new observation to the newest keyframe
        std::unique_ptr<PhotometricResidual> obs =
          std::make_unique<PhotometricResidual>(point, lastKeyframe, photometricBA);

        point->addObservation(lastKeyframe.get(), obs);

        // increase counters
        this->numActivePoints++;
        numVisible++;
      }

      // keep the last numAlwaysKeepKeyframes keyframes + the new one
      if ((kf->keyframeID() <= lastKeyframeID - settings.numAlwaysKeepKeyframes) &&
          (this->activeKeyframes_.size() - numFlaggedToDrop) >= settings.maxTemporalKeyframes)
      {
        const float numVisibleFloat = (float)numVisible;
        const float ratio = numVisibleFloat / kf->activePoints().size();

        // drop keyframes with low covisible points
        if (ratio < settings.minPointCovisible || fabs(light.alpha()) > settings.maxLightCovisible)
        {
          kf->setFlaggedToDrop(true);
          numFlaggedToDrop++;
        }
      }
    }

    // if still a lot of keyframes, drop one based on distance
    if ((this->activeKeyframes_.size() - numFlaggedToDrop) > settings.maxTemporalKeyframes)
    {
      float maxScore = -std::numeric_limits<float>::max();
      int idx = -1;

      // keep the last N keyframes
      //const int maxKeyframeID = (int)this->activeKeyframes_.size() - settings.numAlwaysKeepKeyframes;
      const int maxKeyframeID = (int)this->activeKeyframes_.size() - settings.maxTemporalKeyframes;
      for (int i = 0; i < maxKeyframeID; ++i)
      {
        this->activeKeyframes_[i]->setFlaggedToDrop(true);
        numFlaggedToDrop++;
        /*
        const Sophus::SE3f pose_i = this->activeKeyframes_[i]->camToWorld().inverse();

        // distance to other keyframes
        float score = 0.f;
        for (int j = 0; j < this->activeKeyframes_.size() - 1; ++j)
        {
          if (i == j) continue;

          const Sophus::SE3f relPose = pose_i * this->activeKeyframes_[j]->camToWorld();
          const float dist = relPose.translation().norm() + std::numeric_limits<float>::epsilon();
          score += 1.f / dist;
        }

        // distance to the latest keyframe
        const Sophus::SE3f relPose = pose_i * lastKeyframe->camToWorld();
        score *= sqrtf(relPose.translation().norm());

        if (score > maxScore)
        {
          maxScore = score;
          idx = i;
        }
        */
      }

      //if (idx >= 0)
      // {
      //  this->activeKeyframes_[idx]->setFlaggedToDrop(true);
      //  numFlaggedToDrop++;
      //}
    }

    if (settings.debugPrintLog && settings.debugLogActivePoints)
    {
      const std::string msg = "Act. Temporal: " + std::to_string(this->numActivePoints) + "\t";

      auto& log = Log::getInstance();
      log.addCurrentLog(lastKeyframe->frameID(), msg);
    }

    // distance map
    this->distanceMap_->compute(this->activeKeyframes_, lastKeyframe);

    if (settings.debugPrintLog && settings.debugLogDistanceMap)
    {
      const std::string msg = "DT: " + std::to_string(this->distanceMap_->getNumObstacles()) + "\t";

      auto& log = Log::getInstance();
      log.addCurrentLog(lastKeyframe->frameID(), msg);
    }

    if (settings.debugShowDistanceTransformBefore &&
        settings.doOnlyTemporalOpt &&
        this->outputWrapper_)
    {
      cv::Mat distTransform = this->distanceMap_->drawDistanceTransform(true);
      this->outputWrapper_->publishDistanceTransformBefore(distTransform);
    }
  }

  void LMCW::selectTemporalWindowCvo()
  {
    // const configs
    const auto& settings = Settings::getInstance();
    const auto& calib = GlobalCalibration::getInstance();
    const Eigen::Matrix3f& K = calib.matrix3f(0);
    const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
    const int w = calib.width(0);
    const int h = calib.height(0);

    const std::shared_ptr<Frame>& lastKeyframe = this->activeKeyframes_.back();
    const int lastKeyframeID = lastKeyframe->keyframeID();
    const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();
    std::cout<<"selectTemporalWindow: lastKeyframe;s worldToLast is "<<worldToLast.matrix()<<std::endl;

    this->numActivePoints = 0;

    int numFlaggedToDrop = 0;

    // prepare points for new active keyframes
    for (const std::shared_ptr<Frame>& kf : this->activeKeyframes_)
    {
      if (kf == lastKeyframe) continue;

      // relative pose
      const Sophus::SE3f relPose = worldToLast * kf->camToWorld();
      const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
      const Eigen::Vector3f Kt = K * relPose.translation();

      const AffineLight light = AffineLight::calcRelative(kf->affineLight(), lastKeyframe->affineLight());

      int numVisible = 0;

      for (const auto& point : kf->activePoints())
      {
        // project point to new image
        Eigen::Vector2f pt2d;
        if (!Utils::project(point->u(0), point->v(0), point->iDepth(),
                            w, h, KRKinv, Kt, pt2d))
        {
          point->setVisibility(lastKeyframeID, Visibility::OOB);
          continue;
        }

        // it is visible!
        point->setCenterProjection(pt2d);
        point->setVisibility(lastKeyframeID, Visibility::VISIBLE);

        // create new observation to the newest keyframe
        //std::unique_ptr<PhotometricResidual> obs =
        //  std::make_unique<PhotometricResidual>(point, lastKeyframe, photometricBA);

        //point->addObservation(lastKeyframe.get(), obs);

        // increase counters
        this->numActivePoints++;
        numVisible++;
      }
      std::cout<<"Frame "<<kf->frameID()<<" has "<<numVisible<<" active points that are visible.\n";

      // keep the last numAlwaysKeepKeyframes keyframes + the new one
      if ((kf->keyframeID() <= lastKeyframeID - settings.numAlwaysKeepKeyframes) &&
          (this->activeKeyframes_.size() - numFlaggedToDrop) >= settings.maxTemporalKeyframes)
      {
        const float numVisibleFloat = (float)numVisible;
        const float ratio = numVisibleFloat / kf->activePoints().size();

        // drop keyframes with low covisible points
        if (ratio < settings.minPointCovisible || fabs(light.alpha()) > settings.maxLightCovisible)
        {
          kf->setFlaggedToDrop(true);
          numFlaggedToDrop++;
        }
      }
    }

    // if still a lot of keyframes, drop one based on distance
    if ((this->activeKeyframes_.size() - numFlaggedToDrop) > settings.maxTemporalKeyframes)
    {
      float maxScore = -std::numeric_limits<float>::max();
      int idx = -1;

      // keep the last N keyframes
      //const int maxKeyframeID = (int)this->activeKeyframes_.size() - settings.numAlwaysKeepKeyframes;
      const int maxKeyframeID = (int)this->activeKeyframes_.size() - settings.maxTemporalKeyframes;
      for (int i = 0; i < maxKeyframeID; ++i)
      {
        this->activeKeyframes_[i]->setFlaggedToDrop(true);
        numFlaggedToDrop++;
        /*
        const Sophus::SE3f pose_i = this->activeKeyframes_[i]->camToWorld().inverse();

        // distance to other keyframes
        float score = 0.f;
        for (int j = 0; j < this->activeKeyframes_.size() - 1; ++j)
        {
          if (i == j) continue;

          const Sophus::SE3f relPose = pose_i * this->activeKeyframes_[j]->camToWorld();
          const float dist = relPose.translation().norm() + std::numeric_limits<float>::epsilon();
          score += 1.f / dist;
        }

        // distance to the latest keyframe
        const Sophus::SE3f relPose = pose_i * lastKeyframe->camToWorld();
        score *= sqrtf(relPose.translation().norm());

        if (score > maxScore)
        {
          maxScore = score;
          idx = i;
        }
        */
      }

      //if (idx >= 0)
      // {
      //  this->activeKeyframes_[idx]->setFlaggedToDrop(true);
      //  numFlaggedToDrop++;
      //}
    }

    if (settings.debugPrintLog && settings.debugLogActivePoints)
    {
      const std::string msg = "Act. Temporal: " + std::to_string(this->numActivePoints) + "\t";

      auto& log = Log::getInstance();
      log.addCurrentLog(lastKeyframe->frameID(), msg);
    }

    // distance map
    this->distanceMap_->compute(this->activeKeyframes_, lastKeyframe);

    if (settings.debugPrintLog && settings.debugLogDistanceMap)
    {
      const std::string msg = "DT: " + std::to_string(this->distanceMap_->getNumObstacles()) + "\t";

      auto& log = Log::getInstance();
      log.addCurrentLog(lastKeyframe->frameID(), msg);
    }

    if (settings.debugShowDistanceTransformBefore &&
        settings.doOnlyTemporalOpt &&
        this->outputWrapper_)
    {
      cv::Mat distTransform = this->distanceMap_->drawDistanceTransform(true);
      this->outputWrapper_->publishDistanceTransformBefore(distTransform);
    }
  }

  
  void LMCW::selectCovisibleWindow(const std::unique_ptr<CeresPhotometricBA>& photometricBA)

  {
    assert(this->temporalWindowIndex == 0);

    const auto& settings = Settings::getInstance();
    const auto& calib = GlobalCalibration::getInstance();

    const Eigen::Matrix3f& K = calib.matrix3f(0);
    const int w = calib.width(0);
    const int h = calib.height(0);

    const std::shared_ptr<Frame> lastActKeyframe = this->activeKeyframes_.back();		// make a copy, required!
    const Sophus::SE3f worldToLast = lastActKeyframe->camToWorld().inverse();
    const int lastActID = lastActKeyframe->keyframeID();
    const int firstActID = this->activeKeyframes_.front()->keyframeID();

    // minimum distance based on pattern padding
    const float padding = (float)Pattern::padding();
    const float minDist = padding * padding; // squared dist

    std::vector<std::shared_ptr<Frame>> allCovisibleKeyframes;
    std::vector<int> numVisiblePoints;
    std::vector<int> numTotalVisiblePoints;

    // select some covisible keyframes
    for (const auto& kf : this->allKeyframes_)
    {
      // break if we have already reached the temporal part
      if (kf->keyframeID() >= firstActID) break;

      // relative pose
      const Sophus::SE3f kfToLast = worldToLast * kf->camToWorld();
      const Eigen::Matrix3f R = kfToLast.rotationMatrix();
      const Eigen::Vector3f t = kfToLast.translation();
      Eigen::Matrix4f kfToLastEigen = Utils::SE3ToEigen(kfToLast);

      int numVisible = 0;
      int numTotalVisible = 0;

      for (const auto& point : kf->activePoints())
      {
        // project point to new image
        Eigen::Vector2f pt2d;
        if (!Utils::projectAndCheck(point->u(0), point->v(0), point->iDepth(),
                                    K, w, h, R, t, pt2d))
        {
          point->setVisibility(lastActID, Visibility::OOB);
          continue;
        }

        // activate point
        point->setCenterProjection(pt2d);
        point->setVisibility(lastActID, Visibility::VISIBLE);

        numTotalVisible++;

        // check distance
        int x = static_cast<int>(pt2d[0] + 0.5f);
        int y = static_cast<int>(pt2d[1] + 0.5f);

        if (this->distanceMap_->dist(x, y) < minDist) continue;

        // increase counter
        numVisible++;
      }

      float numVisibleFloat = (float)numVisible;
      float ratio = numVisibleFloat / kf->activePoints().size();

      if (ratio > settings.minPointCovisible)
      {
        allCovisibleKeyframes.push_back(kf);
        numVisiblePoints.push_back(numVisible);
        numTotalVisiblePoints.push_back(numTotalVisible);

        if (cvo_align != nullptr ) {
          float ip = cvo_align->function_angle(*(kf->getTrackingPoints()),
                                               *(lastActKeyframe->getTrackingPoints()),
                                               kfToLastEigen,
                                               false);
        std::cout<<"Covisible projection ratio "<<ratio <<", with inner product between "<<kf->frameID()<<" and "<<lastActKeyframe->frameID()<<" being "<<ip<<", Add frame "<<kf->frameID()<<" to covisible window\n";
        }
      }
    }

    int numCovisibleKeyframes = (int)allCovisibleKeyframes.size();

    while (this->temporalWindowIndex < settings.maxCovisibleKeyframes)
    {
      if (numCovisibleKeyframes == 0) break;

      // select the keyframes with most visible points
      const auto maxElement = std::max_element(numVisiblePoints.begin(), numVisiblePoints.end());
      const int position = (int)std::distance(numVisiblePoints.begin(), maxElement);
			
      // pick the one with highest number of visible points
      const std::shared_ptr<Frame>& selectedKeyframe = allCovisibleKeyframes[position];
      selectedKeyframe->activate();
      this->activeKeyframes_.insert(this->activeKeyframes_.begin(), selectedKeyframe);
      this->numActivePoints += numTotalVisiblePoints[position];
      this->temporalWindowIndex++;
			
      // update map
      this->distanceMap_->add(selectedKeyframe, lastActKeyframe);
      std::cout<<"LMCW add frame "<<selectedKeyframe->frameID()<<", number of Visible points is "<<numVisiblePoints[position]<<std::endl;
			
      // remove from vectors
      allCovisibleKeyframes[position] = allCovisibleKeyframes.back();
      allCovisibleKeyframes.pop_back();
      numVisiblePoints[position] = numVisiblePoints.back();
      numVisiblePoints.pop_back();
      numTotalVisiblePoints[position] = numTotalVisiblePoints.back();
      numTotalVisiblePoints.pop_back();
      numCovisibleKeyframes--;

      // early exit
      if (this->temporalWindowIndex == settings.maxCovisibleKeyframes) break;

      // update num visible
      for (int i = 0; i < numCovisibleKeyframes; ++i)
      {
        numVisiblePoints[i] = 0;

        const auto& kf = allCovisibleKeyframes[i];
        for (auto& pt : kf->activePoints())
        {
          if (pt->visibility(lastActID) == Visibility::VISIBLE)
          {
            // check distance
            const Eigen::Vec2f& pt2d = pt->centerProjection();
            int x = static_cast<int>(pt2d[0] + 0.5f);
            int y = static_cast<int>(pt2d[1] + 0.5f);

            if (this->distanceMap_->dist(x, y) < minDist) continue;

            numVisiblePoints[i]++;
          }
        }
      }
    }

    if (this->temporalWindowIndex > 0)
    {
      // create new observations to latest keyframe
      for (int i = 0; i < this->temporalWindowIndex; ++i)
      {
        for (const auto& point : this->activeKeyframes_[i]->activePoints())
        {
          if (point->visibility(lastActID) == Visibility::VISIBLE)
          {
            std::unique_ptr<PhotometricResidual> obs =
              std::make_unique<PhotometricResidual>(point, lastActKeyframe, photometricBA);

            // add residual
            point->addObservation(lastActKeyframe.get(), obs);
          }
        }
      }

      // set active id again
      for (int i = 0; i < this->activeKeyframes_.size(); ++i)
      {
        this->activeKeyframes_[i]->setActiveID(i);
      }
    }

    if (settings.debugShowDistanceTransformBefore &&
        !settings.doOnlyTemporalOpt &&
        this->outputWrapper_)
    {
      cv::Mat distTransform = this->distanceMap_->drawDistanceTransform(true);
      this->outputWrapper_->publishDistanceTransformBefore(distTransform);
    }
  }

  std::unordered_map<int, int> LMCW::selectCovisibleWindowCvo()
  {
    assert(this->temporalWindowIndex == 0);

    // const int firstActID = this->activeKeyframes_.front()->keyframeID();
    // for (const auto& kf : this->allKeyframes_)
    // {
    //   if (kf->keyframeID() >= firstActID) break;

    //   for (const auto& point : kf->activePoints())
    //   {

    //   }
    // }
    Utils::Time t1 = std::chrono::steady_clock::now();
    
    const auto& settings = Settings::getInstance();
    const auto& calib = GlobalCalibration::getInstance();
    const Eigen::Matrix3f& K = calib.matrix3f(0);
    const int w = calib.width(0);
    const int h = calib.height(0);
    
    const int firstKeyframeID = this->activeKeyframes_.front()->keyframeID();
    //const int firstFrameID = this->activeKeyframes_[this->temporalWindowIndex]->frameID();
    //const int maxCovisibleFrameID = this->activeKeyframes_.front()->frameID();
    std::unordered_set<std::shared_ptr<Frame>> selectCovisibleKeyframes;
    std::unordered_map<int, int> edgesCovisibleToTemporal;

    for (int i = this->temporalWindowIndex; i < activeKeyframes_.size(); i++)
    {
      // find best covisible frame for each temporal frame
      const CovisibilityNode* temporalNode = this->activeKeyframes_[i]->graphNode;
      // get the node with highest weight among all edges
      //CovisibilityNode* highCovisNode = nullptr;
      //int maxWeight = 0;
      std::cout<<"selectedCovis temp-window: frame "<<this->activeKeyframes_[i]->frameID()<<" has "<<temporalNode->edges.size()<<" neighbors in the covis-graph: ";
      std::vector<CovisibilityNode *> sorted_nodes;
      sorted_nodes.reserve(temporalNode->edges.size());
      for (const auto &s : temporalNode->edges) {
        std::cout<<s.first->node->frameID()<<", ";
        sorted_nodes.emplace_back(s.first);
      }
      std::cout<<std::endl<<std::flush;
      std::sort(sorted_nodes.begin(), sorted_nodes.end(),
                [&](CovisibilityNode * a, CovisibilityNode *b) {
                  return temporalNode->edges.at(a) > temporalNode->edges.at(b);
                });
      /*
      for (auto& pair : sorted_nodes) 
      {
        std::cout<<pair.first->node->frameID()<<", ";
          
        if (//pair.first->node->keyframeID() >= firstTempID ||
            (int)pair.first->node->frameID() >= (int)firstFrameID - settings.gapCovisibleToTemporal ) continue; // disregard temporal frames
        if (pair.second > maxWeight)
        {
          maxWeight = pair.second;
          highCovisNode = pair.first;
        }
        }
      std::cout<<"\n";
      */
      //if (highCovisNode == nullptr) continue;
      //std::shared_ptr<Frame> bestCovisFrame = this->allKeyframes_[highCovisNode->node->keyframeID()];
      
      // ensure no duplicates
      for (auto & node : sorted_nodes) {
        std::shared_ptr<Frame> bestCovisFrame = this->allKeyframes_[node->node->keyframeID()];
        if (!selectCovisibleKeyframes.count(bestCovisFrame)
            && bestCovisFrame->frameID() < activeKeyframes_.front()->frameID() - settings.gapCovisibleToTemporal
            ) {
          
          bestCovisFrame->activate();
          //edgesCovisibleToTemporal[bestCovisFrame->frameID()] = this->activeKeyframes_[i]->frameID();
          edgesCovisibleToTemporal.insert(std::make_pair<int, int>(bestCovisFrame->frameID(),
                                                                   this->activeKeyframes_[i]->frameID()));
          
          
          std::cout<<"select covisible frame: ("<<bestCovisFrame->frameID()<<", "<<this->activeKeyframes_[i]->frameID()<<")"<<std::endl;
          // Q: need to update this->numActivePoints?
          selectCovisibleKeyframes.insert((bestCovisFrame));
          break;
          
        }
      }
    }

    if (selectCovisibleKeyframes.empty()) return edgesCovisibleToTemporal;

    // set visibility for visualization
    const std::shared_ptr<Frame> lastActKeyframe = this->activeKeyframes_.back();		// make a copy, required!
    const Sophus::SE3f worldToLast = lastActKeyframe->camToWorld().inverse();
    const int lastActID = lastActKeyframe->keyframeID();
    for (auto & covisFrame : selectCovisibleKeyframes) {
      // relative pose
      const Sophus::SE3f covisToLast = worldToLast * covisFrame->camToWorld();
      const Eigen::Matrix3f R = covisToLast.rotationMatrix();
      const Eigen::Vector3f t = covisToLast.translation();
      Eigen::Matrix4f covisToLastEigen = Utils::SE3ToEigen(covisToLast);

      for (const auto& point : covisFrame->activePoints())
      {
        // project point to new image
        Eigen::Vector2f pt2d;
        if (!Utils::projectAndCheck(point->u(0), point->v(0), point->iDepth(),
                                    K, w, h, R, t, pt2d))
        {
          point->setVisibility(lastActID, Visibility::OOB);
          continue;
        }
      
        /*
      std::shared_ptr<Frame> currTemporal = this->activeKeyframes_[i];        
      const Sophus::SE3f covisToTemporal = currTemporal->camToWorld().inverse() * bestCovisFrame->camToWorld();
      const Eigen::Matrix3f KRKinv = K * covisToTemporal.rotationMatrix() * Kinv;
      const Eigen::Vector3f Kt = K * covisToTemporal.translation();

      for (const auto& point : currTemporal->activePoints()) {
        // project point to new image
        Eigen::Vector2f pt2d;
        if (!Utils::project(point->u(0), point->v(0), point->iDepth(),
                            w, h, KRKinv, Kt, pt2d)) {
          point->setVisibility(lastKeyframeID, Visibility::OOB);
          continue;
        }
        */
        // it is visible!
        point->setCenterProjection(pt2d);
        point->setVisibility(lastActID, Visibility::VISIBLE);
      }

      
    }

    this->activeKeyframes_.insert(this->activeKeyframes_.begin(), selectCovisibleKeyframes.begin(), selectCovisibleKeyframes.end());
    this->temporalWindowIndex += selectCovisibleKeyframes.size();

    for (int i = 0; i < this->activeKeyframes_.size(); i++)
    {
      this->activeKeyframes_[i]->setActiveID(i);
    }

    Utils::Time t2 = std::chrono::steady_clock::now();

    std::ofstream file;
    file.open("covisResult.txt", std::ios_base::app);
    file << "Time: " << Utils::elapsedTime(t1, t2) << "\n";
    file << "Temporal frames: \n";
    for (int i = this->temporalWindowIndex; i < activeKeyframes_.size(); i++)
    {
      int frameID = this->activeKeyframes_[i]->frameID();
      file << frameID << ", ";
    }
    file << "\nCovisible frames: \n";
    for (int i = 0; i < this->temporalWindowIndex; i++)
    {
      int frameID = this->activeKeyframes_[i]->frameID();
      file << frameID << ", ";
    }
    //file << "\nAll candidates: \n";
    //for (auto it = edgesCovisibleToTemporal.begin(); it != edgesCovisibleToTemporal.end(); it++)
    //{
    //  int frameID = it->first->frameID();
    //  file << frameID << ": " << it->second << " < ";
    // }

    file << "\n==============================================\n";
    file.close();


    
    return edgesCovisibleToTemporal;
  }


  void LMCW::selectCovisibleWindowCvo2()
  {
    assert(this->temporalWindowIndex == 0);

    const auto& settings = Settings::getInstance();

    const int firstTempID = this->activeKeyframes_[this->temporalWindowIndex]->keyframeID();

    const int ignoreRecent = 5;

    std::unordered_map<std::shared_ptr<Frame>, int> selectCovisibleKeyframes;

    Utils::Time t1 = std::chrono::steady_clock::now();

    for (int i = this->temporalWindowIndex; i < activeKeyframes_.size(); i++)
    {
      const CovisibilityNode* temporalNode = this->activeKeyframes_[i]->graphNode;
      
      for (auto& pair : temporalNode->edges) 
      {
        if (pair.first->node->keyframeID() >= firstTempID - ignoreRecent) continue; // disregard temporal frames
        // accumulate weights to everyone covisible candidate frame
        const std::shared_ptr<Frame> curCovisFrame = this->allKeyframes_[pair.first->node->keyframeID()];
        selectCovisibleKeyframes[curCovisFrame] += pair.second;
      }
    }

    if (selectCovisibleKeyframes.empty()) return;


    // extract k out of the set.
    std::vector<std::pair<std::shared_ptr<Frame>, int>> selectCovisVec(selectCovisibleKeyframes.begin(), selectCovisibleKeyframes.end());
    std::sort(selectCovisVec.begin(), selectCovisVec.end(), [](const std::pair<std::shared_ptr<Frame>, int>& a, const std::pair<std::shared_ptr<Frame>, int>& b) {
                                                              return a.second > b.second;
                                                              });
    
    const int numCovisFrame = std::min(settings.maxCovisibleKeyframes, (int)selectCovisibleKeyframes.size());
    std::vector<std::shared_ptr<Frame>> topCovis;
    for (int i = 0; i < numCovisFrame; i++)
    {
      topCovis.push_back(selectCovisVec[i].first);
    }
    
    this->activeKeyframes_.insert(this->activeKeyframes_.begin(), topCovis.begin(), topCovis.end());

    this->temporalWindowIndex += numCovisFrame;

    for (int i = 0; i < this->activeKeyframes_.size(); i++)
    {
      this->activeKeyframes_[i]->setActiveID(i);
    }

    Utils::Time t2 = std::chrono::steady_clock::now();

    // TL: debug log
    std::ofstream file;
    file.open("covisResult.txt", std::ios_base::app);
    file << "Time: " << Utils::elapsedTime(t1, t2) << "\n";
    file << "Temporal frames: \n";
    for (int i = this->temporalWindowIndex; i < activeKeyframes_.size(); i++)
    {
      int frameID = this->activeKeyframes_[i]->frameID();
      file << frameID << ", ";
    }
    file << "\nCovisible frames: \n";
    for (int i = 0; i < this->temporalWindowIndex; i++)
    {
      int frameID = this->activeKeyframes_[i]->frameID();
      file << frameID << ", ";
    }
    file << "\nAll candidates: \n";
    for (auto it = selectCovisVec.begin(); it != selectCovisVec.end(); it++)
    {
      int frameID = it->first->frameID();
      file << frameID << ": " << it->second << " < ";
    }

    file << "\n==============================================\n";
    file.close();
  }


  void LMCW::activatePoints(const std::unique_ptr<CeresPhotometricBA>& photometricBA)
  {
    const auto& settings = Settings::getInstance();
    const auto& calib = GlobalCalibration::getInstance();

    const Eigen::Matrix3f& K = calib.matrix3f(0);
    const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
    const int width = (int)calib.width(0);
    const int height = (int)calib.height(0);

    // change heuristically the minimum distance
    const float ratio = (float)this->numActivePoints / settings.numActivePoints;

    if (ratio > 1.7f) this->minDistToActivate = this->minDistToActivate + 3;
    else if (ratio > 1.4f) this->minDistToActivate = this->minDistToActivate + 2;
    else if (ratio > 1.1f) this->minDistToActivate = this->minDistToActivate + 1;
		
    if (ratio < 0.3f) this->minDistToActivate = this->minDistToActivate - 3;
    else if (ratio < 0.6f) this->minDistToActivate = this->minDistToActivate - 2;
    else if (ratio < 0.9f) this->minDistToActivate = this->minDistToActivate - 1;

    this->minDistToActivate = std::max(this->minDistToActivate, 1);
    this->minDistToActivate = std::min(this->minDistToActivate, 2*Pattern::width());

    const float minDist = (float)this->minDistToActivate*this->minDistToActivate; // squared dist

    const std::shared_ptr<Frame>& lastKeyframe = this->activeKeyframes_.back();
    const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();

    // select active points from candidates
    int numPointsCreated = 0;

    Utils::Time t1 = std::chrono::steady_clock::now();
    int numActiveKeyframes = (int)this->activeKeyframes_.size();

    for (int i = this->temporalWindowIndex; i < numActiveKeyframes - 1; ++i)
    {
      std::cout<<"activePoint: new iteration "<<i<<std::endl<<std::flush;
      const std::shared_ptr<Frame>& owner = this->activeKeyframes_[i];
      //std::shared_ptr<Frame> owner = this->activeKeyframes_[i];

      // relative pose
      const Sophus::SE3f ownerToLast = worldToLast * owner->camToWorld();	
      const Eigen::Matrix3f KRKinv = K * ownerToLast.rotationMatrix() * Kinv;
      const Eigen::Vector3f Kt = K * ownerToLast.translation();

      auto& candidates = owner->candidates();
      auto& activePoints = owner->activePoints();

      for (auto& cand : candidates)
      {
        CandidatePoint::PointStatus status = cand->status();

        // check status
        if (status == CandidatePoint::OPTIMIZED )
        {
          // project into new keyframe
          Eigen::Vector2f pointInFrame;
          if (!Utils::project(cand->u0(), cand->v0(), cand->iDepth(),
                              width, height, KRKinv, Kt, pointInFrame))
          {
            cand = nullptr;
            continue;
          }

          int x = static_cast<int>(pointInFrame[0] + 0.5f);
          int y = static_cast<int>(pointInFrame[1] + 0.5f);

          // check squared distance
          float level = ((float)cand->detectedLevel() + 1.f);
          if (this->distanceMap_->dist(x, y) < (minDist * level * level)) continue;

          // activePoint from candidate
          std::unique_ptr<ActivePoint> point = std::make_unique<ActivePoint>(lastKeyframe->keyframeID(), cand);

          // observations & visibility
          for (const std::shared_ptr<Frame>& frame : this->activeKeyframes_)
          {
            if (frame == owner) continue;

            Visibility vis = cand->visibility(frame->activeID());

            // set visibility
            point->setVisibility(frame->keyframeID(), vis);

            if (vis == Visibility::VISIBLE)
            {
              // create new observation
              std::unique_ptr<PhotometricResidual> obs =
                std::make_unique<PhotometricResidual>(point, frame, photometricBA);

              point->addObservation(frame.get(), obs);
            }
          }

          cand = nullptr;						// delete candidate after activation

          point->setCenterProjection(pointInFrame);

          numPointsCreated++;
          this->numActivePoints++;

          // update distance map
          this->distanceMap_->add(point, lastKeyframe);

          // insert into list
          activePoints.push_back(std::move(point));
        }
        else if (status == CandidatePoint::OUTLIER)
        {
          cand = nullptr;
          continue;
        }
      }

      // reorder candidates filling the gaps
      for (int j = 0; j < candidates.size(); ++j)
      {
        if (candidates[j] == nullptr)
        {
          candidates[j] = std::move(candidates.back());
          candidates.pop_back();
          j--;						// go back again to check if last one was nullptr too
        }
        //std::cout<<j<<std::endl<<std::flush;
      }
      std::cout<<"next frame\n"<<std::flush;
    }
    Utils::Time t2 = std::chrono::steady_clock::now();

    if (settings.debugPrintLog && settings.debugLogActivePoints)
    {
      const std::string msg = "Act. New: " + std::to_string(numPointsCreated) + "\t";

      auto& log = Log::getInstance();
      log.addCurrentLog(lastKeyframe->frameID(), msg);
    }

    if (settings.debugShowDistanceTransformAfter && this->outputWrapper_)
    {
      cv::Mat distTransform = this->distanceMap_->drawDistanceTransform(true);
      this->outputWrapper_->publishDistanceTransformAfter(distTransform);
    }
  }
  void LMCW::activatePointsCvo()
  {
    // TL: log points
    // std::ofstream file;
    // file.open("/home/tannerliu/dsm/points.txt", std::ios_base::app);
    
    const auto& settings = Settings::getInstance();
    const auto& calib = GlobalCalibration::getInstance();

    const Eigen::Matrix3f& K = calib.matrix3f(0);
    const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
    const int width = (int)calib.width(0);
    const int height = (int)calib.height(0);

    Utils::Time t1 = std::chrono::steady_clock::now();
    int numActiveKeyframes = (int)this->activeKeyframes_.size();

    // activate the old keyframes' (excluding the lastest one) candidates
    // whose status is 'OPTIMIZED'
    // Q: why excluding the latest?
    int numPointsCreated = 0;
    auto lastKeyframe = this->activeKeyframes_.back();
    const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();
    
    for (int i = this->temporalWindowIndex; i < numActiveKeyframes-1; ++i)
    {
      std::cout<<"activePoint: frame ID "<<activeKeyframes_[i]->frameID()<<std::endl<<std::flush;
      const std::shared_ptr<Frame>& owner = this->activeKeyframes_[i];
      //std::shared_ptr<Frame> owner = this->activeKeyframes_[i];

      // relative pose
      const Sophus::SE3f ownerToLast = worldToLast * owner->camToWorld();	
      const Eigen::Matrix3f KRKinv = K * ownerToLast.rotationMatrix() * Kinv;
      const Eigen::Vector3f Kt = K * ownerToLast.translation();

      auto& candidates = owner->candidates();
      auto& activePoints = owner->activePoints();

      int counter = 0;
      for (auto& cand : candidates)
      {
        CandidatePoint::PointStatus status = cand->status();

        // check status
        if (status == CandidatePoint::OPTIMIZED )
        {
          // project into new keyframe
          std::unique_ptr<ActivePoint> point;

          if (settings.enableDepthRegression) {
            float filteredIdepth = cand->regressIdepth();
            std::cout<<"regressIdepth: num observations is "<<cand->observedIdepths().size()<<", before: "<<cand->iDepth()<<", after: "<<filteredIdepth<<std::endl;
            
            point.reset(new ActivePoint (lastKeyframe->keyframeID(), cand, filteredIdepth));
          } else
            point.reset(new ActivePoint (lastKeyframe->keyframeID(), cand));          
          counter++;

          // observations & visibility
          for (const std::shared_ptr<Frame>& frame : this->activeKeyframes_)
          {
            if (frame == owner) continue;
            Eigen::Vector2f pt2d;
            const Sophus::SE3f covisToLast = worldToLast * frame->camToWorld();
            const Eigen::Matrix3f R = covisToLast.rotationMatrix();
            const Eigen::Vector3f t = covisToLast.translation();
            Eigen::Matrix4f covisToLastEigen = Utils::SE3ToEigen(covisToLast);
            if (!Utils::projectAndCheck(point->u(0), point->v(0), point->iDepth(),
                                        K, width, height, R, t, pt2d)) {
              point->setVisibility(frame->keyframeID() , Visibility::OOB);
              continue;
            }

            //Visibility vis = cand->visibility(frame->activeID());

            // set visibility
            point->setVisibility(frame->keyframeID(), Visibility::VISIBLE);

            //if (vis == Visibility::VISIBLE)
            {
              // create new observation
              //std::unique_ptr<PhotometricResidual> obs =
              //  std::make_unique<PhotometricResidual>(point, frame, photometricBA);

              //point->addObservation(frame.get(), obs);
            }
          }
          cand = nullptr;						// delete candidate after activation

          //point->setCenterProjection(pointInFrame);

          numPointsCreated++;
          this->numActivePoints++;

          // update distance map
          //this->distanceMap_->add(point, lastKeyframe);



          // update covisibilityGraph_
          const Voxel* tracedVoxel = voxelMap_->query_point(point.get());
          if (tracedVoxel)
          {
            // voxel exists, add edges in covisGraph
            // add current frame as node to the covisGraph
            if (owner->graphNode == nullptr) 
            {
              covisibilityGraph_->addNode(owner);
            }
            CovisibilityNode* curNode = owner->graphNode;
            for (ActivePoint* prevFramePt : tracedVoxel->voxPoints) 
            {
              // if prevFrame is already a temporal frame, don't skip
              // if (prevFramePt->currentID() > this->activeKeyframes_.front()->keyframeID()) continue;
              // trace refKF of an ActivePoint
              const std::shared_ptr<Frame> covisFrame = allKeyframes_[prevFramePt->currentID()];  // Q: correct?
              // if prevFrame and owner are the same frame, skip
              if (prevFramePt->currentID() == owner->keyframeID()) continue;
              // refKF should already have a CovisibilityNode
              CovisibilityNode* refNode = covisFrame->graphNode;
              // intialize or update weight
              int newWeight = curNode->edges[refNode] + 1; // unordered_map should return 0 if doesn't exist, the newWeight will be 1
              covisibilityGraph_->connect(owner, covisFrame, newWeight);
              
            }
          }



          // insert to voxel map
          voxelMap_->insert_point(point.get());
          point->setVoxel(voxelMap_->query_point(point.get()));

          // TL: testing
          // std::cout <<"Writing point to file: " << pt << std::endl;
          // Eigen::Matrix4f Tcw = point->reference()->camToWorld().matrix(); //camToWorld
          // Eigen::Vector3f p_cam = point->xyz(); // pt in cam frame
          // Eigen::Vector4f p_cam_4;
          // p_cam_4 << p_cam(0), p_cam(1), p_cam(2), 1.0;
          // Eigen::Vector4f p_wld = Tcw * p_cam_4;
          // if (point->currentID() == 1) {
          //   voxelMap_->insert_point(point.get());
          // }
          // else if (point->currentID() == 2 || point->currentID() == 3) {
          //   Voxel qVox;
          //   if (voxelMap_->query_point(point.get(), qVox)) {
          //     // TL: log points
          //     file << p_wld(0) << ", " << p_wld(1) << ", " << p_wld(2) << ", " << point->currentID() << "\n";
          //   }
          // }
          // std::cout << "Voxel Map size: " << voxelMap_->size() << std::endl;


          // insert into list
           activePoints.push_back(std::move(point));

        }
        //else if (status == CandidatePoint::OUTLIER)
        // {
        //  cand = nullptr;
        // continue;
        //}
      }

      // reorder candidates filling the gaps
      for (int j = 0; j < candidates.size(); ++j)
      {
        if (candidates[j] == nullptr)
        {
          candidates[j] = std::move(candidates.back());
          candidates.pop_back();
          j--;						// go back again to check if last one was nullptr too
        }
        //std::cout<<j<<std::endl<<std::flush;
      }
      std::cout<<"Frame "<<owner->frameID()<<" just activated "<<counter<<" points.\n";
      owner->dump_active_points_to_pcd(std::to_string(owner->frameID())+".pcd");
    }
    Utils::Time t2 = std::chrono::steady_clock::now();

    if (settings.debugPrintLog && settings.debugLogActivePoints)
    {
      const std::string msg = "Act. New: " + std::to_string(numPointsCreated) + "\t";

      auto& log = Log::getInstance();
      log.addCurrentLog(lastKeyframe->frameID(), msg);
    }

    //if (settings.debugShowDistanceTransformAfter && this->outputWrapper_)
    // {
    //  cv::Mat distTransform = this->distanceMap_->drawDistanceTransform(true);
    //  this->outputWrapper_->publishDistanceTransformAfter(distTransform);
    //}

    // TL: log points
    // file << "======================================\n";
    // file.close();
  }


  void LMCW::removeOutliers() const
  {
    const auto& settings = Settings::getInstance();

    const int lastKeyframeID = this->activeKeyframes_.back()->keyframeID();

    // remove outlier points
    for (const std::shared_ptr<Frame>& keyframe : this->activeKeyframes_)
    {
      auto& activePoints = keyframe->activePoints();

      for (int i = 0; i < activePoints.size(); ++i)
      {
        auto& point = activePoints[i];

        bool remove = false;

        // check if the point is new
        const int32_t age = point->age(lastKeyframeID);
        if (age < settings.minNumKFToConsiderNew && !keyframe->flaggedToDrop())
        {
          // for new points check that have been observed in all new keyframes
          if (point->numObservations() < (age + 1))
          {
            remove = true;
          }
        }
        else
        {
          // for old points check that have been observed at least in "minNumGoodObservations" keyframes
          if (point->numObservations() < settings.minNumGoodObservations)
          {
            remove = true;
          }
        }

        if (remove)
        {
          activePoints[i] = std::move(activePoints.back());
          activePoints.pop_back();
          i--;
        }
      }
    }
  }

  void LMCW::updateConnectivity() const
  {
    const int numActiveKeyframes = (int)this->activeKeyframes_.size();

    Eigen::MatrixXi adj;
    adj.resize(numActiveKeyframes, numActiveKeyframes);
    adj.setZero();

    // update connectivity of active keyframes
    // TODO: use inner product to check
    for (int i = 0; i < this->activeKeyframes_.size(); ++i)
    {
      const std::shared_ptr<Frame>& keyframe = this->activeKeyframes_[i];

      for (int j = 0; j < this->activeKeyframes_.size(); ++j)
      {
        if (i == j) continue;

        int idx = this->activeKeyframes_[j]->keyframeID();

        int numVisible = 0;

        // look for all active point residuals
        const auto& activePoints = keyframe->activePoints();
        for (const auto& point : activePoints)
        {
          if (point->visibility(idx) == Visibility::VISIBLE)
          {
            numVisible++;
          }
        }

        adj(i, j) = numVisible;
      }
    }

    for (int i = 0; i < numActiveKeyframes; ++i)
    {
      for (int j = i + 1; j < numActiveKeyframes; ++j)
      {
        int totalVisible = adj(i, j) + adj(j, i);
        this->covisibilityGraph_->connect(this->activeKeyframes_[i], this->activeKeyframes_[j], totalVisible);
      }
    }

    if (this->outputWrapper_)
    {
      this->outputWrapper_->publishCovisibility(this->covisibilityGraph_->adjacencyMatrix());
    }
  }

  void LMCW::deleteTemporalVoxelPoints()
  {
    // We will not modify the voxel ptr in ActivePoint so that we can compare old and new voxel during covisibilityGraph update
    int numActiveKeyframes = (int)this->activeKeyframes_.size();
    for (int i = this->temporalWindowIndex; i < numActiveKeyframes; i++)
    {
      const std::shared_ptr<Frame>& owner = this->activeKeyframes_[i];
      std::vector<std::unique_ptr<ActivePoint>>& activePoints = owner->activePoints();
      for (std::unique_ptr<ActivePoint>& actPt : activePoints)
      {
        this->voxelMap_->delete_point(actPt.get());
      }
    }
  }

  void LMCW::insertTemporalVoxelPoints()
  {
    // We will not modify the voxel ptr in ActivePoint so that we can compare old and new voxel during covisibilityGraph update
    int numActiveKeyframes = (int)this->activeKeyframes_.size();
    for (int i = this->temporalWindowIndex; i < numActiveKeyframes; i++)
    {
      const std::shared_ptr<Frame>& owner = this->activeKeyframes_[i];
      std::vector<std::unique_ptr<ActivePoint>>& activePoints = owner->activePoints();
      for (std::unique_ptr<ActivePoint>& actPt : activePoints)
      {
        this->voxelMap_->insert_point(actPt.get());
      }
    }
  }

  void LMCW::updateVoxelMapCovisGraph()
  {
    int numActiveKeyframes = (int)this->activeKeyframes_.size();
    const int firstTemporalID = this->activeKeyframes_[this->temporalWindowIndex]->keyframeID();
    for (int i = this->temporalWindowIndex; i < numActiveKeyframes; i++)
    {
      const std::shared_ptr<Frame>& owner = this->activeKeyframes_[i];
      std::vector<std::unique_ptr<ActivePoint>>& activePoints = owner->activePoints();
      CovisibilityNode* curFrameNode = owner->graphNode;
      for (std::unique_ptr<ActivePoint>& actPt : activePoints)
      {
        // if voxel didn't change, no action required
        const Voxel* newVoxel = voxelMap_->query_point(actPt.get());
        if (newVoxel == actPt->voxel()) continue;

        // for each Active Point in the old voxel, decrease the edge weight
        std::vector<ActivePoint*> oldPoints = actPt->voxel()->voxPoints;
        for (ActivePoint* oldPt : oldPoints)
        {
          // if (oldPt->currentID() >= firstTemporalID) break; // stop if we've reached temporal frames
          CovisibilityNode* covisFrameNode = oldPt->reference()->graphNode;
          const std::shared_ptr<Frame>& covisFrame = this->allKeyframes_[oldPt->currentID()];
          int newWeight = curFrameNode->edges[covisFrameNode] - 1;
          if (newWeight == 0)
            covisibilityGraph_->disconnect(covisFrame, owner);
          else if (newWeight > 0)
            covisibilityGraph_->connect(covisFrame, owner, newWeight);
        }

        // delete this point from old voxel
        this->voxelMap_->delete_point_BA(actPt.get(), actPt->voxel());
        // insert this point to new voxel
        this->voxelMap_->insert_point(actPt.get());

        // for each Active point in the new voxel, increase the edge weight
        std::vector<ActivePoint*> newPoints = this->voxelMap_->query_point(actPt.get())->voxPoints;
        for (ActivePoint* newPt : newPoints)
        {
          // if (newPt->currentID() >= firstTemporalID) break; // stop if we've reached temporal frames
          CovisibilityNode* covisFrameNode = newPt->reference()->graphNode;
          const std::shared_ptr<Frame>& covisFrame = this->allKeyframes_[newPt->currentID()];
          int newWeight = curFrameNode->edges[covisFrameNode] + 1;
          covisibilityGraph_->connect(covisFrame, owner, newWeight);
        }

        // update the activepoint with its adjusted voxel
        actPt->setVoxel(this->voxelMap_->query_point(actPt.get()));
      }
    }
  }

}

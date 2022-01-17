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

#include "FullSystem.h"
#include "PointDetector.h"
#include "DistanceTransform.h"
#include "Log.h"
#include "LMCW.h"
#include "Initializer/MonoInitializer.h"
#include "Utils/Settings.h"
#include "Utils/UtilFunctions.h"
#include "Utils/Projection.h"
#include "Utils/Interpolation.h"
#include "Utils/GlobalCalibration.h"
#include "DataStructures/Frame.h"
#include "DataStructures/CandidatePoint.h"
#include "DataStructures/ActivePoint.h"
#include "Memory/Buffer.h"
#include "Memory/BufferPool.h"
#include "Thread/WorkerThreadPool.h"
#include "Tracking/FrameTracker.h"
#include "Tracking/FrameTrackerReference.h"
#include "Optimization/PhotometricBA.h"
#include "Optimization/PhotometricResidual.h"
#include "Visualizer/IVisualizer.h"
#include "cvo/Association.hpp"
#include "utils/StaticStereo.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "pcl/io/pcd_io.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Sparse>
#include <fstream>
#include <time.h>
#include "cvo/CvoGPU.hpp"
#include "utils/CvoPoint.hpp"
#include "utils/CvoPointCloud.hpp"
#include "utils/ImageStereo.hpp"
#include "utils/ImageRGBD.hpp"
#include "utils/RawImage.hpp"

#include "DataStructures/CovisibilityGraph.h"

namespace dsm
{
  FullSystem::FullSystem(int w, int h, const Eigen::Matrix3f &calib,
                         const std::string &settingsFile,
                         IVisualizer *outputWrapper) :
    initialized(false),
    trackingIsGood(true),
    createNewKeyframe(false),
    createNewKeyframeID(-1),

    numTrackedFramesFromLastKF(0),
    shouldStop(false),
    newFrameMappedDone(true),
    outputWrapper(outputWrapper),
    lastTrackingCos(1)
  {
    std::cout<<"FullSystem default constructor...\n";
    // First: initialize pattern types
    Pattern::initialize();

    // Second: read settings from file
    auto& settings = Settings::getInstance();
    if (!settingsFile.empty())
    {
      std::cout << "Reading settings...";
      if (settings.fromFile(settingsFile))
      {
        std::cout << "succesfully!" << std::endl;
      }
      else
      {
        std::cout << "failed!" << std::endl;
      }
    }

    // Third: set global calibration
    auto& globalCalib = GlobalCalibration::getInstance();
    globalCalib.setCalibration(w, h, calib, settings.pyramidLevels);

    const int width = (int)globalCalib.width(0);
    const int height = (int)globalCalib.height(0);

    // thread pool
    this->threadPool = std::make_shared<WorkerThreadPool>(settings.mappingThreads);

    // initializer
    this->initializer = std::make_unique<MonoInitializer>(this->threadPool, this->outputWrapper);

    // pixel detector
    this->pointDetector = std::make_unique<PointDetector>(width, height, (int)settings.pointDetectionLevels,
                                                          (int)settings.numBlocksPerDimension, settings.minGradAdd,
                                                          this->threadPool);

    this->pixelMask = (int32_t*) new int32_t [width * height];  //Eigen::internal::aligned_malloc(width * height * sizeof(int32_t));

    // Fourth: initialize member variables
    this->tracker = std::make_unique<FrameTracker>(width, height);
    //this->trackingReference = std::make_shared<FrameTrackerReference>(width, height);
    //this->newTrackingReference = std::make_shared<FrameTrackerReference>(width, height);
    this->trackingReferenceUpdated = false;

    // optimization window
    this->lmcw = std::make_unique<LMCW>(width, height, this->outputWrapper);

    // optimizer
    PhotometricBAConfig config;
    this->ceresOptimizer = std::make_unique<CeresPhotometricBA>(config, this->outputWrapper);

    // initial tracking priors
    this->lastTrackedMotion = Sophus::SE3f();
    this->lastTrackedFrame = nullptr;
    this->lastTrackingCos = 0;
    //this->lastTrackedResidual = std::numeric_limits<float>::max();

    // depth map save counter
    this->saveID = 0;

    // depth map visualization image
    this->depthMapImage = cv::Mat(h >> settings.showDepthMapLvl, w >> settings.showDepthMapLvl, CV_8UC3);
    this->minIDepthTracker = -1.f;
    this->maxIDepthTracker = -1.f;
    this->minIDepthOpt = -1.f;
    this->maxIDepthOpt = -1.f;

    // start mapping thread
    if (!settings.singleThreaded)
    {
      this->mappingThread = std::make_unique<std::thread>(&FullSystem::mappingThreadLoop, this);
    }
  }

  FullSystem::~FullSystem()
  {
    const auto& settings = Settings::getInstance();

    // make sure none is waiting for something
    std::cout << "Waiting for DSM to finish.." << std::endl;

    // wait all frames to be processed
    if (!settings.singleThreaded)
    {
      this->waitUntilMappingFinished();

      // send a signal to finish!
      {
        std::lock_guard<std::mutex> lock(this->unmappedTrackedFramesMutex);
        this->shouldStop = true;
      }
      this->unmappedTrackedFramesSignal.notify_all();

      // wait the thread to exit
      if (this->mappingThread->joinable())
      {
        this->mappingThread->join();
      }
    }

    std::cout << " .. DSM has finished." << std::endl;

    // delete main classes in order
    if (this->initializer) this->initializer = nullptr;
    if (this->pointDetector) this->pointDetector = nullptr;
    if (this->tracker) this->tracker = nullptr;
    if (this->trackingReference) this->trackingReference = nullptr;
    if (this->newTrackingReference) this->newTrackingReference = nullptr;
    if (this->ceresOptimizer) this->ceresOptimizer = nullptr;
    if (this->lastTrackedFrame) lastTrackedFrame = nullptr;
    if (this->threadPool) this->threadPool = nullptr;
    if (this->lmcw) this->lmcw = nullptr;

    // clear keyframes and return all buffers to their pools
    // make sure std::shared_ptr<Frame> live only in this class
    this->unmappedTrackedFrames.clear();

    // delete mask for pixel detector
    //Eigen::internal::aligned_free(this->pixelMask);
    delete this->pixelMask;

    // delete buffers
    // guarantee that all buffers have been returned to the pool
    BufferPool<bool>::getInstance().clearPool();				// bool pool
    BufferPool<char>::getInstance().clearPool();				// char pool
    BufferPool<unsigned char>::getInstance().clearPool();		// unsigned char pool
    BufferPool<float>::getInstance().clearPool();				// float pool
  }

  FullSystem::FullSystem(int w, int h, const cvo::Calibration &calib, 
                         const std::string &cvoParamsFile,
                         const std::string &settingsFile,
                         IVisualizer *outputWrapper) : FullSystem(w, h, calib.intrinsic(), settingsFile, outputWrapper) {

    std::cout<<"FullSystem Cvo constructor..\n";
    this->cvo_align.reset(new cvo::CvoGPU(cvoParamsFile));
    
    auto& globalCalib = GlobalCalibration::getInstance();
    globalCalib.setBaseline(calib.baseline());
    globalCalib.setDepthScale(calib.scaling_factor());
    lastTrackingCos = 1;

    this->lmcw = std::make_unique<LMCW>(w, h, cvo_align.get(), outputWrapper, 0.05);
  }
  

  bool FullSystem::isInitialized() const
  {
    return this->initialized;
  }

  bool FullSystem::isLost() const
  {
    return !this->trackingIsGood;
  }

  void FullSystem::getTrajectory(std::vector<Eigen::Matrix4f> &poses, std::vector<double> &timestamps,
                                 std::vector<int> & ids) const
  {
    const auto& allKeyframes = this->lmcw->allKeyframes();

    poses.clear();
    timestamps.clear();
    poses.reserve(allKeyframes.size());
    timestamps.reserve(allKeyframes.size());

    for (const auto& kf : allKeyframes)
    {
      poses.push_back(kf->camToWorld().matrix());
      timestamps.push_back(kf->timestamp());
      ids.push_back(kf->frameID());
    }
  }

    void FullSystem::getFullTrajectory(std::vector<Eigen::Matrix4f> &poses, std::vector<double> &timestamps,
                           std::vector<int> &ids) const
    {
      poses.clear();
      timestamps.clear();
      poses.reserve(allFrames.size());
      timestamps.reserve(allFrames.size());

      for (const auto& frm : allFrames)
      {
        if (frm->type() == Frame::KEYFRAME)
        {
          poses.push_back(frm->camToWorld().matrix());
        } else {
          // non-KF, find parent KF
          poses.push_back(frm->parent()->camToWorld().matrix() * frm->thisToParentPose().matrix());
        }
        timestamps.push_back(frm->timestamp());
        ids.push_back(frm->frameID());
      }
    }
  

  void FullSystem::getStructure(std::vector<Eigen::Vector3f>& structure) const
  {
    structure.clear();

    const auto& calib = GlobalCalibration::getInstance();
    const auto& K = calib.matrix3f(0);

    const auto& allKeyframes = this->lmcw->allKeyframes();

    for (const auto& kf : allKeyframes)
    {
      const auto& pose = kf->camToWorld();

      for (const auto& pt : kf->activePoints())
      {
        const float u = pt->u(0);
        const float v = pt->v(0);
        const float depth = 1.f / pt->iDepth();

        if (depth <= 0.f) continue;

        for (int idx = 0; idx < Pattern::size(); ++idx)
        {
          float uj = u + (float)Pattern::at(idx, 0);
          float vj = v + (float)Pattern::at(idx, 1);

          // point in camera
          Eigen::Vector3f pt = depth*Eigen::Vector3f((uj - K(0, 2)) / K(0, 0),
                                                     (vj - K(1, 2)) / K(1, 1),
                                                     1.f);

          // point in world
          pt = pose * pt;

          // save
          structure.push_back(pt);
        }
      }
    }
  }

  float FullSystem::getMeanTime(const std::vector<float> & timeVec)const  {
    
    float time = 0.f;
    if (timeVec.size() > 0)
    {
      for (int i = 0; i < timeVec.size(); ++i)
      {
        time += timeVec[i];
      }
      time /= timeVec.size();
    }
    return time;
    
  }

  float FullSystem::getLocalBAMeanTime() const {
    return getMeanTime(localBATime);
  }
  float FullSystem::getTotalBackendMeanTime() const {
    return getMeanTime(totalBackendTime);
  }
  float FullSystem::getWindowConstructionMeanTime() const {
    return getMeanTime(windowConstructionTime);
  }
  float FullSystem::getCvoBAMeanTime() const {
    return getMeanTime(cvoBATime);
  }
  
  float FullSystem::getCamTrackingMeanTime() const
  {
    float time = 0.f;
    if (this->camTrackingTime.size() > 0)
    {
      for (int i = 0; i < this->camTrackingTime.size(); ++i)
      {
        time += this->camTrackingTime[i];
      }
      time /= this->camTrackingTime.size();
    }
    return time;
  }

  float FullSystem::getPointTrackingMeanTime() const
  {
    float time = 0.f;
    if (this->pointTrackingTime.size() > 0)
    {
      for (int i = 0; i < this->pointTrackingTime.size(); ++i)
      {
        time += this->pointTrackingTime[i];
      }
      time /= this->pointTrackingTime.size();
    }
    return time;
  }

  int FullSystem::getNumPoints() const
  {
    const auto& allKeyframes = this->lmcw->allKeyframes();

    int num = 0;
    for (const auto& kf : allKeyframes)
    {
      num += (int)kf->activePoints().size();
    }
    return num;
  }

  int FullSystem::getNumKeyframes() const
  {
    return (int)this->lmcw->allKeyframes().size();
  }

  bool FullSystem::initialize_lmcw(const std::shared_ptr<Frame>& frame)
  {
    if (this->initialized) return true;

    Utils::Time t1 = std::chrono::steady_clock::now();

    const auto& allKeyframes = this->lmcw->allKeyframes();

    if (allKeyframes.empty())
    {
      // insert frame as first keyframe. Just set the reference frame, not actually tracking
      // is involved
      frame->setTrackingResult(nullptr, Sophus::SE3f(), AffineLight());
      this->lmcw->insertNewKeyframe(frame);

      // create candidates
      this->createCandidates(frame);
      this->lastTrackedFrame = frame;
      
      // set as initializer reference
      //this->initializer->setReference(frame);
      return false;
    }
    else
    {
      // try to initialize
      Sophus::SE3f firstToSecond;
      Eigen::Matrix4f trackingPoseResult;
      double trackingTime;
      cvo::CvoParams & init_param = cvo_align->get_params();
      std::swap(init_param.ell_init, init_param.ell_init_first_frame);
      std::swap(init_param.ell_decay_rate, init_param.ell_decay_rate_first_frame);
      std::swap(init_param.ell_decay_start, init_param.ell_decay_start_first_frame);
      cvo_align->write_params(&init_param);
      //pcl::io::savePCDFileASCII ("source.pcd",      *this->initializer->getReference()->get_cvo_pcd());
      //pcl::io::savePCDFileASCII ("target.pcd",      *frame->get_cvo_pcd());
      std::shared_ptr<Frame>  last_KF = this->lmcw->activeWindow()[this->lmcw->getActiveWindowSize()-1];
      int cvoTrackingResult = cvo_align->align(*(last_KF->getTrackingPoints()),
                                               *(frame->getTrackingPoints()),
                                               firstToSecond.matrix(),
                                               // outputs
                                               trackingPoseResult,
                                               nullptr,
                                               &trackingTime);
      Sophus::SE3f resultSophus(trackingPoseResult.block<3,3>(0,0), trackingPoseResult.block<3,1>(0,3));
      firstToSecond = resultSophus.inverse(); // convert back to sophus
      std::swap(init_param.ell_init, init_param.ell_init_first_frame);
      std::swap(init_param.ell_decay_rate, init_param.ell_decay_rate_first_frame);
      std::swap(init_param.ell_decay_start, init_param.ell_decay_start_first_frame);
      cvo_align->write_params(&init_param);

      

      //if (isUsingCvo) {// || this->initializer->initialize(frame, firstToSecond))
      
      // rescale to norm(t) = 0.1m
      //firstToSecond.translation() /= firstToSecond.translation().norm();
      //firstToSecond.translation() *= 0.1f;
      
      // set initialization pose as tracking result
      const auto& firstKF = allKeyframes[0];
      //std::cout<<"\nInit: set track result firstToSecond "<<firstToSecond.matrix()<<std::endl;
      frame->setTrackingResult(firstKF.get(), firstToSecond.inverse(), AffineLight());
      
      // initialize some values: motion, flags
      this->lastTrackedFrame = frame;
      this->lastTrackedMotion = firstToSecond.inverse(); //Sophus::SE3f();
      
      //Utils::Time t2 = std::chrono::steady_clock::now();
      //std::cout << "Done initialization in " << Utils::elapsedTime(t1, t2) << "ms" << std::endl;
      
      // insert frame as keyframe and optimize
      this->createKeyframeAndOptimize(frame);

      
      Utils::Time t2 = std::chrono::steady_clock::now();
      std::cout << "Trying to initialize... " << Utils::elapsedTime(t1, t2) << "ms" << std::endl;

      
      return true;
        
    }
    /*
    // reset if it cannot initialize and start again
    if (this->initializer->isResetRequired())
    {
      this->initialized = false;

      this->lastTrackedFrame = nullptr;

      this->lmcw->clear();

      this->initializer->reset();
    }

    */

  }

  /*
  void FullSystem::trackFrame(int id, double timestamp, unsigned char* image)
  {

    auto& settings = Settings::getInstance();

    // track frame
    Utils::Time t1 = std::chrono::steady_clock::now();

    // Create new frame
    std::shared_ptr<Frame> trackingNewFrame = std::make_shared<Frame>(id, timestamp, image);

    if (settings.debugPrintLog)
    {
      auto& log = Log::getInstance();
      log.addNewLog(trackingNewFrame->frameID());
    }

    if (!this->trackingIsGood)
    {
      std::cout << "LOST..." << std::endl;
      return;
    }

    // initialization
    if (!this->initialized)
    {
      this->initialized = this->initialize(trackingNewFrame, false);
      return;
    }

    // track
    this->trackNewFrame(trackingNewFrame);

    if (!this->trackingIsGood)
    {
      std::cout << "Tracking LOST!!" << std::endl;
      return;
    }

    if (settings.debugPrintLog && settings.debugLogTracking)
    {
      const float* pose = trackingNewFrame->thisToParentPose().data();
      const AffineLight& light = trackingNewFrame->thisToParentLight();
      const float energy = this->tracker->totalResidual();

      std::string msg = "trackingPose: " + std::to_string(pose[0]) + "\t" + std::to_string(pose[1]) + "\t" + std::to_string(pose[2]) + "\t" + std::to_string(pose[3]) + "\t"
        + std::to_string(pose[4]) + "\t" + std::to_string(pose[5]) + "\t" + std::to_string(pose[6]) + "\t";
      msg += "affineLight: " + std::to_string(light.alpha()) + "\t" + std::to_string(light.beta()) + "\t";
      msg += "cost: " + std::to_string(energy) + "\t";

      auto& log = Log::getInstance();
      log.addCurrentLog(trackingNewFrame->frameID(), msg);
    }

    // Keyframe selection
    // If we have already asked to create one, dont do it again
    bool localCreateNewKeyframe = false;
    if (!this->createNewKeyframe)
    {
      if(this->numMappedFramesFromLastKF >= settings.minNumMappedFramesToCreateKF)
      {
        localCreateNewKeyframe = this->isNewKeyframeRequired(trackingNewFrame);
      }
						
      this->createNewKeyframe = localCreateNewKeyframe;
    }

    // insert current frame to unmapped queue
    Utils::Time t2;
    if (!settings.singleThreaded)
    {
      {
        std::lock_guard<std::mutex> unMappedLock(this->unmappedTrackedFramesMutex);
        this->unmappedTrackedFrames.push_back(trackingNewFrame);

        if (localCreateNewKeyframe)
        {
          this->createNewKeyframeID = trackingNewFrame->frameID();
        }

        // control flag for blocking
        {
          std::lock_guard<std::mutex> newFrameMappedLock(this->newFrameMappedMutex);
          this->newFrameMappedDone = false;
        }
      }
      this->unmappedTrackedFramesSignal.notify_one();

      t2 = std::chrono::steady_clock::now();
    }
    else
    {
      if (localCreateNewKeyframe)
      {
        this->createNewKeyframeID = trackingNewFrame->frameID();
      }
      
      this->doMapping(trackingNewFrame);
      t2 = std::chrono::steady_clock::now();      
    }

    const float time = Utils::elapsedTime(t1, t2);
    this->camTrackingTime.push_back(time);

    if (this->outputWrapper)
    {
      // current camera pose
      const Eigen::Matrix4f camPose = (this->lastTrackedFrame->parent()->camToWorld() * 
                                       this->lastTrackedFrame->thisToParentPose()).matrix();
      this->outputWrapper->publishCurrentFrame(camPose);

      //timings			
      this->outputWrapper->publishCamTrackingTime(time);
    }

    // implement blocking
    // required for debugging
    if (!settings.singleThreaded && settings.blockUntilMapped && this->trackingIsGood)
    {
      this->waitUntilMappingFinished();
    }
  }
  */
  /*
  void FullSystem::trackFrame(int id, double timestamp, unsigned char* image, std::shared_ptr<cvo::RawImage> left_img, const cv::Mat & right_img,  pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd)
  {

    auto& settings = Settings::getInstance();

    // track frame
    Utils::Time t1 = std::chrono::steady_clock::now();

    // Create new frame
    std::vector<float> stereoDisparity;
    cvo::StaticStereo::disparity(left_img->image(), right_img, stereoDisparity );
    std::shared_ptr<Frame> trackingNewFrame = std::make_shared<Frame>(id, timestamp, image, left_img, new_frame_pcd, stereoDisparity);

    if (settings.debugPrintLog)
    {
      auto& log = Log::getInstance();
      log.addNewLog(trackingNewFrame->frameID());
    }

    if (!this->trackingIsGood)
    {
      std::cout << "LOST..." << std::endl;
      return;
    }

    // initialization: TODO
    if (!this->initialized)
    {
      this->initialized = this->initialize_lmcw(trackingNewFrame);
      return;
    }

    // track
    this->trackModelToFrameCvo(trackingNewFrame, new_frame_pcd );
    if (!this->trackingIsGood)
    {
      std::cout << "Tracking LOST!!" << std::endl;
      return;
    }

    if (settings.debugPrintLog && settings.debugLogTracking)
    {
      const float* pose = trackingNewFrame->thisToParentPose().data();
      const AffineLight& light = trackingNewFrame->thisToParentLight();
      const float energy = this->tracker->totalResidual();

      std::string msg = "trackingPose: " + std::to_string(pose[0]) + "\t" + std::to_string(pose[1]) + "\t" + std::to_string(pose[2]) + "\t" + std::to_string(pose[3]) + "\t"
        + std::to_string(pose[4]) + "\t" + std::to_string(pose[5]) + "\t" + std::to_string(pose[6]) + "\t";
      msg += "affineLight: " + std::to_string(light.alpha()) + "\t" + std::to_string(light.beta()) + "\t";
      msg += "cost: " + std::to_string(energy) + "\t";

      auto& log = Log::getInstance();
      log.addCurrentLog(trackingNewFrame->frameID(), msg);
    }

    // Keyframe selection
    // If we have already asked to create one, dont do it again
    bool localCreateNewKeyframe = false;
    if (!this->createNewKeyframe)
    {
      if(this->numTrackedFramesFromLastKF >= settings.minNumTrackedFramesToCreateKF)
      {
        localCreateNewKeyframe = this->isNewKeyframeRequired(trackingNewFrame);
      }
						
      this->createNewKeyframe = localCreateNewKeyframe;
    }

    // insert current frame to unmapped queue
    Utils::Time t2;
    if (!settings.singleThreaded)
    {
      {
        std::lock_guard<std::mutex> unMappedLock(this->unmappedTrackedFramesMutex);
        this->unmappedTrackedFrames.push_back(trackingNewFrame);

        if (localCreateNewKeyframe)
        {
          this->createNewKeyframeID = trackingNewFrame->frameID();
        }

        // control flag for blocking
        {
          std::lock_guard<std::mutex> newFrameMappedLock(this->newFrameMappedMutex);
          this->newFrameMappedDone = false;
        }
      }
      this->unmappedTrackedFramesSignal.notify_one();

      t2 = std::chrono::steady_clock::now();
    }
    else
    {
      if (localCreateNewKeyframe)
      {
        this->createNewKeyframeID = trackingNewFrame->frameID();
      }

      t2 = std::chrono::steady_clock::now();
      
      this->doMapping(trackingNewFrame);
    }

    const float time = Utils::elapsedTime(t1, t2);
    this->camTrackingTime.push_back(time);

    if (this->outputWrapper)
    {
      // current camera pose
      const Eigen::Matrix4f camPose = (this->lastTrackedFrame->parent()->camToWorld() * 
                                       this->lastTrackedFrame->thisToParentPose()).matrix();
      this->outputWrapper->publishCurrentFrame(camPose);

      //timings			
      this->outputWrapper->publishCamTrackingTime(time);
    }

    // implement blocking
    // required for debugging
    if (!settings.singleThreaded && settings.blockUntilMapped && this->trackingIsGood)
    {
      this->waitUntilMappingFinished();
    }
  }

  void FullSystem::trackFrame(int id, double timestamp,
                             unsigned char* image, std::shared_ptr<cvo::RawImage> left_img, const std::vector<uint16_t> & depth_img, float depth_scale,
                             pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd)
  {
    auto& settings = Settings::getInstance();

    // track frame
    Utils::Time t1 = std::chrono::steady_clock::now();

    // Create new frame
    std::shared_ptr<Frame> trackingNewFrame = std::make_shared<Frame>(id, timestamp, image, left_img, new_frame_pcd, depth_img, depth_scale);

    if (settings.debugPrintLog)
    {
      auto& log = Log::getInstance();
      log.addNewLog(trackingNewFrame->frameID());
    }

    if (!this->trackingIsGood)
    {
      std::cout << "LOST..." << std::endl;
      return;
    }

    // initialization: TODO
    if (!this->initialized)
    {
      this->initialized = this->initialize_lmcw(trackingNewFrame);
      return;
    }

    // track
    //this->trackNewFrame(trackingNewFrame);
    this->trackModelToFrameCvo(trackingNewFrame, new_frame_pcd );

    if (!this->trackingIsGood)
    {
      std::cout << "Tracking LOST!!" << std::endl;
      return;
    }

    Utils::Time t2;
    const float time = Utils::elapsedTime(t1, t2);
    this->camTrackingTime.push_back(time);
    
    
    if (settings.debugPrintLog && settings.debugLogTracking)
    {
      const float* pose = trackingNewFrame->thisToParentPose().data();
      const AffineLight& light = trackingNewFrame->thisToParentLight();
      const float energy = this->tracker->totalResidual();

      std::string msg = "trackingPose: " + std::to_string(pose[0]) + "\t" + std::to_string(pose[1]) + "\t" + std::to_string(pose[2]) + "\t" + std::to_string(pose[3]) + "\t"
        + std::to_string(pose[4]) + "\t" + std::to_string(pose[5]) + "\t" + std::to_string(pose[6]) + "\t";
      msg += "affineLight: " + std::to_string(light.alpha()) + "\t" + std::to_string(light.beta()) + "\t";
      msg += "cost: " + std::to_string(energy) + "\t";

      auto& log = Log::getInstance();
      log.addCurrentLog(trackingNewFrame->frameID(), msg);

    }

    

    // Keyframe selection
    // If we have already asked to create one, dont do it again
    bool localCreateNewKeyframe = false;
    if (!this->createNewKeyframe)
    {
      if(this->numTrackedFramesFromLastKF >= settings.minNumTrackedFramesToCreateKF)
      {
        localCreateNewKeyframe = this->isNewKeyframeRequired(trackingNewFrame);
      }
						
      this->createNewKeyframe = localCreateNewKeyframe;
    }

    std::cout<<"Create new keyframe is "<<localCreateNewKeyframe<<std::endl;

    // insert current frame to unmapped queue
    Utils::Time t3;
    if (!settings.singleThreaded)
    {
      {
        std::lock_guard<std::mutex> unMappedLock(this->unmappedTrackedFramesMutex);
        this->unmappedTrackedFrames.push_back(trackingNewFrame);

        if (localCreateNewKeyframe)
        {
          this->createNewKeyframeID = trackingNewFrame->frameID();
        }

        // control flag for blocking
        {
          std::lock_guard<std::mutex> newFrameMappedLock(this->newFrameMappedMutex);
          this->newFrameMappedDone = false;
        }
      }
      this->unmappedTrackedFramesSignal.notify_one();

      t3 = std::chrono::steady_clock::now();
    }
    else
    {
      if (localCreateNewKeyframe)
      {
        this->createNewKeyframeID = trackingNewFrame->frameID();
      }

      t2 = std::chrono::steady_clock::now();
      
      this->doMapping(trackingNewFrame);
    }

    //const float time = Utils::elapsedTime(t1, t2);
    //this->camTrackingTime.push_back(time);

    if (this->outputWrapper)
    {
      // current camera pose
      const Eigen::Matrix4f camPose = (this->lastTrackedFrame->parent()->camToWorld() * 
                                       this->lastTrackedFrame->thisToParentPose()).matrix();
      this->outputWrapper->publishCurrentFrame(camPose);

      //timings			
      this->outputWrapper->publishCamTrackingTime(time);
    }

    // implement blocking
    // required for debugging
    if (!settings.singleThreaded && settings.blockUntilMapped && this->trackingIsGood)
    {
      this->waitUntilMappingFinished();
    }
  }


  void FullSystem::trackFrame(int id, double timestamp, Frame::Ptr trackingNewFrame) {
                              //unsigned char* image, std::shared_ptr<cvo::RawImage> left_img, const std::vector<uint16_t> & depth_img, float depth_scale,
                              //pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd)
  {
    auto& settings = Settings::getInstance();

    // track frame
    Utils::Time t1 = std::chrono::steady_clock::now();

    // Create new frame
    //std::shared_ptr<Frame> trackingNewFrame = std::make_shared<Frame>(id, timestamp, image, left_img, new_frame_pcd, depth_img, depth_scale);

    if (settings.debugPrintLog)
    {
      auto& log = Log::getInstance();
      log.addNewLog(trackingNewFrame->frameID());
    }

    if (!this->trackingIsGood)
    {
      std::cout << "LOST..." << std::endl;
      return;
    }

    // initialization: TODO
    if (!this->initialized)
    {
      this->initialized = this->initialize_lmcw(trackingNewFrame);
      return;
    }

    // track
    //this->trackNewFrame(trackingNewFrame);
    this->trackModelToFrameCvo(trackingNewFrame);

    if (!this->trackingIsGood)
    {
      std::cout << "Tracking LOST!!" << std::endl;
      return;
    }

    Utils::Time t2;
    const float time = Utils::elapsedTime(t1, t2);
    this->camTrackingTime.push_back(time);
    
    
    if (settings.debugPrintLog && settings.debugLogTracking)
    {
      const float* pose = trackingNewFrame->thisToParentPose().data();
      const AffineLight& light = trackingNewFrame->thisToParentLight();
      const float energy = this->tracker->totalResidual();

      std::string msg = "trackingPose: " + std::to_string(pose[0]) + "\t" + std::to_string(pose[1]) + "\t" + std::to_string(pose[2]) + "\t" + std::to_string(pose[3]) + "\t"
        + std::to_string(pose[4]) + "\t" + std::to_string(pose[5]) + "\t" + std::to_string(pose[6]) + "\t";
      msg += "affineLight: " + std::to_string(light.alpha()) + "\t" + std::to_string(light.beta()) + "\t";
      msg += "cost: " + std::to_string(energy) + "\t";

      auto& log = Log::getInstance();
      log.addCurrentLog(trackingNewFrame->frameID(), msg);

    }

    

    // Keyframe selection
    // If we have already asked to create one, dont do it again
    bool localCreateNewKeyframe = false;
    if (!this->createNewKeyframe)
    {
      if(this->numTrackedFramesFromLastKF >= settings.minNumTrackedFramesToCreateKF)
      {
        localCreateNewKeyframe = this->isNewKeyframeRequired(trackingNewFrame);
      }
						
      this->createNewKeyframe = localCreateNewKeyframe;
    }

    std::cout<<"Create new keyframe is "<<localCreateNewKeyframe<<std::endl;

    // insert current frame to unmapped queue
    Utils::Time t3;
    if (!settings.singleThreaded)
    {
      {
        std::lock_guard<std::mutex> unMappedLock(this->unmappedTrackedFramesMutex);
        this->unmappedTrackedFrames.push_back(trackingNewFrame);

        if (localCreateNewKeyframe)
        {
          this->createNewKeyframeID = trackingNewFrame->frameID();
        }

        // control flag for blocking
        {
          std::lock_guard<std::mutex> newFrameMappedLock(this->newFrameMappedMutex);
          this->newFrameMappedDone = false;
        }
      }
      this->unmappedTrackedFramesSignal.notify_one();

      t3 = std::chrono::steady_clock::now();
    }
    else
    {
      if (localCreateNewKeyframe)
      {
        this->createNewKeyframeID = trackingNewFrame->frameID();
      }

      t2 = std::chrono::steady_clock::now();
      
      this->doMapping(trackingNewFrame);
    }

    //const float time = Utils::elapsedTime(t1, t2);
    //this->camTrackingTime.push_back(time);

    if (this->outputWrapper)
    {
      // current camera pose
      const Eigen::Matrix4f camPose = (this->lastTrackedFrame->parent()->camToWorld() * 
                                       this->lastTrackedFrame->thisToParentPose()).matrix();
      this->outputWrapper->publishCurrentFrame(camPose);

      //timings			
      this->outputWrapper->publishCamTrackingTime(time);
    }

    // implement blocking
    // required for debugging
    if (!settings.singleThreaded && settings.blockUntilMapped && this->trackingIsGood)
    {
      this->waitUntilMappingFinished();
    }
  }
  */  
  /*  
  void FullSystem::trackFrameToFrameCvo(const std::shared_ptr<Frame>& frame, const cvo::CvoPointCloud & new_frame_pcd ){

    Utils::Time t1 = std::chrono::steady_clock::now();

    // select tracking reference keyframe
    Sophus::SE3f lastToWorldPose = this->lastTrackedFrame->parent()->camToWorld() *
      this->lastTrackedFrame->thisToParentPose();

    AffineLight lastToWorldLight = AffineLight::calcGlobal(this->lastTrackedFrame->parent()->affineLight(),
                                                           this->lastTrackedFrame->thisToParentLight());

    if (this->trackingReferenceUpdated)
    {
      std::lock_guard<std::mutex> lock(this->trackingReferenceMutex);
      std::swap(this->trackingReference, this->newTrackingReference);
      this->createNewKeyframe = false;
      this->trackingReferenceUpdated = false;

      Frame* const newReference = this->trackingReference->reference();
      Frame* const oldReference = this->newTrackingReference->reference();

      if (newReference && oldReference)
      {
        // correct last frame pose with PBA result
        // oldReference = parent of newReference

        // pose
        const Sophus::SE3f poseCorrection = newReference->camToWorld().inverse() * 
          oldReference->camToWorld() * 
          newReference->thisToParentPose();

        lastToWorldPose *= poseCorrection.inverse();

        // light
        const AffineLight newRefOldLight = AffineLight::calcGlobal(newReference->parent()->affineLight(),
                                                                   newReference->thisToParentLight());
        const AffineLight lightCorrection = AffineLight::calcRelative(newRefOldLight,
                                                                      newReference->affineLight());
        lastToWorldLight = AffineLight::calcGlobal(lastToWorldLight,
                                                   lightCorrection);
      }
    }

    Frame* const reference = this->trackingReference->reference();

    // pose prior -> constant velocity model
    const Sophus::SE3f lastToRef = reference->camToWorld().inverse() * lastToWorldPose;
    Sophus::SE3f frameToRefPose = lastToRef * this->lastTrackedMotion;

    // affine light prior
    AffineLight frameToRefLight = AffineLight::calcRelative(reference->affineLight(),
                                                            lastToWorldLight);

    // Error distribution
    std::shared_ptr<IDistribution> errorDistribution;

    // Track
    // relative pose to reference keyframe
    // relative affine light to reference keyframe
    bool goodTracked = this->tracker->trackFrame(this->trackingReference, frame, frameToRefPose, frameToRefLight,
                                                 errorDistribution, this->outputWrapper);

    //tracking lost? - reset tracking internal data
    if (!goodTracked)
    {
      this->trackingIsGood = false;

      this->lastTrackedFrame = nullptr;
      this->lastTrackedMotion = Sophus::SE3f();

      this->trackingReference->reset();
      this->tracker->reset();

      this->unmappedTrackedFramesSignal.notify_one();

      return;
    }

    // save result to the latest frame
    frame->setTrackingResult(reference, frameToRefPose, frameToRefLight);

    // error distribution
    frame->setErrorDistribution(errorDistribution);

    // residuals per level
    this->lastTrackedResidual = this->tracker->totalResidual();

    if (this->trackingReference->firstFrameResidual() < 0.f)
    {
      this->trackingReference->setFirstFrameResidual(this->lastTrackedResidual);
    }

    // save info for tracking priors
    this->lastTrackedMotion = lastToRef.inverse() * frameToRefPose;
    this->lastTrackedFrame = frame;

    Utils::Time t2 = std::chrono::steady_clock::now();
    //std::cout << "Tracking: " << Utils::elapsedTime(t1, t2) << "\n";
    
  
  }
  */
  
  void FullSystem::trackModelToFrameCvo( std::shared_ptr<Frame>& frame){

    Utils::Time t1 = std::chrono::steady_clock::now();

    // select tracking reference keyframe
    Sophus::SE3f lastToWorldPose = this->lastTrackedFrame->parent()->camToWorld() *
      this->lastTrackedFrame->thisToParentPose();

    //AffineLight lastToWorldLight = AffineLight::calcGlobal(this->lastTrackedFrame->parent()->affineLight(),
    //                                                      this->lastTrackedFrame->thisToParentLight());

    // if a new kf (starting from the third one) is setup in CreateNewKFAndOptimize,
    // update the tracking reference, the initial guess pose and light
    if (this->trackingReferenceUpdated)
    {
      std::lock_guard<std::mutex> lock(this->trackingReferenceMutex);
      std::swap(this->trackingReference, this->newTrackingReference);
      std::cout<<"setting this->createNewKeyframe to false"<<std::endl;
      this->createNewKeyframe = false;
      this->trackingReferenceUpdated = false;

      Frame::Ptr newReference = this->trackingReference;
      Frame::Ptr oldReference = this->newTrackingReference;

      if (newReference && oldReference)
      {
        // correct last frame pose with PBA result
        // oldReference = parent of newReference

        // pose
        const Sophus::SE3f poseCorrection = newReference->camToWorld().inverse() * 
          oldReference->camToWorld() * 
          newReference->thisToParentPose();

        lastToWorldPose *= poseCorrection.inverse();

        // light
        //const AffineLight newRefOldLight = AffineLight::calcGlobal(newReference->parent()->affineLight(),
        //                                                            newReference->thisToParentLight());
      //const AffineLight lightCorrection = AffineLight::calcRelative(newRefOldLight,
        //                                                              newReference->affineLight());
      //  lastToWorldLight = AffineLight::calcGlobal(lastToWorldLight,
                                                   //                                           lightCorrection);
      }
    }

    Frame::Ptr reference = this->trackingReference;
    // pose prior -> constant velocity model
    std::cout<<"\nCvo tracking init values: reference id "<<reference->frameID()<<" camToWorld is \n"<<reference->camToWorld().matrix()<<"\n last frame id "<<lastTrackedFrame->frameID()<<"'s lastToWorldPose is "<<lastToWorldPose.matrix()<<std::endl;
    const Sophus::SE3f lastToRef = reference->camToWorld().inverse() * lastToWorldPose;
    Sophus::SE3f frameToRefPose = lastToRef * this->lastTrackedMotion;
    Eigen::Matrix4f frameToRefPoseEigen = frameToRefPose.matrix().inverse();

    // affine light prior
    //AffineLight frameToRefLight = AffineLight::calcRelative(reference->affineLight(),
    //                                                        lastToWorldLight);

    // Error distribution
    //std::shared_ptr<IDistribution> errorDistribution;

    // Track
    // relative pose to reference keyframe
    std::cout<<"Cvo trying to align frame "<<reference->frameID()<<" and frame "<<frame->frameID()<<std::endl;
    pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd = frame->getTrackingPoints();
    Eigen::Matrix4f trackingPoseResult;
    double trackingTime;
    // std::cout<<"Each has "<<reference->getTrackingPoints()->size()<<" and "<<new_frame_pcd->size()<<"\n";
    int cvoTrackingResult = cvo_align->align(*(reference->getTrackingPoints()),
                                             *(new_frame_pcd),
                                             frameToRefPoseEigen,
                                             // outputs
                                             trackingPoseResult,
                                             nullptr,
                                             &trackingTime);
    Sophus::SE3f resultSophus(trackingPoseResult.block<3,3>(0,0), trackingPoseResult.block<3,1>(0,3));
    frameToRefPose = resultSophus; // convert back to sophus
    
    // track relative affine light to reference keyframe
    //bool goodTrackedLight = this->tracker->trackFrame(this->trackingReference, frame, frameToRefPose, frameToRefLight,                                                                                                                 errorDistribution, this->outputWrapper, true);      

    //tracking lost? - reset tracking internal data
    bool goodTracked = (cvoTrackingResult == 0);
    //bool goodTracked = (cvoTrackingResult == 0 && goodTrackedLight);
    if (!goodTracked)
    {
      this->trackingIsGood = false;

      this->lastTrackedFrame = nullptr;
      this->lastTrackedMotion = Sophus::SE3f();

      this->trackingReference = nullptr;
      //this->tracker->reset();

      this->unmappedTrackedFramesSignal.notify_one();

      return;
    }

    // save result to the latest frame
    std::cout<<"Just aligned frame "<<reference->frameID()<<" and "<<frame->frameID()<<std::endl;
    this->lastTrackingCos = cvo_align->function_angle(*(reference->getTrackingPoints()),
                                                      *(new_frame_pcd),
                                                      trackingPoseResult.inverse(),
                                                      cvo_align->get_params().ell_init,
                                                      false);
    std::cout<<"Tracking cvo function_angle is "<<lastTrackingCos<<std::endl;
    //std::cout<<"CVO result frameToRefPose is "<<frameToRefPose.matrix()<<std::endl;
    
    frame->setTrackingResult(reference.get(), frameToRefPose);

    // error distribution
    //frame->setErrorDistribution(errorDistribution);

    // residuals per level
    //this->lastTrackedResidual = this->tracker->totalResidual();

    //if (this->trackingReference->firstFrameResidual() < 0.f)
    // {
    //  this->trackingReference->setFirstFrameResidual(this->lastTrackedResidual);
    // }

    // save info for tracking priors
    this->lastTrackedMotion = lastToRef.inverse() * frameToRefPose;
    this->lastTrackedFrame = frame;

    Utils::Time t2 = std::chrono::steady_clock::now();
    //std::cout << "Tracking: " << Utils::elapsedTime(t1, t2) << "\n";
    
  
  }

  void FullSystem::trackFrameToFrameCvo( std::shared_ptr<Frame>& frame){

    Utils::Time t1 = std::chrono::steady_clock::now();

    // select tracking reference keyframe
    Sophus::SE3f lastToWorldPose = this->lastTrackedFrame->parent()->camToWorld() *
      this->lastTrackedFrame->thisToParentPose();

    //AffineLight lastToWorldLight = AffineLight::calcGlobal(this->lastTrackedFrame->parent()->affineLight(),
    //                                                      this->lastTrackedFrame->thisToParentLight());

    // if a new kf (starting from the third one) is setup in CreateNewKFAndOptimize,
    // update the tracking reference, the initial guess pose and light
    if (this->trackingReferenceUpdated)
    {
      std::lock_guard<std::mutex> lock(this->trackingReferenceMutex);
      std::swap(this->trackingReference, this->newTrackingReference);
      std::cout<<"setting this->createNewKeyframe to false"<<std::endl;
      this->createNewKeyframe = false;
      this->trackingReferenceUpdated = false;

      Frame::Ptr newReference = this->trackingReference;
      Frame::Ptr oldReference = this->newTrackingReference;

      if (newReference && oldReference)
      {
        // correct last frame pose with PBA result
        // oldReference = parent of newReference

        // pose
        //const Sophus::SE3f poseCorrection = newReference->camToWorld().inverse() * 
        //  oldReference->camToWorld() * 
        //  newReference->thisToParentPose();

        //lastToWorldPose *= poseCorrection.inverse();

        // light
        //const AffineLight newRefOldLight = AffineLight::calcGlobal(newReference->parent()->affineLight(),
        //                                                            newReference->thisToParentLight());
      //const AffineLight lightCorrection = AffineLight::calcRelative(newRefOldLight,
        //                                                              newReference->affineLight());
      //  lastToWorldLight = AffineLight::calcGlobal(lastToWorldLight,
                                                   //                                           lightCorrection);
      }
    }
    Frame::Ptr lastTrackedFrame = this->lastTrackedFrame;
    Frame::Ptr reference = this->trackingReference;
    Eigen::Matrix4f lastTrackedMotionEigen = this->lastTrackedMotion.matrix();
    // pose prior -> constant velocity model
    std::cout<<"\nCvo last frame id "<<lastTrackedFrame->frameID()<<"'s lastToWorldPose is "<<lastToWorldPose.matrix()<<std::endl;
    Sophus::SE3f lastToRef = reference->camToWorld().inverse() * lastToWorldPose;
    //Sophus::SE3f frameToRefPose = lastToRef * this->lastTrackedMotion;
    //Eigen::Matrix4f frameToRefPoseEigen = frameToRefPose.matrix().inverse();
    Eigen::Matrix4f initialGuess = lastTrackedMotionEigen.inverse();

    // affine light prior
    //AffineLight frameToRefLight = AffineLight::calcRelative(reference->affineLight(),
    //                                                        lastToWorldLight);

    // Error distribution
    //std::shared_ptr<IDistribution> errorDistribution;

    // Track
    // relative pose to reference keyframe
    std::cout<<"Cvo trying to align frame "<<lastTrackedFrame->frameID()<<" and frame "<<frame->frameID()<<std::endl;
    pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd = frame->getTrackingPoints();
    Eigen::Matrix4f trackingPoseResult;
    double trackingTime;
    int cvoTrackingResult = cvo_align->align(*(this->lastTrackedFrame->getTrackingPoints()),
                                             *(new_frame_pcd),
                                             initialGuess,
                                             // outputs
                                             trackingPoseResult,
                                             nullptr,
                                             &trackingTime);
    Sophus::SE3f resultSophus(trackingPoseResult.block<3,3>(0,0), trackingPoseResult.block<3,1>(0,3));
    
    // frameToRefPose = resultSophus; // convert back to sophus
    
    // track relative affine light to reference keyframe
    //bool goodTrackedLight = this->tracker->trackFrame(this->trackingReference, frame, frameToRefPose, frameToRefLight,                                                                                                                 errorDistribution, this->outputWrapper, true);      

    //tracking lost? - reset tracking internal data
    bool goodTracked = (cvoTrackingResult == 0);
    //bool goodTracked = (cvoTrackingResult == 0 && goodTrackedLight);
    if (!goodTracked)
    {
      this->trackingIsGood = false;

      this->lastTrackedFrame = nullptr;
      this->lastTrackedMotion = Sophus::SE3f();

      this->trackingReference = nullptr;
      //this->tracker->reset();

      this->unmappedTrackedFramesSignal.notify_one();

      return;
    }

    // save result to the latest frame
    Sophus::SE3f frameToRefPose = lastToRef * resultSophus;
    Eigen::Matrix4f frameToRefPoseEigen = frameToRefPose.matrix();
    std::cout<<"Just aligned frame "<<reference->frameID()<<" and "<<frame->frameID()<<", ";
    std::cout<<"reference frame to lastest frame pose is "<<frameToRefPoseEigen<<std::endl;    
    this->lastTrackingCos = cvo_align->function_angle(*(reference->getTrackingPoints()),
                                                      *(new_frame_pcd),
                                                      frameToRefPoseEigen.inverse(),
                                                      cvo_align->get_params().ell_init,
                                                      false);
    std::cout<<"Tracking cvo function_angle is "<<this->lastTrackingCos<<std::endl;

    // convert to frame-to-ref
    frame->setTrackingResult(reference.get(), frameToRefPose);

    // error distribution
    //frame->setErrorDistribution(errorDistribution);

    // residuals per level
    //this->lastTrackedResidual = this->tracker->totalResidual();

    //if (this->trackingReference->firstFrameResidual() < 0.f)
    // {
    //  this->trackingReference->setFirstFrameResidual(this->lastTrackedResidual);
    // }

    // save info for tracking priors
    this->lastTrackedMotion = resultSophus;
    this->lastTrackedFrame = frame;

    Utils::Time t2 = std::chrono::steady_clock::now();
    
    std::cout << "Tracking: " << Utils::elapsedTime(t1, t2) << "\n";
    std::string msg = "Tracking: " + std::to_string(Utils::elapsedTime(t1, t2)) + "\t";
    auto& log = Log::getInstance();
    log.addNewLog(frame->frameID());
    log.addCurrentLog(frame->frameID(), msg);
  
  }

  
  /*
  void FullSystem::trackNewFrame(const std::shared_ptr<Frame>& frame)
  {
    Utils::Time t1 = std::chrono::steady_clock::now();

    // select tracking reference keyframe
    Sophus::SE3f lastToWorldPose = this->lastTrackedFrame->parent()->camToWorld() *
      this->lastTrackedFrame->thisToParentPose();

    AffineLight lastToWorldLight = AffineLight::calcGlobal(this->lastTrackedFrame->parent()->affineLight(),
                                                           this->lastTrackedFrame->thisToParentLight());

    if (this->trackingReferenceUpdated)
    {
      std::lock_guard<std::mutex> lock(this->trackingReferenceMutex);
      std::swap(this->trackingReference, this->newTrackingReference);
      this->createNewKeyframe = false;
      this->trackingReferenceUpdated = false;

      Frame* const newReference = this->trackingReference->reference();
      Frame* const oldReference = this->newTrackingReference->reference();

      if (newReference && oldReference)
      {
        // correct last frame pose with PBA result
        // oldReference = parent of newReference

        // pose
        const Sophus::SE3f poseCorrection = newReference->camToWorld().inverse() * 
          oldReference->camToWorld() * 
          newReference->thisToParentPose();

        lastToWorldPose *= poseCorrection.inverse();

        // light
        const AffineLight newRefOldLight = AffineLight::calcGlobal(newReference->parent()->affineLight(),
                                                                   newReference->thisToParentLight());
        const AffineLight lightCorrection = AffineLight::calcRelative(newRefOldLight,
                                                                      newReference->affineLight());
        lastToWorldLight = AffineLight::calcGlobal(lastToWorldLight,
                                                   lightCorrection);
      }
    }

    Frame* const reference = this->trackingReference->reference();

    // pose prior -> constant velocity model
    std::cout<<"camToWorld is "<<reference->camToWorld().matrix()<<"\n, lastToWorldPose is "<<lastToWorldPose.matrix()<<std::endl;    
    const Sophus::SE3f lastToRef = reference->camToWorld().inverse() * lastToWorldPose;
    Sophus::SE3f frameToRefPose = lastToRef * this->lastTrackedMotion;
    std::cout<<"frameToRefPose is "<<frameToRefPose.matrix()<<std::endl;

    // affine light prior
    AffineLight frameToRefLight = AffineLight::calcRelative(reference->affineLight(),
                                                            lastToWorldLight);

    // Error distribution
    std::shared_ptr<IDistribution> errorDistribution;

    // Track
    // relative pose to reference keyframe
    // relative affine light to reference keyframe
    bool goodTracked = this->tracker->trackFrame(this->trackingReference, frame, frameToRefPose, frameToRefLight,
                                                 errorDistribution, this->outputWrapper);

    //tracking lost? - reset tracking internal data
    if (!goodTracked)
    {
      this->trackingIsGood = false;

      this->lastTrackedFrame = nullptr;
      this->lastTrackedMotion = Sophus::SE3f();

      this->trackingReference->reset();
      this->tracker->reset();

      this->unmappedTrackedFramesSignal.notify_one();

      return;
    }

    // save result to the latest frame
    std::cout<<"Image Alignment frameToRefPose is "<<frameToRefPose.matrix()<<std::endl;
    frame->setTrackingResult(reference, frameToRefPose, frameToRefLight);

    // error distribution
    frame->setErrorDistribution(errorDistribution);

    // residuals per level
    this->lastTrackedResidual = this->tracker->totalResidual();

    if (this->trackingReference->firstFrameResidual() < 0.f)
    {
      this->trackingReference->setFirstFrameResidual(this->lastTrackedResidual);
    }

    // save info for tracking priors
    this->lastTrackedMotion = lastToRef.inverse() * frameToRefPose;
    this->lastTrackedFrame = frame;

    Utils::Time t2 = std::chrono::steady_clock::now();
    //std::cout << "Tracking: " << Utils::elapsedTime(t1, t2) << "\n";
  }
  */

  void FullSystem::trackFrame(int id, double timestamp, Frame::Ptr trackingNewFrame) 
  {
    auto& settings = Settings::getInstance();

    // track frame
    Utils::Time t1 = std::chrono::steady_clock::now();

    // Create new frame
    //std::shared_ptr<Frame> trackingNewFrame = std::make_shared<Frame>(id, timestamp, image, left_img, new_frame_pcd, depth_img, depth_scale);

    // store all frames
    allFrames.push_back(trackingNewFrame);

    if (settings.debugPrintLog)
    {
      auto& log = Log::getInstance();
      log.addNewLog(trackingNewFrame->frameID());
    }

    if (!this->trackingIsGood)
    {
      std::cout << "LOST..." << std::endl;
      return;
    }

    // initialization: TODO
    if (!this->initialized)
    {
      this->initialized = this->initialize_lmcw(trackingNewFrame);
      return;
    }

    // track
    this->trackFrameToFrameCvo(trackingNewFrame);
    //this->trackModelToFrameCvo(trackingNewFrame);

    if (!this->trackingIsGood)
    {
      std::cout << "Tracking LOST!!" << std::endl;
      return;
    }

    Utils::Time t2;
    const float time = Utils::elapsedTime(t1, t2);
    this->camTrackingTime.push_back(time);
    
    
    if (settings.debugPrintLog && settings.debugLogTracking)
    {
      const float* pose = trackingNewFrame->thisToParentPose().data();
      const AffineLight& light = trackingNewFrame->thisToParentLight();
      const float energy = this->tracker->totalResidual();

      std::string msg = "trackingPose: " + std::to_string(pose[0]) + "\t" + std::to_string(pose[1]) + "\t" + std::to_string(pose[2]) + "\t" + std::to_string(pose[3]) + "\t"
        + std::to_string(pose[4]) + "\t" + std::to_string(pose[5]) + "\t" + std::to_string(pose[6]) + "\t";
      msg += "affineLight: " + std::to_string(light.alpha()) + "\t" + std::to_string(light.beta()) + "\t";
      msg += "cost: " + std::to_string(energy) + "\t";

      auto& log = Log::getInstance();
      log.addCurrentLog(trackingNewFrame->frameID(), msg);

    }

    

    // Keyframe selection
    // If we have already asked to create one, dont do it again
    bool localCreateNewKeyframe = false;
    // TODO: update for multi thread case
    //if (!this->createNewKeyframe) {
    //  if(this->numTrackedFramesFromLastKF >= settings.minNumTrackedFramesToCreateKF) {
        localCreateNewKeyframe = this->isNewKeyframeRequired(trackingNewFrame);
        //  }
      this->createNewKeyframe = localCreateNewKeyframe;
      //}
    std::cout<<"Create new keyframe is "<<this->createNewKeyframe<<std::endl;

    // insert current frame to unmapped queue
    Utils::Time t3;
    if (!settings.singleThreaded)
    {
      {
        std::lock_guard<std::mutex> unMappedLock(this->unmappedTrackedFramesMutex);
        this->unmappedTrackedFrames.push_back(trackingNewFrame);

        if (localCreateNewKeyframe)
        {
          this->createNewKeyframeID = trackingNewFrame->frameID();
        }

        // control flag for blocking
        {
          std::lock_guard<std::mutex> newFrameMappedLock(this->newFrameMappedMutex);
          this->newFrameMappedDone = false;
        }
      }
      this->unmappedTrackedFramesSignal.notify_one();

      t3 = std::chrono::steady_clock::now();
    }
    else
    {
      if (localCreateNewKeyframe)
      {
        this->createNewKeyframeID = trackingNewFrame->frameID();
      }
      
      t2 = std::chrono::steady_clock::now();
      
      this->doMapping(trackingNewFrame);
    }

    //const float time = Utils::elapsedTime(t1, t2);
    //this->camTrackingTime.push_back(time);

    if (this->outputWrapper)
    {
      // current camera pose
      const Eigen::Matrix4f camPose = (this->lastTrackedFrame->parent()->camToWorld() * 
                                       this->lastTrackedFrame->thisToParentPose()).matrix();
      this->outputWrapper->publishCurrentFrame(camPose);

      //timings			
      this->outputWrapper->publishCamTrackingTime(time);
    }

    // implement blocking
    // required for debugging
    if (!settings.singleThreaded && settings.blockUntilMapped && this->trackingIsGood)
    {
      this->waitUntilMappingFinished();
    }
  }

  
  bool FullSystem::isNewKeyframeRequired(const std::shared_ptr<Frame>& frame) const
  {
    const auto& settings = Settings::getInstance();

    auto reference = this->trackingReference;

    // check camera translation relative to mean inverse depth
    //const Eigen::Vector3f dist = frame->thisToParentPose().translation() * 
    //  this->trackingReference->meanIDepth();

    // check point usage by tracker 
    //const float pointUsage = this->tracker->pointUsage();

    // check illumination change
    //const AffineLight& relativeLight = frame->thisToParentLight();

    // new keyframe required?
    std::cout<<"isNewKeyframeRequired: lastTrackingCos is "<<this->lastTrackingCos<<", coslimit is "<<settings.trackingCosLimit<<std::endl;
    if (//(dist.norm()*settings.newKFDistWeight +
        // (1.f - pointUsage) * settings.newKFUsageWeight +
        // fabs(relativeLight.alpha()) * settings.newKFAffineWeight) > 1.f ||
        //settings.newKFResidualWeight*this->trackingReference->firstFrameResidual() < this->lastTrackedResidual ||
        this->lastTrackingCos < settings.trackingCosLimit)
    {
      return true;
    }

    return false;
  }

  void FullSystem::mappingThreadLoop()
  {
    std::cout << "Started mapping thread!" << std::endl;

    const auto& settings = Settings::getInstance();

    Utils::Time t1, t2;

    while (true)
    {
      std::shared_ptr<Frame> frame;

      // waiting condition. It will wake up when a new frame is set in the queue 
      // or when shutting down
      {
        std::unique_lock<std::mutex> unMappedLock(this->unmappedTrackedFramesMutex);
        std::cout<<" unmappedTrackedFrames size is "<<this->unmappedTrackedFrames.size()<<std::endl;
        if (this->unmappedTrackedFrames.empty())
        {
          // signal to stop blocking
          {
            std::lock_guard<std::mutex> newFrameMappedLock(this->newFrameMappedMutex);
            this->newFrameMappedDone = true;
            //std::cout << "wake up!" << std::endl;
          }
          this->newFrameMappedSignal.notify_all();

          this->unmappedTrackedFramesSignal.wait(
                                                 unMappedLock, [&]() {return this->shouldStop || !this->unmappedTrackedFrames.empty(); }
                                                 );

          if (this->shouldStop)
          {
            return;
          }
        }	

        t1 = std::chrono::steady_clock::now();

        // take a new frame from the front
        frame = std::move(this->unmappedTrackedFrames.front());
        this->unmappedTrackedFrames.pop_front();

        // some checkings
        if (!this->unmappedTrackedFrames.empty())
        {
          const auto& activeKeyframes = this->lmcw->activeWindow();

          // if new keyframe required, discard all the frames until the need was requested
          if (this->createNewKeyframeID > activeKeyframes.back()->frameID())
          {
            while (this->unmappedTrackedFrames.size() > 0 &&
                   this->unmappedTrackedFrames.front()->frameID() <= this->createNewKeyframeID)
            {
              frame = std::move(this->unmappedTrackedFrames.front());
              this->unmappedTrackedFrames.pop_front();
            }
          }
				
          // check if mapping thread is running too slow
          // and the number of unmapped frames is growing
          if (this->unmappedTrackedFrames.size() > settings.maxUnmappedFrames)
          {
            std::cout << "Unmapped frames is growing.." << std::endl;
				
            // then, skip the following frame
            this->unmappedTrackedFrames.pop_front();
          }
        }				
      }

      // do mapping stuff
      this->doMapping(frame);
    }
    std::cout << "Finished mapping thread!" << std::endl;
  }

  void FullSystem::doMapping(const std::shared_ptr<Frame>& frame)
  {
    assert(frame != nullptr);

    Utils::Time t1 = std::chrono::steady_clock::now();

    const auto& activeKeyframes = this->lmcw->activeWindow();

    // do mapping stuff
    if (this->createNewKeyframeID > activeKeyframes.back()->frameID())
    {
      // create new keyframe and optimize
      this->createKeyframeAndOptimize(frame);
      this->numTrackedFramesFromLastKF = 0;

      Utils::Time t2 = std::chrono::steady_clock::now();
      const float time = Utils::elapsedTime(t1, t2);
      this->totalBackendTime.push_back(time);

      if (this->outputWrapper)
      {
        this->outputWrapper->publishLocalBATime(time);
      }
    }
    else
    {
      // track previous keyframes's candidates with the tracked frame.
      // The newly tracked frame is not a keyframe
      this->trackCandidatesCvo(frame);
      //this->lmcw->allKeyframes()[lmcw->allKeyframes().size()-1]->dump_candidates_to_pcd(std::to_string(numTrackedFramesFromLastKF)+".pcd");
      this->numTrackedFramesFromLastKF++;

      Utils::Time t2 = std::chrono::steady_clock::now();
      const float time = Utils::elapsedTime(t1, t2);
      this->pointTrackingTime.push_back(time);

      if (this->outputWrapper)
      {
        this->outputWrapper->publishPointTrackingTime(time);
      }
    }
  }

  void FullSystem::waitUntilMappingFinished()
  {
    std::unique_lock<std::mutex> newFrameMappedLock(this->newFrameMappedMutex);
    //std::cout << "waiting..." << std::endl;
    this->newFrameMappedSignal.wait(newFrameMappedLock, [&]() {return this->newFrameMappedDone; });
  }


  static int FAST_corner_sampling(const cv::Mat & left_gray,
                                  int expected_points,
                                  // output
                                  std::vector<int32_t> & selected_inds_map
                                  ) {
    
    std::vector<cv::KeyPoint> keypoints;    



    int maxKeypoints, minKeypoints;
    
    //cv::Ptr<cv::FastAdjuster::FastAdjuster> adjust = new cv::FastAdjuster::FastAdjuster();
    //cv::Ptr<cv::FeatureDetector> detector = new cv::DynamicAdaptedFeatureDetector::DynamicAdaptedFeatureDetector(adjust,500,1000,5);
    //vector<KeyPoint> keypoints;
    //detector->detect(left_gray, keypoints);
    //std::cout<<"Fast sampled "<<keypoints.size()<<" points \n";    
    cv::FAST(left_gray, keypoints, 20,false);    
    cv::KeyPointsFilter::retainBest(keypoints, 1000);
    for (auto && kp: keypoints) {
      int c = (int)kp.pt.x;
      int r = (int)kp.pt.y;
      selected_inds_map[r * left_gray.cols + c] = 0;
    }
    std::cout<<"after cv::retainBest, we obtain "<<keypoints.size()<<" points\n";
    return keypoints.size();
    
  }
  

  static int stereo_surface_sampling(const cv::Mat & left_gray,
                                     bool is_using_canny,
                                     bool is_using_uniform_rand,
                                     int expected_points,
                                     // output
                                     std::vector<int32_t> & selected_inds_map
                                     //std::vector<Vec2i, Eigen::aligned_allocator<Vec2i>> & final_selected_uv                   
                                     ) {
    //selected_inds_map.resize(left_gray.total(), false);
    
    // canny
    cv::Mat detected_edges;
    if (is_using_canny)
      cv::Canny( left_gray, detected_edges, 50, 50*3, 3 );
    int counter = 0;
    
    std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i>> tmp_uvs_canny, tmp_uvs_surface;
    for (int r = 0 ; r < left_gray.rows; r++) {
      for (int c = 0; c < left_gray.cols; c++) {
        // using Canny
        if (is_using_canny &&  detected_edges.at<uint8_t>(r, c) > 0)  {
          //selected_inds_map[r * left_gray.cols + c] = true;
          tmp_uvs_canny.push_back(Eigen::Vector2i(c, r));
        }
        selected_inds_map[r * left_gray.cols + c] = -1;
      }
      
    }
    for (int r = 0 ; r < left_gray.rows; r++) {
      for (int c = 0; c < left_gray.cols; c++) {
        
        // using uniform sampling
        if ( (!is_using_canny || detected_edges.at<uint8_t>(r, c) == 0 ) &&
            is_using_uniform_rand &&
            //r > left_gray.rows   &&
            rand() % 10 == 0)  {
          //selected_inds_map[r * left_gray.cols + c] = true;

          tmp_uvs_surface.push_back(Eigen::Vector2i(c, r));
        }

      }
      
    }
    std::cout<<"Canny size "<<tmp_uvs_canny.size()<<", surface size "<<tmp_uvs_surface.size()<<"\n";
    int total_selected_canny = tmp_uvs_canny.size();
    int total_selected_surface = tmp_uvs_surface.size();
    int total = 0;
    int found = 0;
    for (int i = 0; i < tmp_uvs_canny.size(); i++) {
      if (rand() % total_selected_canny < expected_points * 3 / 5 ) {
        //final_selected_uv.push_back(tmp_uvs_canny[i]);
        auto c = tmp_uvs_canny[i](0);        
        auto r = tmp_uvs_canny[i](1);
        selected_inds_map[r * left_gray.cols + c] = 0;
        total++;
      }
      
    }
    for (int i = 0; i < tmp_uvs_surface.size(); i++) {
      if (rand() % total_selected_surface < expected_points * 2 / 5 ) {
        //final_selected_uv.push_back(tmp_uvs_surface[i]);
        auto c = tmp_uvs_surface[i](0);        
        auto r = tmp_uvs_surface[i](1);
        selected_inds_map[r * left_gray.cols + c] = 0;
        total++;
      }
      
    }
    
    return total;
    //std::cout<<" final selected uv size is "<<final_selected_uv.size()<<std::endl;
    //cv::imwrite("canny.png", detected_edges);    
    
  }

  

  

  void FullSystem::createCandidates(const std::shared_ptr<Frame>& frame)
  {
    const auto& calib = GlobalCalibration::getInstance();
    const auto & intrinsic = calib.matrix3f(0);
    float baseline = calib.getBaseline();
    const auto& settings = Settings::getInstance();

    const int32_t width = calib.width(0);
    const int32_t height = calib.height(0);
    const int32_t distToBorder = Pattern::padding() + 1;

    auto & rawImg = * (frame->getRawImage());

    // Create new candidates in the frame
    // They have to be homogeneously distributed in the image	
    // int num = this->pointDetector->detect(frame, (int)settings.numCandidates, this->pixelMask,
    //                                     this->outputWrapper);

    cv::Mat left_gray;
    cv::cvtColor(rawImg.image(), left_gray, cv::COLOR_BGR2GRAY);
    //std::vector<int32_t> selected_inds_map(this->pixelMask, this->pixelMask + sizeof(int32_t) * left_gray.total());
    std::vector<int32_t> selected_inds_map(left_gray.rows * left_gray.cols);
    std::fill(selected_inds_map.begin(), selected_inds_map.end(), -1);
    
    int num = 0;
    if (settings.candidatePointsSampling == 0)
      num = stereo_surface_sampling(left_gray,
                                    true,
                                    true,
                                    1500,
                                    // output
                                    selected_inds_map
                                    //std::vector<Vec2i, Eigen::aligned_allocator<Vec2i>> & final_selected_uv                   
                                    );
    else
      num = FAST_corner_sampling(left_gray, settings.numPointsPerFrame,
                                 selected_inds_map);
    

    // create candidates
    auto& candidates = frame->candidates();
    candidates.reserve(num);

    int cc = 0;

    Utils::Time t1 = std::chrono::steady_clock::now();

    for (int32_t row = distToBorder; row < height - distToBorder; ++row)
    {
      for (int32_t col = distToBorder; col < width - distToBorder; ++col)
      {
        int32_t idx = col + row * width;

        //if (this->pixelMask[idx] < 0) continue;
        if (selected_inds_map[idx] != 0) continue;
        cc++;
        // create a point
        if (cvo_align!=nullptr) {
          float idepth = 0;
          if (frame->depthType == Frame::DepthType::STEREO)
            idepth = std::dynamic_pointer_cast<cvo::ImageStereo> (frame->getRawImage())->disparity()[width * row + col] / (std::abs(baseline) * intrinsic(0,0));
          else if (frame->depthType == Frame::DepthType::RGBD)
            idepth =  frame->depthScale / (std::dynamic_pointer_cast<cvo::ImageRGBD>(frame->getRawImage())->depth_image()[width * row + col]); 
          if ( ! std::isfinite(idepth) || idepth < 0.03f ) continue;
          std::unique_ptr<CandidatePoint> new_candidate(new CandidatePoint((float)col, (float)row, selected_inds_map[idx] , frame, idepth));

          candidates.emplace_back(std::move(new_candidate));
            //D}
        } else
          candidates.emplace_back(std::make_unique<CandidatePoint>((float)col, (float)row, selected_inds_map[idx] , frame));
      }
    }
    
    candidates.shrink_to_fit();
    frame->initCandidateQualityFlag();
    std::cout<<"Create "<<candidates.size()<<" new candidates for frame "<<frame->frameID()<<std::endl;

    Utils::Time t2 = std::chrono::steady_clock::now();
		
    if (settings.debugPrintLog && settings.debugLogPixelDetection)
    {
      const std::string msg = "PointDetect: " + std::to_string(candidates.size()) + "\t";

      auto& log = Log::getInstance();
      log.addCurrentLog(frame->frameID(), msg);
    }

    // reserve memory
    auto& activePoints = frame->activePoints();
    activePoints.reserve(candidates.size());

    //std::string fname("candidates_after_creation.pcd");
    //frame->dump_candidates_to_pcd(fname);
    std::cout << "Select pixels: " << Utils::elapsedTime(t1, t2) << std::endl;
  }

  void FullSystem::trackCandidatesCvo(const Frame::Ptr frame, bool include_curr) {

    const auto& settings = Settings::getInstance();
    const auto& calib = GlobalCalibration::getInstance();
    const auto& K = calib.matrix3f(0);
    const auto& Kinv = calib.invMatrix3f(0);
    
    const auto& activeKeyframes = this->lmcw->activeWindow();
    std::shared_ptr<cvo::CvoPointCloud> candidates_curr;
    /*
    if (frame->candidates().size() == 0)
      candidates_curr.reset(new cvo::CvoPointCloud(*frame->getTrackingPoints()));
     else {
      candidates_curr.reset(new cvo::CvoPointCloud);
      frame->candidatesToCvoPointCloud(*candidates_curr);
      }*/
    candidates_curr = frame->getFullPoints();
    
    Sophus::SE3f camToRef = frame->thisToParentPose();
    Frame * const parent = frame->parent();
    
    
    //if (settings.debugCandidates || (settings.debugPrintLog && settings.debugLogCandidatesTracking))
    //{
    int numGood = 0;
    int numNegativeIDepth = 0;

    for (const auto& kf : activeKeyframes) {
      // clean candidate vector
      // remove outliers
      auto& candidates = kf->candidates();
      cvo::CvoPointCloud candidates_cvo;
      kf->candidatesToCvoPointCloud(candidates_cvo);

      Sophus::SE3f parentToCurrKF = kf->camToWorld().inverse() *  parent->camToWorld();
      Sophus::SE3f kfToFrame = (parentToCurrKF * camToRef).inverse();
      Eigen::Matrix4f kfToFrameEigen = kfToFrame.matrix();
        
      cvo::Association association_mat;
      Eigen::Matrix3f kernel;
      if (settings.enableDepthRegression)
        kernel << 0.01, 0, 0,
          0, 0.01, 0,
          0,  0,   0.03;
      else
        kernel << 0.1, 0, 0,
          0, 0.1, 0,
          0, 0, 0.1;
      cvo_align->compute_association_gpu(candidates_cvo,
                                         *candidates_curr,
                                         kfToFrameEigen,
                                         kernel,
                                         association_mat);
      int counter = 0;
      for (int j = 0; j < candidates.size(); j++) {
        kf->candidates()[j]->setStatus( CandidatePoint::PointStatus::OPTIMIZED);                    
        counter++;
      }

      //for (int j=0; j < association_mat.outerSize(); ++j) {
      /*
      for (int j = 0; j < association_mat.source_inliers.size(); j++) {
        int kfPtIdx = association_mat.source_inliers[j];
        kf->candidatesHighQuaity()[kfPtIdx] = CandidatePoint::PointStatus::OPTIMIZED;
        kf->candidates()[kfPtIdx]->setStatus( CandidatePoint::PointStatus::OPTIMIZED);
        for (Eigen::SparseMatrix<float, Eigen::RowMajor>::InnerIterator it(association_mat.pairs,kfPtIdx); it; ++it) {
          float weight = it.value();
           int idx_target = it.col();   // col index (here it is equal to k)
          Eigen::Vector3f XYZ = candidates_curr->positions()[idx_target]; //frame->candidates()[idx_target]->xyz();

          float obsIdepth = 1/((K * (kfToFrame.inverse() * XYZ))(2));
          //std::cout<<"matching "<<kfPtIdx<<" with "<<idx_target<<" with weight "<<weight<<" and idepth in kf "<<obsIdepth<<std::endl;          
          kf->candidates()[kfPtIdx]->addIdepthObservation(obsIdepth,weight);
        }
        //}
        if (kf->candidatesHighQuaity()[j] == CandidatePoint::PointStatus::OPTIMIZED)
          counter++;
          }*/
      std::cout<<"Frame "<<kf->frameID()<<" has "<<counter<<" traced points\n"<<std::flush;

      if (include_curr) {
        counter = 0;
        for (int j = 0; j < association_mat.target_inliers.size(); j++) {
          //if (association_mat.target_inliers[j] ) {
          int framePtIdx = association_mat.target_inliers[j];
          //frame->candidatesHighQuaity()[framePtIdx] = CandidatePoint::PointStatus::OPTIMIZED;
          frame->candidates()[framePtIdx]->setStatus( CandidatePoint::PointStatus::OPTIMIZED);            
            //}
          //if (frame->candidatesHighQuaity()[framePtIdx] == CandidatePoint::PointStatus::OPTIMIZED)
          //  counter++;
        }
        std::cout<<"Frame "<<frame->frameID()<<" has "<<counter<<" traced points\n";
      } else {
        
      }

      if (settings.debugCandidates) {
        // std::cout<<"Frame "<<kf->frameID()<<" has "<<counter<<" traced points\n";
      }
      /*
        for (const auto& cand : candidates)
        {
        // skip if the point is an outlier or is not visible
        if (cand->status() == CandidatePoint::OUTLIER ||
        cand->lastObservation() == CandidatePoint::OOB)
        {
        continue;
        }
        else
        {
        CandidatePoint::ObserveStatus status =  cand->observe(frame);

        if (status == CandidatePoint::GOOD) numGood++;
        else {
            
        }
        } */
    }
      
      //}
    /*
      // try to initialize candidates
      // do it in parallel
      int itemsCount = (int)toObserve.size();

      int start = 0;
      int end = 0;
      int step = (itemsCount + settings.mappingThreads - 1) / settings.mappingThreads;

      for (int i = 0; i < settings.mappingThreads; ++i)
      {
        start = end;
        end = std::min(end + step, itemsCount);

        this->threadPool->addJob(PointObserver(toObserve, frame, start, end));
      }

      // wait until finish
      this->threadPool->wait();
      }*/
    
  }
  

  // depth filtering by tracing the active KF's candidate points on the new non-keyframe
  void FullSystem::trackCandidates(const std::shared_ptr<Frame>& frame)
  {
    const auto& settings = Settings::getInstance();

    const auto& activeKeyframes = this->lmcw->activeWindow();

    if (settings.debugCandidates || (settings.debugPrintLog && settings.debugLogCandidatesTracking))
    {
      int numGood = 0;
      int numBadError = 0;
      int numBadEpiLine = 0;
      int numOOB = 0;
      int numSkipped = 0;
      int numBadConditioned = 0;
      int numNegativeIDepth = 0;

      for (const auto& kf : activeKeyframes)
      {
        // clean candidate vector
        // remove outliers
        const auto& candidates = kf->candidates();

        for (const auto& cand : candidates)
        {
          // skip if the point is an outlier or is not visible
          if (cand->status() == CandidatePoint::OUTLIER ||
              cand->lastObservation() == CandidatePoint::OOB)
          {
            continue;
          }
          else
          {
            CandidatePoint::ObserveStatus status =  cand->observe(frame);

            if (status == CandidatePoint::GOOD) numGood++;
            else if (status == CandidatePoint::BAD_ERROR) numBadError++;
            else if (status == CandidatePoint::BAD_EPILINE) numBadEpiLine++;
            else if (status == CandidatePoint::OOB) numOOB++;
            else if (status == CandidatePoint::SKIPPED) numSkipped++;
            else if (status == CandidatePoint::BAD_CONDITIONED) numBadConditioned++;
            else if (cand->iDepth() < 0.f) numNegativeIDepth++;
          }
        }
      }

      if (settings.debugCandidates)
      {
        std::cout << "Candidates good: " << numGood << std::endl;
        std::cout << "Candidates bad error: " << numBadError << std::endl;
        std::cout << "Candidates bad epiline: " << numBadEpiLine << std::endl;
        std::cout << "Candidates oob: " << numOOB << std::endl;
        std::cout << "Candidates skipped: " << numSkipped << std::endl;
        std::cout << "Candidates bad_cond: " << numBadConditioned << std::endl;
        std::cout << "Candidates neg_iDepth: " << numNegativeIDepth << std::endl;
        std::cout << std::endl;
      }

      if (settings.debugPrintLog && settings.debugLogCandidatesTracking)
      {
        const std::string msg = "Candidates: " + std::to_string(numGood) + "\t" + std::to_string(numBadError) + "\t" + std::to_string(numBadEpiLine) + "\t"
          + std::to_string(numOOB) + "\t" + std::to_string(numSkipped) + "\t" + std::to_string(numBadConditioned) + "\t";

        auto& log = Log::getInstance();
        log.addCurrentLog(frame->frameID(), msg);
      }
    }
    else
    {
      // vector of candidates to proccess
      std::vector<CandidatePoint*> toObserve;

      for (const auto& kf : activeKeyframes)
      {
        const auto& candidates = kf->candidates();
        for (const auto& cand : candidates)
        {
          // skip if the point is an outlier or is not visible
          if (cand->status() == CandidatePoint::OUTLIER ||
              cand->lastObservation() == CandidatePoint::OOB)
          {
            continue;
          }
          else
          {
            // if still visible
            toObserve.push_back(cand.get());
          }
        }
      }

      // try to initialize candidates
      // do it in parallel
      int itemsCount = (int)toObserve.size();

      int start = 0;
      int end = 0;
      int step = (itemsCount + settings.mappingThreads - 1) / settings.mappingThreads;

      for (int i = 0; i < settings.mappingThreads; ++i)
      {
        start = end;
        end = std::min(end + step, itemsCount);

        this->threadPool->addJob(PointObserver(toObserve, frame, start, end));
      }

      // wait until finish
      this->threadPool->wait();
    }
  }

  void FullSystem::refineCandidates()
  {
    const auto& settings = Settings::getInstance();

    const auto& calib = GlobalCalibration::getInstance();
    const Eigen::Matrix3f& K = calib.matrix3f(0);
    const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
    const int width = (int)calib.width(0);
    const int height = (int)calib.height(0);

    const auto& activeKeyframes = this->lmcw->activeWindow();
    const auto temporalKeyframes = this->lmcw->temporalWindow();
    const std::shared_ptr<Frame>& lastKeyframe = temporalKeyframes.back();
    const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();

    // vector of candidates to proccess
    std::vector<CandidatePoint*> toOptimize;
    int counter_uninit = 0;
    int counter_OOB = 0;
    int counter_proj = 0;
    for (int j = 0; j < temporalKeyframes.size() - 1; ++j)
    {
      // relative pose
      const Sophus::SE3f relPose = worldToLast * temporalKeyframes[j]->camToWorld();
      const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
      const Eigen::Vector3f Kt = K * relPose.translation();

      auto& candidates = temporalKeyframes[j]->candidates();
      for (auto& cand : candidates)
      {
        CandidatePoint::PointStatus status = cand->status();
        CandidatePoint::ObserveStatus lastObs = cand->lastObservation();

        // remove if the point is an outlier
        // or if the point has never been observed				
        if (status == CandidatePoint::PointStatus::UNINITIALIZED ||
            lastObs == CandidatePoint::ObserveStatus::BAD_ERROR || 
            lastObs == CandidatePoint::ObserveStatus::BAD_EPILINE)
        {
          //cand = nullptr;
          counter_uninit ++;
          continue;
        }

        bool isGoodCandidate = cand->matchUncertainty() < settings.maxCandidateUncertainty &&
                                                          cand->matchQuality() > settings.minCandidateQuality;

        if (!isGoodCandidate)
        {
          // erase if it is not visible anymore
          if (lastObs == CandidatePoint::OOB ||
              temporalKeyframes[j]->flaggedToDrop())
          {
            //  cand = nullptr;
          }
          counter_OOB++;
          continue;
        }

        // project to last keyframe
        Eigen::Vector2f pointInFrame;
        if (!Utils::project(cand->u0(), cand->v0(), cand->iDepth(),
                            width, height, KRKinv, Kt, pointInFrame))
        {
          counter_proj++;
          //cand = nullptr;
          continue;
        }

        // we have a good candidate!
        // insert to list
        toOptimize.push_back(cand.get());
      }	
    }

    if (settings.debugCandidates || (settings.debugPrintLog && settings.debugLogCandidatesOpt))
    {
      int numGood = 0;
      int numBad = 0;
      int numSkipped = 0;

      int itemsCount = (int)toOptimize.size();

      for (int i = 0; i < itemsCount; ++i)
      {
        toOptimize[i]->optimize(activeKeyframes);

        if (toOptimize[i]->status() == CandidatePoint::PointStatus::OPTIMIZED) numGood++;
        else if (toOptimize[i]->status() == CandidatePoint::PointStatus::TRACED) numSkipped++;
        else if (toOptimize[i]->status() == CandidatePoint::PointStatus::OUTLIER) numBad++;
      }

      if (settings.debugCandidates)
      {
        std::cout << "Candidates opt good: " << numGood << std::endl;
        std::cout << "Candidates opt skipped: " << numSkipped << std::endl;
        std::cout << "Candidates opt bad: " << numBad << std::endl;
        std::cout << std::endl;
      }

      if (settings.debugPrintLog && settings.debugLogCandidatesOpt)
      {
        const std::string msg = "Candidates Opt: " + std::to_string(numGood) + "\t" + std::to_string(numSkipped) + "\t" + std::to_string(numBad) + "\t";

        auto& log = Log::getInstance();
        log.addCurrentLog(lastKeyframe->frameID(), msg);
      }
    }
    else
    {
      // try to refine candidates
      // do it in parallel
      int itemsCount = (int)toOptimize.size();

      int start = 0;
      int end = 0;
      int step = (itemsCount + settings.mappingThreads - 1) / settings.mappingThreads;

      for (int i = 0; i < settings.mappingThreads; ++i)
      {
        start = end;
        end = std::min(end + step, itemsCount);

        this->threadPool->addJob(PointOptimizer(toOptimize, activeKeyframes, start, end));
      }
    }

    // meantime reorder candidates
    for (int j = 0; j < temporalKeyframes.size() - 1; ++j)
    {
      auto& candidates = temporalKeyframes[j]->candidates();		
      for (int i = 0; i < candidates.size(); ++i)
      {
        if (candidates[i] == nullptr)
        {
          candidates[i] = std::move(candidates.back());
          candidates.pop_back();
          i--;
        }
      }
      temporalKeyframes[j]->dump_candidates_to_pcd("candidates_after_optimize.pcd");
    }

    // wait until finish
    
    this->threadPool->wait();
  }

  void FullSystem::dumpFramesToPcd (const std::string & graphDefFileName,
                                    const std::vector<std::shared_ptr<Frame>> & activeKeyframes,
                                    const std::vector<cvo::CvoFrame::Ptr> & cvo_frames,
                                    const std::list<std::pair<int, int>> & edges
                                    ) const {
    std::ofstream outfile(graphDefFileName);

    outfile << cvo_frames.size()<<" "<<edges.size()<<std::endl;
    for (int i = 0; i < cvo_frames.size(); i++) {
      int frameID = activeKeyframes[i]->frameID();
      outfile<<frameID<<" ";
    }
    outfile<<std::endl;
    for (auto && edgePair: edges) {
      outfile << edgePair.first<<" "<<edgePair.second<<std::endl;
    }

    for (int i = 0; i < cvo_frames.size(); i++) {
      outfile<<"\n";
      for (int j = 0; j < 12; j++)
        outfile<<cvo_frames[i]->pose_vec[j]<<" ";
    }

    outfile.close();
    
   
  }

  void FullSystem::cvoMultiAlign(const std::vector<std::shared_ptr<Frame>> & activeKeyframes,
                                 const std::list<std::pair<CovisibilityNode *, CovisibilityNode*>> & edgesCovisibleToTemporal) {
    if (activeKeyframes.size() < 4) return;
    std::vector<cvo::CvoPointCloud> cvo_pcs;
    std::vector<cvo::CvoFrame::Ptr> cvo_frames;
    int temporalStartIndex = this->lmcw->getTemporalWindowSize();
    cvo_pcs.resize(activeKeyframes.size()-1);
    std::vector<bool> const_flags_in_BA(cvo_pcs.size());        
    std::cout<<"CvoMultiAlign: Frames are ";    
    for (int i = 0; i < activeKeyframes.size()-1; i++) {
      auto kf = activeKeyframes[i];
      std::cout<<kf->frameID()<<", ";
      kf->activePointsToCvoPointCloud(cvo_pcs[i]);
      assert(kf->activePoints().size() != 0);
      Eigen::Matrix<double, 4,4, Eigen::RowMajor> kf_to_world = kf->camToWorld().matrix().cast<double>();
      cvo::CvoFrame::Ptr cvo_ptr (new cvo::CvoFrame(&cvo_pcs[i], kf_to_world.data()));
      cvo_frames.push_back(cvo_ptr);

      const_flags_in_BA[i] = (i <= temporalStartIndex);
    }
    //if (temporalStartIndex == 0)



    // read edges to construct graph
    // TODO: edges will be constructed from the covisibility graph later
    std::list<std::pair<cvo::CvoFrame::Ptr, cvo::CvoFrame::Ptr>> edges;
    std::list<std::pair<int, int>> edges_inds;    

    for (int i = temporalStartIndex; i < cvo_frames.size(); i++) {
      for (int j = i+1; j < std::min(i+4, (int)cvo_frames.size()); j++) {
        std::pair<cvo::CvoFrame::Ptr, cvo::CvoFrame::Ptr> p(cvo_frames[i], cvo_frames[j]);
        edges.push_back(p);
        edges_inds.push_back(std::make_pair(activeKeyframes[i]->frameID(),
                                            activeKeyframes[j]->frameID()));
      }
    }
    //for (int i = 0; i < temporalStartIndex; i++) {
    for (auto & edgePair : edgesCovisibleToTemporal) {
      CovisibilityNode * covisNode = edgePair.first;
      int covisibleID = covisNode->node->activeID();
      CovisibilityNode * tempNode = edgePair.second;
      int temporalID = tempNode->node->activeID();
      std::pair<cvo::CvoFrame::Ptr, cvo::CvoFrame::Ptr> p(cvo_frames[covisibleID], cvo_frames[temporalID]);
      edges.push_back(p);
      edges_inds.push_back(std::make_pair(covisNode->node->frameID(), tempNode->node->frameID()));            
    }

    std::cout<<" and among them are "<<edges.size()<<" edges, including\n";
    for (auto & p : edges_inds )
      std::cout<<"(" << p.first<<","<<p.second<<"), ";
    std::cout<<std::endl;

    static int irls_counter = 0;
    dumpFramesToPcd (std::to_string(irls_counter)+"_graph.txt",
                     activeKeyframes,
                     cvo_frames,
                     edges_inds);
    irls_counter++;

    
    
    double time = 0;    


    cvo_align->align(cvo_frames, const_flags_in_BA,  edges, &time);
    std::cout<<"cvo BA time is "<<time<<", for "<<cvo_frames.size()<<" frames\n";

    for (int i = 0; i < activeKeyframes.size()-1; i++) {
      auto kf = activeKeyframes[i];
      Eigen::Matrix<double, 4,4, Eigen::RowMajor> kf_to_world = Eigen::Matrix<double, 4,4, Eigen::RowMajor>::Identity();
      kf_to_world.block<3,4>(0,0) = Eigen::Map<Eigen::Matrix<double, 3,4, Eigen::RowMajor>>(cvo_frames[i]->pose_vec);
      Eigen::Matrix4f kf_to_world_f = kf_to_world.cast<float>();
      Sophus::SE3f pose_BA(kf_to_world_f);
      kf->setCamToWorld(pose_BA);
    }
    std::cout<<"just written CVO BA results to all frames\n";

    

    Frame::Ptr lastKf = activeKeyframes.back();
    Frame::Ptr secondLastKf = activeKeyframes[activeKeyframes.size()-2];
    Sophus::SE3f lastKfToWorld = secondLastKf->camToWorld() * lastKf->thisToParentPose();
    lastKf->setCamToWorld(lastKfToWorld);
  }

  void FullSystem::createKeyframeAndOptimize(const std::shared_ptr<Frame>& frame)
  {

    const auto& settings = Settings::getInstance();

    Utils::Time t1 = std::chrono::steady_clock::now();

    // initialize candidates
    this->createCandidates(frame);    
    // this->trackCandidatesCvo(frame, true);
    this->trackCandidatesCvo(frame);
    
    // insert new keyframe
    this->lmcw->insertNewKeyframe(frame);

    if (settings.debugPrintLog && settings.debugLogKeyframes)
    {
      const std::string msg = "Keyframe: " + std::to_string(frame->keyframeID()) + "\t number of candidates in new keyframe is " + std::to_string(frame->candidates().size());

      auto& log = Log::getInstance();
      log.addCurrentLog(frame->frameID(), msg);

    }

    // select window
    Utils::Time t_window_init = std::chrono::steady_clock::now();    
    // this->lmcw->selectWindow(this->ceresOptimizer);  //TL: need to split to two functions
    this->lmcw->selectTemporalWindowCvo();
    Utils::Time t_window_end = std::chrono::steady_clock::now();
    windowConstructionTime.push_back(Utils::elapsedTime(t_window_init, t_window_end));

    // refine candidates from the temporal window
    //Utils::Time t_refine_init = std::chrono::steady_clock::now();
    //this->refineCandidates();
    //Utils::Time t_refine_end = std::chrono::steady_clock::now();
    //std::cout << "Refine Candidates: " << Utils::elapsedTime(t_refine_init, t_refine_end) << std::endl;

    // select new active points from the temporal window
    this->lmcw->activatePointsCvo();

    std::list<std::pair<CovisibilityNode*, CovisibilityNode*>> edgesCovisibleToTemporal;
    if (!settings.doOnlyTemporalOpt)
      edgesCovisibleToTemporal = this->lmcw->selectCovisibleWindowCvo();
    //this->lmcw->selectCovisibleWindowCvo2();
    //if (lmcw->allKeyframes().size() > 1 && lmcw->allKeyframes()[lmcw->allKeyframes().size()-2]->activePoints().size())
    //  this->lmcw->allKeyframes()[lmcw->allKeyframes().size()-2]->dump_active_points_to_pcd("active_points.pcd");    

    // optimize
    const auto& activeKeyframes = this->lmcw->activeWindow();


    // cvo BA
    if (cvo_align) {
      Utils::Time t_cvoba_init =  std::chrono::steady_clock::now();
      this->cvoMultiAlign(activeKeyframes, edgesCovisibleToTemporal); // TL: commented out to speed up
      Utils::Time t_cvoba_end =  std::chrono::steady_clock::now();
      cvoBATime.push_back(Utils::elapsedTime(t_cvoba_init, t_cvoba_end));    
      // this->lmcw->updateVoxelMapCovisGraph();
    }



    // remove outliers
    // TODO: only keep the active points that have neighbors
    //this->lmcw->removeOutliers();

    // TODO: mapping for all keyframes' active points' positions/color/semantics
    //Utils::Time t_ba_init =  std::chrono::steady_clock::now();
    //this->ceresOptimizer->solve(activeKeyframes);
    //Utils::Time t_ba_end =  std::chrono::steady_clock::now();
    //localBATime.push_back(Utils::elapsedTime(t_ba_init, t_ba_end));    

    
    // set new tracking reference
    // use covisibility keyframes here too
    Utils::Time t13 = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(this->trackingReferenceMutex);
      //this->newTrackingReference->setNewReference(activeKeyframes);
      this->newTrackingReference = frame;
      this->trackingReferenceUpdated = true;

      /*
      if ((this->outputWrapper && settings.showDepthMap) ||
          settings.saveDepthMaps)
      {
        // depth map
        this->drawTrackDepthMap(this->newTrackingReference, this->depthMapImage, settings.showDepthMapLvl);
      }

      if (settings.saveDepthMaps)
      {
        // check directory
        dsm::Utils::makeDir(settings.depthMapsDir);

        std::string file = settings.depthMapsDir + "/depth" + std::to_string(this->saveID) + ".png";
        cv::imwrite(file, this->depthMapImage);
        this->saveID++;
      }

      if (this->outputWrapper && settings.showDepthMap)
      {
        this->outputWrapper->publishProcessFrame(this->depthMapImage);
      }
      */
    }
    Utils::Time t14 = std::chrono::steady_clock::now();
    //std::cout << "Tracking ref time: " << Utils::elapsedTime(t13, t14) << std::endl;

    // Debug windows
    if (settings.debugShowOptKeyframes && this->outputWrapper)
      this->drawActiveKeyframes();

    //if (settings.debugShowOptError && this->outputWrapper)
    //  this->drawOptErrorMap();

    //if (settings.debugShowOptWeight && this->outputWrapper)
    //  this->drawOptWeight();

    //if (settings.debugShowOptLight && this->outputWrapper)
    //  this->drawOptLight();

    //if (settings.debugShowOptErrorDist && this->outputWrapper)
    //  this->drawOptErrorDist();

    // Publish to output wrapper
    Utils::Time t15 = std::chrono::steady_clock::now();
    if (this->outputWrapper)
    {
      // reset types
      this->outputWrapper->resetKeyframeTypes();

      // 3d points & cameras
      const auto temporalKeyframes = this->lmcw->temporalWindow();
      const auto covisibleKeyfames = this->lmcw->covisibleWindow();

      for (const auto& kf : temporalKeyframes)
      {
        this->outputWrapper->publishKeyframe(kf, KeyframeType::TEMPORAL);
      }
      for (const auto& kf : covisibleKeyfames)
      {
        this->outputWrapper->publishKeyframe(kf, KeyframeType::COVISIBILITY);
      }
    }

    std::cout<<"Just finish optimization, keyframe to world pose results are: \n";
    auto & active_windows = this->lmcw->activeWindow();
    for (int j = 0 ; j < this->lmcw->getActiveWindowSize(); j++ ) {
      if (j < this->lmcw->getTemporalWindowSize())
        std::cout<<"Covisible KF id ";
      else
        std::cout<<"Temporal KF id ";
      std::cout<<active_windows[j]->frameID()<<" with pose \n"<<active_windows[j]->camToWorld().matrix()<<std::endl;
    }
    std::cout<<"\n";
    
    Utils::Time t16 = std::chrono::steady_clock::now();
    //std::cout << "Visualization time: " << Utils::elapsedTime(t15, t16) << std::endl;

    // update covisibility graph
    //tils::Time t17 = std::chrono::steady_clock::now();
    //this->lmcw->updateConnectivity();
    //Utils::Time t18 = std::chrono::steady_clock::now();
    //std::cout << "Update connectivity time: " << Utils::elapsedTime(t9, t10) << std::endl;

    // drop keyframes and remove covisible keyframes
    // we will only estimate new candidates from the temporal window
    this->lmcw->dropFlaggedKeyframes();

    // create new candidates for last keyframe
    //Utils::Time t19 = std::chrono::steady_clock::now();
    //this->createCandidates(frame);
    //Utils::Time t20 = std::chrono::steady_clock::now();
    //std::cout << "Create candidates time: " << Utils::elapsedTime(t19, t20) << std::endl;

    Utils::Time t2 = std::chrono::steady_clock::now();
    std::cout << "createKeyframeAndOptimize in " << Utils::elapsedTime(t1, t2) << std::endl;
  }

  void FullSystem::drawTrackDepthMap(const std::shared_ptr<FrameTrackerReference>& trackRef, cv::Mat& depthMapBGR, int lvl)
  {
    if (trackRef == nullptr) return;

    // constant values
    const float* pyrImg = trackRef->reference()->image(lvl);
    const float* iDepthMap = trackRef->iDepthMap(lvl);

    const int w = GlobalCalibration::getInstance().width(lvl);
    const int h = GlobalCalibration::getInstance().height(lvl);

    if (pyrImg == nullptr || iDepthMap == nullptr)
    {
      return;
    }

    // obtain inverse depth range
    std::vector<float> allIDepths;
    for (int i = 0; i < w*h; i++)
    {
      if (!(iDepthMap[i] <= 0.f))
      {
        allIDepths.push_back(iDepthMap[i]);
      }		
    }
    std::sort(allIDepths.begin(), allIDepths.end());

    if (allIDepths.empty()) return;

    this->minIDepthTracker = std::max(allIDepths[(int)(allIDepths.size()*0.1f)], 0.f);
    this->maxIDepthTracker = std::max(allIDepths[(int)(allIDepths.size()*0.9f)], 0.f);

    // copy image
    for (int row = 0; row < h; ++row)
    {
      for (int col = 0; col < w; ++col)
      {
        int index = col + row * w;
        unsigned char c = (unsigned char)pyrImg[index];
        depthMapBGR.at<cv::Vec3b>(row, col) = cv::Vec3b(c, c, c);
      }
    }

    // draw points
    for (int row = 0; row < h; ++row)
    {
      for (int col = 0; col < w; ++col)
      {
        int index = col + row * w;
        if (!(iDepthMap[index] <= 0.f))
        {
          Eigen::Vector3f rgb = Utils::colorMap(this->minIDepthTracker, this->maxIDepthTracker, iDepthMap[index]) * 255;
          cv::Vec3b bgr = cv::Vec3b((unsigned char)rgb[2], (unsigned char)rgb[1], (unsigned char)rgb[0]);

          Utils::draw3x3Square(depthMapBGR, row, col, bgr);
        }
      }
    }
  }

  void FullSystem::drawActiveKeyframes()
  {
    const auto& activeKeyframes = this->lmcw->activeWindow();
    int numActiveKeyframes = (int)activeKeyframes.size();

    if (!(numActiveKeyframes > 1)) return;

    const auto& calib = GlobalCalibration::getInstance();
    const Eigen::Matrix3f& K = calib.matrix3f(0);
    const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
    const int w = (int)calib.width(0);
    const int h = (int)calib.height(0);

    const std::shared_ptr<Frame>& lastKeyframe = activeKeyframes.back();
    const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();
    int lastIdx = lastKeyframe->keyframeID();

    // obtain inverse depth range
    std::vector<float> allIDepths;
    for (int i = 0; i < numActiveKeyframes - 1; ++i)
    {
      const auto& activePoints = activeKeyframes[i]->activePoints();
      for (const auto& point : activePoints)
      {
        allIDepths.push_back(point->iDepth());
      }
    }
    std::sort(allIDepths.begin(), allIDepths.end());

    if (allIDepths.empty()) {
      std::cout<<"Drawactivekeyframes: Idepth emtpy\n";
      return; 
    }

    this->minIDepthOpt = std::max(allIDepths[(int)(allIDepths.size()*0.1f)], 0.f);
    this->maxIDepthOpt = std::max(allIDepths[(int)(allIDepths.size()*0.9f)], 0.f);

    // create images
    std::vector<cv::Mat> images(numActiveKeyframes);
    for (int i = 0; i < numActiveKeyframes; ++i)
    {
      // create image
      images[i] = cv::Mat(h, w, CV_8UC1);

      // copy values
      const float* imagePtr = activeKeyframes[i]->image(0);
      for (int idx = 0; idx < w*h; idx++)
      {
        images[i].data[idx] = (unsigned char)imagePtr[idx];
      }

      // change from gray to bgr
      cv::cvtColor(images[i], images[i], cv::COLOR_GRAY2BGR);

      std::string keyframeTypeStr;
      if (i < this->lmcw->getTemporalWindowSize())
        keyframeTypeStr = "Covisible Window: frame id ";
      else
        keyframeTypeStr = "Temporal Widnow: frame id ";

      int frameId = this->lmcw->activeWindow()[i]->frameID();
      std::string imageLabel = keyframeTypeStr + std::to_string(frameId);
      
      cv::putText(images[i], //target image
                  imageLabel, //text
                  cv::Point(10, images[i].rows - 10), //top-left position
                  cv::FONT_HERSHEY_DUPLEX,
                  1.0,
                  CV_RGB(118, 185, 0), //font color
                  2);
    }

    const int rectSize = 6;
    const int rectPad = rectSize / 2;

    // draw for each active keyframe their active points
    // draw also the active points in the last keyframe
    for (int i = 0; i < numActiveKeyframes - 1; ++i)
    {
      // relative pose data
      const Sophus::SE3f relPose = worldToLast * activeKeyframes[i]->camToWorld();
      const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
      const Eigen::Vector3f Kt = K * relPose.translation();

      // active points
      // use all active points
      const auto& activePoints = activeKeyframes[i]->activePoints();
      for (const auto& point : activePoints)
      {
        const float u = point->u(0);
        const float v = point->v(0);

        if (point->visibility(lastIdx) == Visibility::VISIBLE)
        {
          Eigen::Vector2f pt2d;
          if (!Utils::project(u, v, point->iDepth(),
                              w, h, KRKinv, Kt, pt2d))
          {
            continue;
          }
					
          Eigen::Vector3f rgb = Utils::colorMap(this->minIDepthOpt, this->maxIDepthOpt, point->iDepth()) * 255;
					
          // draw in the reference image
          cv::rectangle(images[i], cv::Rect((int)u - rectPad, (int)v - rectPad, rectSize, rectSize), cv::Scalar(rgb[2], rgb[1], rgb[0]), 2);
					
          // draw in last keyframe
          cv::rectangle(images[numActiveKeyframes - 1], cv::Rect((int)pt2d[0] - rectPad, (int)pt2d[1] - rectPad, rectSize, rectSize), cv::Scalar(rgb[2], rgb[1], rgb[0]), 2);
        }
        else
        {
          cv::rectangle(images[i], cv::Rect((int)u - rectPad, (int)v - rectPad, rectSize, rectSize), cv::Scalar(0, 0, 0), 2);
        }		
      }
    }

    // show images
    cv::Mat output;
    Utils::concatImages(images, output);

    if (this->outputWrapper)
    {
      this->outputWrapper->publishOptKeyframes(output);
    }
  }

  void FullSystem::drawOptErrorMap()
  {
    const auto& settings = Settings::getInstance();

    const auto& calib = GlobalCalibration::getInstance();
    const Eigen::Matrix3f& K = calib.matrix3f(0);
    const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
    const int w = calib.width(0);
    const int h = calib.height(0);

    const auto& activeKeyframes = this->lmcw->activeWindow();
    const auto& lastKeyframe = activeKeyframes.back();
    const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();
    int idx = lastKeyframe->keyframeID();

    // initialize an opencv image
    cv::Mat errorMap(h, w, CV_8UC1);

    // copy values
    const float* imagePtr = lastKeyframe->image(0);
    for (int idx = 0; idx < w*h; idx++)
    {
      errorMap.data[idx] = (unsigned char)imagePtr[idx];
    }

    // change from gray to bgr
    cv::cvtColor(errorMap, errorMap, cv::COLOR_GRAY2BGR);

    for (auto& kf : activeKeyframes)
    {
      // relative pose data
      const Sophus::SE3f relPose = worldToLast * kf->camToWorld();
      const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
      const Eigen::Vector3f Kt = K * relPose.translation();

      for (auto& point : kf->activePoints())
      {
        if (point->visibility(idx) == Visibility::VISIBLE)
        {
          auto it = point->observations().find(lastKeyframe.get());

          if (it == point->observations().end()) continue;

          Eigen::Vector2f pt2d;
          if (!Utils::project(point->u(0), point->v(0), point->iDepth(),
                              w, h, KRKinv, Kt, pt2d))
          {
            continue;
          }

          double error = it->second->energy();

          Eigen::Vector3f rgb = Utils::colorMap(0.f, 15.f*15.f*Pattern::size(), (float)error) * 255;

          // draw in last keyframe
          cv::circle(errorMap, cv::Point((int)(pt2d[0] + 0.5f), (int)(pt2d[1] + 0.5f)), 2, cv::Scalar(rgb[2], rgb[1], rgb[0]), -1);
        }
      }
    }

    if (this->outputWrapper)
    {
      this->outputWrapper->publishOptError(errorMap);
    }
  }

  void FullSystem::drawOptWeight()
  {
    const auto& calib = GlobalCalibration::getInstance();
    const Eigen::Matrix3f& K = calib.matrix3f(0);
    const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
    const int w = calib.width(0);
    const int h = calib.height(0);

    const auto& activeKeyframes = this->lmcw->activeWindow();
    const auto& lastKeyframe = activeKeyframes.back();
    const Sophus::SE3f worldToLast = lastKeyframe->camToWorld().inverse();
    int idx = lastKeyframe->keyframeID();

    // initialize an opencv image
    cv::Mat weightMap(h, w, CV_8UC1);

    // copy values
    const float* imagePtr = lastKeyframe->image(0);
    for (int idx = 0; idx < w*h; idx++)
    {
      weightMap.data[idx] = (unsigned char)imagePtr[idx];
    }

    // change from gray to bgr
    cv::cvtColor(weightMap, weightMap, cv::COLOR_GRAY2BGR);

    for (auto& kf : activeKeyframes)
    {
      // relative pose data
      const Sophus::SE3f relPose = worldToLast * kf->camToWorld();
      const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
      const Eigen::Vector3f Kt = K * relPose.translation();

      for (auto& point : kf->activePoints())
      {
        if (point->visibility(idx) == Visibility::VISIBLE)
        {
          auto it = point->observations().find(lastKeyframe.get());

          if (it == point->observations().end()) continue;

          Eigen::Vector2f pt2d;
          if (!Utils::project(point->u(0), point->v(0), point->iDepth(),
                              w, h, KRKinv, Kt, pt2d))
          {
            continue;
          }

          double weight = it->second->lossWeight();

          Eigen::Vector3f rgb = Utils::colorMap(0.f, 1.f, (float)weight) * 255;

          // draw in last keyframe
          cv::circle(weightMap, cv::Point((int)(pt2d[0] + 0.5f), (int)(pt2d[1] + 0.5f)), 2, cv::Scalar(rgb[2], rgb[1], rgb[0]), -1);
        }
      }
    }

    if (this->outputWrapper)
    {
      this->outputWrapper->publishOptWeight(weightMap);
    }
  }

  void FullSystem::drawOptLight()
  {
    const auto& activeKeyframes = this->lmcw->activeWindow();
    int numActiveKeyframes = (int)activeKeyframes.size();

    if (!(numActiveKeyframes > 1)) return;

    const auto& calib = GlobalCalibration::getInstance();
    const Eigen::Matrix3f& K = calib.matrix3f(0);
    const Eigen::Matrix3f& Kinv = calib.invMatrix3f(0);
    const int w = (int)calib.width(0);
    const int h = (int)calib.height(0);

    // create images
    std::vector<cv::Mat> images(numActiveKeyframes);
    for (int i = 0; i < numActiveKeyframes; ++i)
    {
      // create image
      images[i] = cv::Mat(255, 255, CV_8UC3);
      images[i].setTo(cv::Scalar(255, 255, 255));
    }

    for (int i = 0; i < numActiveKeyframes; ++i)
    {
      const auto& kf1 = activeKeyframes[i];
      const Sophus::SE3f worldToKf1 = kf1->camToWorld().inverse();

      const AffineLight& light = kf1->affineLight();
      const float* image = kf1->image(0);

      // draw points
      for (const auto& kf2 : activeKeyframes)
      {
        if(kf1 == kf2) continue;

        // relative data
        const Sophus::SE3f relPose = worldToKf1 * kf2->camToWorld();
        const Eigen::Matrix3f KRKinv = K * relPose.rotationMatrix() * Kinv;
        const Eigen::Vector3f Kt = K * relPose.translation();

        const AffineLight& affLight = kf2->affineLight();
        const float affLight_a = affLight.a();
        const float affLight_b = affLight.b();

        for (const auto& point : kf2->activePoints())
        {
          if (point->visibility(kf1->keyframeID()) != Visibility::VISIBLE)
          {
            continue;
          }

          const Eigen::VecXf& color = point->colors(0);

          for (int idx = 0; idx < Pattern::size(); ++idx)
          {
            float uj = point->u(0) + (float)Pattern::at(idx, 0);
            float vj = point->v(0) + (float)Pattern::at(idx, 1);

            // project point to new image
            Eigen::Vector2f pt2d;
            if (!Utils::project(uj, vj, point->iDepth(), w, h, KRKinv, Kt, pt2d))
            {
              continue;
            }
						
            float newImgColor = bilinearInterpolation(image, pt2d[0], pt2d[1], w);
						
            // image coordiantes
            int row = static_cast<int>(affLight_a*color[idx] + affLight_b + 0.5f);
            int col = static_cast<int>(newImgColor + 0.5f);
						
            col = std::min(254, col);
            col = std::max(0, col);
						
            row = std::min(254, row);
            row = std::max(0, row);
						
            // draw
            images[i].at<cv::Vec3b>(254 - row, col) = cv::Vec3b(0, 0, 0);
          }
        }
      }

      // draw affine light line
      const float light_a = light.a();
      const float light_b = light.b();

      cv::Point p1, p2;

      int pt1x = (int)(-light_b / light_a + 0.5f);
      if (pt1x >= 0)
      {
        // cross with y = 0
        pt1x = std::min(254, pt1x);
        p1 = cv::Point(pt1x, 0);
      }
      else
      {
        // cross with x = 0
        int pt1y = (int)(light_b + 0.5f);
        pt1y = std::min(254, pt1y);
        p1 = cv::Point(0, pt1y);
      }

      int pt2x = (int)((254.f - light_b) / light_a + 0.5f);
      if (pt2x <= 254)
      {
        // cross with y = 254r
        pt2x = std::max(0, pt2x);
        p2 = cv::Point(pt2x, 254);
      }
      else
      {
        // cross with x = 254
        int pt2y = (int)(light_a*254.f + light_b + 0.5f);
        pt2y = std::max(0, pt2y);
        p2 = cv::Point(254, pt2y);
      }

      cv::line(images[i], cv::Point(p1.x, 254 - p1.y), cv::Point(p2.x, 254 - p2.y), cv::Scalar(0, 0, 255), 1);
    }

    // show images
    cv::Mat output;
    Utils::concatImages(images, output);

    if (this->outputWrapper)
    {
      this->outputWrapper->publishOptLight(output);
    }
  }

  void FullSystem::drawOptErrorDist()
  {
    const auto& activeKeyframes = this->lmcw->activeWindow();
    int numActiveKeyframes = (int)activeKeyframes.size();

    const auto& calib = GlobalCalibration::getInstance();
    const int w = (int)calib.width(0);
    const int h = (int)calib.height(0);

    // initialize vector
    std::vector<cv::Mat> images(numActiveKeyframes);

    for (int i = 0; i < numActiveKeyframes; ++i)
    {
      const auto& kf1 = activeKeyframes[i];

      std::vector<float> allObservations;

      // take all observations
      for (const auto& kf2 : activeKeyframes)
      {
        if (kf1 == kf2) continue;

        for (const auto& point : kf2->activePoints())
        {
          if (point->visibility(kf1->keyframeID()) == Visibility::UNINITIALIZED ||
              point->visibility(kf1->keyframeID()) == Visibility::OOB)
          {
            continue;
          }

          const auto& obs = point->observations();

          // find kf1
          auto it = obs.find(kf1.get());
          if (it == obs.end()) continue;

          // take residual vector
          const auto& pixelEnergy = it->second->pixelEnergy();
          for (int idx = 0; idx < Pattern::size(); ++idx)
          {
            allObservations.push_back((float)pixelEnergy[idx]);
          }
        }
      }

      // draw distribution
      const auto& distribution = kf1->errorDistribution();
      images[i] = Utils::drawDistribution(allObservations, distribution, 
                                          Eigen::Vector2f(-255.f, 255.f),
                                          Eigen::Vector2f(0, 0.1f),
                                          Eigen::Vector2i(510, 510),
                                          510);
    }

    // show images
    cv::Mat output;
    Utils::concatImages(images, output);

    if (this->outputWrapper)
    {
      this->outputWrapper->publishOptErrorDist(output);
    }
  }

  void FullSystem::printLog() const
  {
    const auto& settings = Settings::getInstance();
    if (settings.debugPrintLog)
    {
      const auto& log = Log::getInstance();
      log.printLog();
    }
  }

  // classes for parallelization
  FullSystem::PointObserver::PointObserver(std::vector<CandidatePoint*>& theCandidates, const std::shared_ptr<Frame> &targetFrame,
                                           int aBegin, int anEnd) :
    candidates(theCandidates), target(targetFrame), begin(aBegin), end(anEnd)
  {
  }

  void FullSystem::PointObserver::operator()()
  {
    // compute candidate observation from begin to end only
    for (int i = this->begin; i < this->end; ++i)
    {
      CandidatePoint* candPoint = this->candidates[i];

      // observe in the new tracked frame
      candPoint->observe(this->target);
    }
  }

  FullSystem::PointOptimizer::PointOptimizer(std::vector<CandidatePoint*>& theCandidates,
                                             const std::vector<std::shared_ptr<Frame>> &activeFrames,
                                             int aBegin, int anEnd) :
    candidates(theCandidates), activeFrames(activeFrames), begin(aBegin), end(anEnd)
  {
  }

  void FullSystem::PointOptimizer::operator()()
  {
    // compute candidate observation from begin to end only
    for (int i = this->begin; i < this->end; ++i)
    {
      CandidatePoint* candPoint = this->candidates[i];

      // optimize inverse depth in all active keyframes
      candPoint->optimize(this->activeFrames);
    }
  }
}

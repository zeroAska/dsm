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

#pragma once

#include "DSMLib.h"

#include <Eigen/Core>
#include <Eigen/StdVector>
#include "LMCW.h"
#include "sophus/se3.hpp"

#include "opencv2/core.hpp"
#include "DataStructures/Frame.h"

#include <deque>
#include <memory>
#include <vector>
#include <mutex>
#include <shared_mutex>
#include <unordered_map>
#include <atomic>
#include <map>
#include <thread>
#include <pcl/point_cloud.h>
#include "utils/CvoPoint.hpp"

namespace cvo {
  class CvoGPU;
  class Calibration;
  class CvoFrame;
  class CvoPointCloud;
  class Association;
  } // namespace cvo
namespace semantic_bki {
  class SemanticBKIOctoMap;
}  

namespace dsm
{
  class Frame;
  class CandidatePoint;
  class ActivePoint;
  class PointDetector;
  class CeresPhotometricBA;
  class MonoInitializer;
  class FrameTracker;
  class FrameTrackerReference;
  class DistanceTransform;
  class CovisibilityGraph;
  class IVisualizer;
  class WorkerThreadPool;
  class LMCW;

  class DSM_EXPORTS_DLL FullSystem
    {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FullSystem(int w, int h, const Eigen::Matrix3f &calib,
               const std::string &settingsFile,
               IVisualizer *outputWrapper = nullptr);    
    FullSystem(int w, int h, const cvo::Calibration &calib,
               const std::string &cvoParamsFile,
               const std::string &settingsFile,
               IVisualizer *outputWrapper = nullptr);

    FullSystem(const FullSystem&) = delete;
    FullSystem& operator=(const FullSystem&) = delete;
    ~FullSystem();

    ////////////////////////////////////////
    //		  MAIN TRACKING FUNC		  //
    ////////////////////////////////////////


    // Tracks the stereo frame. It calculates the system pose and the 3D scene*/
    //void trackFrame(int id, double timestamp, unsigned char* image);
    //void trackFrame(int id, double timestamp, unsigned char* gray_img, std::shared_ptr<cvo::RawImage> left_img, const cv::Mat & right_img,  pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd);
    //void trackFrame(int id, double timestamp, unsigned char* image, std::shared_ptr<cvo::RawImage> left_img, const std::vector<uint16_t> & depth_img, float depth_scale,  pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd);
    void trackFrame(int id, double timestamp, Frame::Ptr frame);
    void trackModelToFrameCvo(std::shared_ptr<Frame>& frame );
    void trackFrameToFrameCvo( std::shared_ptr<Frame>& frame);
    ////////////////////////////////////////
    //		       ACCESORS	 	          //
    ////////////////////////////////////////

    void getTrajectory(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> &poses, std::vector<double> &timestamps,
                       std::vector<int> & ids) const;

    void getFullTrajectory(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > &poses, std::vector<double> &timestamps,
                           std::vector<int> &ids) const;

    void getStructure(std::vector<Eigen::Vector3f>& structure) const;

    float getCamTrackingMeanTime() const;
    float getPointTrackingMeanTime() const;
    float getLocalBAMeanTime() const;
    float getTotalBackendMeanTime() const;
    float getWindowConstructionMeanTime() const;
    float getCvoBAMeanTime() const;
    float getMeanTime(const std::vector<float> & timeVec) const;

    int getNumPoints() const;
    int getNumKeyframes() const;

    bool getLastWasKF() const;

    bool isInitialized() const;

    bool isLost() const;

    void printLog() const;

    private:

    // initialize from video sequence
    bool initialize_lmcw(const std::shared_ptr<Frame>& frame);

    // track new frame using image alignment
    void trackNewFrame(const std::shared_ptr<Frame>& frame);

    // keyframe management
    bool isNewKeyframeRequired(const std::shared_ptr<Frame>& frame) const;

    // mapping thread infinite loop
    void mappingThreadLoop();
    void doMapping(const std::shared_ptr<Frame>& frame);
    void waitUntilMappingFinished();

    // candidates management
    void createCandidatesPlanar(const std::shared_ptr<Frame>& frame);
    void createCandidatesVoxel(const std::shared_ptr<Frame>& frame);
    void trackCandidates(const std::shared_ptr<Frame>& frame);
    void trackCandidatesCvo(const std::shared_ptr<Frame> frame, bool include_curr=false);
    void refineCandidates();

    // Optimization
    void createKeyframeAndOptimize(const std::shared_ptr<Frame>& frame);
    void covisMapAlign(//std::vector<CvoFrame::Ptr> temporal_frames,
                       const cvo::CvoPointCloud & tmpWindow,
                       const cvo::CvoPointCloud & covisWindow,
                       const std::vector<std::shared_ptr<Frame>> & activeKeyframes
                       );
    void cvoMultiAlign(const std::vector<std::shared_ptr<Frame>> & activeKeyframes,
                       const std::list<std::pair<CovisibilityNode *, CovisibilityNode*>> & edgesCovisibleToTemporal);
    void cvoMultiAlign(
                       const std::vector<std::shared_ptr<Frame>> & activeKeyframes,
                       const cvo::CvoPointCloud & covisMapCvo
                       );
    void obtain_inliers(const std::vector<std::shared_ptr<Frame>> & activeKeyframes,
                        const std::vector<std::shared_ptr<cvo::CvoFrame>> & cvo_frames);
    void updateActivePointsInliers(const std::list<std::pair<Frame::Ptr, Frame::Ptr>> & edge_frame_ids,
                                   const std::list<std::shared_ptr<cvo::Association>> & associations);    
    
    void dumpFramesToPcd (const std::string & graphDefFileName,
                          const std::vector<std::shared_ptr<Frame>> & activeKeyframes,
                          const std::vector<std::shared_ptr<cvo::CvoFrame>> & cvo_frames,
                          const std::list<std::pair<int, int>> & edges) const;
    void dumpFramesToPcd (const std::string & graphDefFileName,
                          const std::vector<std::shared_ptr<Frame>> & activeKeyframes,
                          const std::vector<std::shared_ptr<cvo::CvoFrame>> & cvo_frames,
                          std::list<std::pair<Frame::Ptr, Frame::Ptr>> edges_frame_ptrs) const;
    

    // visualization
    void drawTrackDepthMap(const std::shared_ptr<FrameTrackerReference>& trackRef, cv::Mat& depthMapBGR, int lvl);

    // optimization visualization
    void drawActiveKeyframes();	// draws active keyframes with their points

    void drawOptErrorMap();	
    void drawOptWeight();
    void drawOptLight();
    void drawOptErrorDist();

    private:

    // control states
    std::atomic_bool initialized; // check first frame to intialize 


    // threads
    std::unique_ptr<std::thread> mappingThread;
    std::atomic_bool shouldStop;

    // for blocking operation. Set in Mapping, read in Tracking.
    std::condition_variable  newFrameMappedSignal;
    std::mutex newFrameMappedMutex;
    bool newFrameMappedDone;

    // initializer
    std::unique_ptr<MonoInitializer> initializer;

    // detector
    std::unique_ptr<PointDetector> pointDetector;
    int32_t* pixelMask;

    // tracker, only in tracking thread
    std::atomic_bool trackingIsGood;    
    std::unique_ptr<FrameTracker> tracker; // normal tracker
    std::unique_ptr<cvo::CvoGPU> cvo_align; // cvo tracker
    float lastTrackingCos;

    // parallelization
    std::shared_ptr<WorkerThreadPool> threadPool;


    
    /********** tracking states ***************/
    // only in tracking thread
    // last tracked frame information for priors
    std::shared_ptr<Frame> lastTrackedFrame;		// last tracked frame
    Sophus::SE3f lastTrackedMotion;					// last tracked frame motion based on constant velocity model
    //float lastTrackedResidual;
    // tracking reference
    std::mutex trackingReferenceMutex;
    //std::shared_ptr<FrameTrackerReference> trackingReference;			// tracking reference
    //std::shared_ptr<FrameTrackerReference> newTrackingReference;		// updated tracking reference
    int numTrackedFramesFromLastKF;
    std::shared_ptr<Frame> trackingReference;
    std::shared_ptr<Frame> newTrackingReference;
    bool trackingReferenceUpdated;
    // write in tracking, read in mapping
    std::mutex unmappedTrackedFramesMutex;
    std::condition_variable  unmappedTrackedFramesSignal;
    std::deque<std::shared_ptr<Frame>> unmappedTrackedFrames;
    /*******************************************/
    

    // flag to set if new keyframe is required
    bool createNewKeyframe;							// only in tracking thread
    int createNewKeyframeID;						// write in tracking thread, read in mapping
    std::atomic_int numMappedFramesFromLastKF;		// write in mapping thread, read in tracking

    // Optimization window
    std::unique_ptr<LMCW> lmcw;


    /// semantic global map
    std::unique_ptr<semantic_bki::SemanticBKIOctoMap> map;

    // ceres optimizer
    std::unique_ptr<CeresPhotometricBA> ceresOptimizer;				// photometric bundle adjustment

    // statistics
    std::vector<float> camTrackingTime;
    std::vector<float> pointTrackingTime;
    std::vector<float> localBATime;
    std::vector<float> windowConstructionTime;
    std::vector<float> cvoBATime;
    std::vector<float> totalBackendTime;
    bool lastWasKF;

    // visualization
    IVisualizer* outputWrapper;

    cv::Mat depthMapImage;
    float minIDepthTracker;
    float maxIDepthTracker;
    float minIDepthOpt;
    float maxIDepthOpt;

    // tartan air requires
    // all frames (KFs and non-KFs)
    std::vector<std::shared_ptr<Frame>> allFrames;

    // image writing
    int saveID;
		
    // structures for parallelization
    struct PointObserver
    {
      PointObserver(std::vector<CandidatePoint*>& theCandidates, 
                    const std::shared_ptr<Frame> &targetFrame,
                    int aBegin, int anEnd);

      void operator()();

    private:
      std::vector<CandidatePoint*>& candidates;
      const std::shared_ptr<Frame>& target;
      int begin, end;
    };

    struct PointOptimizer
    {
      PointOptimizer(std::vector<CandidatePoint*>& theCandidates,
                     const std::vector<std::shared_ptr<Frame>> &activeFrames,
                     int aBegin, int anEnd);

      void operator()();

    private:
      std::vector<CandidatePoint*>& candidates;
      const std::vector<std::shared_ptr<Frame>>& activeFrames;
      int begin, end;
    };
    };
}

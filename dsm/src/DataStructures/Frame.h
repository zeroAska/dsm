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

#include <vector>
#include <memory>
#include <mutex>
#include <shared_mutex>

#include <Eigen/Core>

#include "sophus/se3.hpp"

#include "ImagePyramid.h"
#include "GradientPyramid.h"
#include "AffineLight.h"
#include "CandidatePoint.h"
#include "Statistics/IDistribution.h"
#include "FullSystem/DSMLib.h"
#include "utils/CvoPoint.hpp"
//#include "utils/CvoPointCloud.hpp"

namespace cvo {
  class ImageStereo;
  class RawImage;
  template <typename DepthT>
  class ImageRGBD;
  class CvoPointCloud;
  //  class CvoPoint;
}

namespace dsm
{
	class CandidatePoint;

	class ActivePoint;

	template<typename T>
	class Buffer;

	class FrameParameterBlock;

	struct CovisibilityNode;



	class DSM_EXPORTS_DLL Frame
	{
	public:

		EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

                enum DepthType { MONO = 0, STEREO = 1, RGBD_FLOAT = 2, RGBD_UINT16 = 3};

		// control flag
		enum Type { FRAME = 0, KEYFRAME = 1};
		enum Status { ACTIVE = 0, INACTIVE = 1 };

                //using Ptr = std::shared_ptr<Frame>;
                typedef std::shared_ptr<Frame> Ptr;

                // Mono
		Frame(int id, double timestamp, unsigned char* image);

                // stereo
                Frame(int id, double timestamp, unsigned char* image, std::shared_ptr<cvo::ImageStereo> left_img, pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd, std::shared_ptr<cvo::CvoPointCloud> full_pc);

                // rgb-d

                Frame(int id, double timestamp, unsigned char* image, std::shared_ptr<cvo::ImageRGBD<float>> left_img, pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd, float depth_scale, std::shared_ptr<cvo::CvoPointCloud> full_pc);
                Frame(int id, double timestamp, unsigned char* image, std::shared_ptr<cvo::ImageRGBD<uint16_t>> left_img, pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd, float depth_scale, std::shared_ptr<cvo::CvoPointCloud> full_pc);

		~Frame();

		// activation or desactivation
		void activate();
		void deactivate();

		bool isActive();

		// minimize internal memory
		void minimizeMemory();

		// function to evolve image to keyframe
		void evolveToKeyframe();

		// optimization functions
		void mergeOptimizationResult();

		// accessors

		// frame unique identifier
		int frameID() const;

		// keyframe unique identifier
		int keyframeID() const;
		void setKeyframeID(int id);

		// active keyframe identifier - only valid in local window
		int activeID() const;
		void setActiveID(int id);

		// frame timestamp
		double timestamp() const;

		// type
		Type type() const;

		// image pyramids
		const float* image(int32_t level) const;

		// gradient pyramids
		const float* gx(int32_t level) const;	
		const float* gy(int32_t level) const;	
		const float* gradient(int32_t level) const;		// squared norm of gradient magnitude grad = gx*gx + gy*gy

		// coarse tracking result
		// it is set only once
		// used by FRAMES
		Frame* const parent() const;
		const Sophus::SE3f& thisToParentPose() const;
		const AffineLight& thisToParentLight() const;
		void setTrackingResult(Frame* const parent, const Sophus::SE3f& thisToParentPose, 
							   const AffineLight& thisToParentAffineLight);
                void setTrackingResult(Frame* const parent, const Sophus::SE3f& thisToParentPose);

		// camera world position
		// used by KEYFRAMES
		void setCamToWorld(const Sophus::SE3f& pose);
		const Sophus::SE3f& camToWorld();

		// Sets affine light values. 
		// They fix the brightness of the frame with respect to a global reference
		void setAffineLight(const AffineLight& newAffineLight); 
		const AffineLight& affineLight();

		// flag to remove from active window
		void setFlaggedToDrop(bool flag); // only set to false in LMCW::selectTemporalWindow()
		bool flaggedToDrop() const;
                void setNonzerosLastBA(unsigned int num) {total_nonzeros_last_BA = num; }
                unsigned int getNonzerosLastBA()const {return total_nonzeros_last_BA;}

		// photometric error distribution
		void setErrorDistribution(const std::shared_ptr<IDistribution>& dist);
		const std::shared_ptr<IDistribution>& errorDistribution();

		// Dynamic energy threhold respect to the error distribution
		float energyThreshold() const;

		// access points candidates
		const std::vector<std::unique_ptr<CandidatePoint>>& candidates() const;
		std::vector<std::unique_ptr<CandidatePoint>>& candidates();
                void initCandidateQualityFlag();
                void dump_candidates_to_pcd(const std::string & fname);

		// access active points
		const std::vector<std::unique_ptr<ActivePoint>>& activePoints() const;
		std::vector<std::unique_ptr<ActivePoint>>& activePoints();
                void dump_active_points_to_pcd(const std::string & fname);                

		// get optimization parameter block
		const std::unique_ptr<FrameParameterBlock>& frameBlock() const;

                // access cvo points
                pcl::PointCloud<cvo::CvoPoint>::Ptr getTrackingPoints() {return trackingPoints_;}
                void activePointsToCvoPointCloud(cvo::CvoPointCloud & output);
                void candidatesToCvoPointCloud(cvo::CvoPointCloud & output);
                
                
                std::shared_ptr<cvo::RawImage> getRawImage() {return rawImg;}
                const std::vector <float> & getStereoDisparity();
                const std::vector<uint16_t> & getRgbdDepth();
                const std::shared_ptr<cvo::CvoPointCloud> getFullPoints() {return fullPoints_;}
                const float depthScale;
                const DepthType depthType;

		// Reference in covisibility graph
		CovisibilityNode* graphNode;
			
	private:

		// frame identification - constant from initialization
		const int frameID_;

		// acquisition time - constant from initialization
		const double timestamp_;

		// keyframe identification
		int keyframeID_;

		// active keyframe identification
		int activeID_;

		// frame type to control if it is a keyframe
		// and work with camToWorld directly
		Type type_;

		// frame status to control if is active
		Status status_;

		// image pyramids
		std::unique_ptr<ImagePyramid<float>> images_;

		// gradient pyramids - skip the border of 1 pixel
		std::unique_ptr<GradientPyramid<float>> gradients_;

		// relative parameters for frames
		// only in tracking thread
		Frame* trackingParent_;					// coarse tracking parent
		Sophus::SE3f thisToParentPose_;				// coarse tracking result
		AffineLight thisToParentLight_;			// from coarse tracking
                
                pcl::PointCloud<cvo::CvoPoint>::Ptr trackingPoints_; // for CVO coarse tracking
                std::shared_ptr<cvo::CvoPointCloud> fullPoints_;
                std::shared_ptr<cvo::RawImage> rawImg;
                //std::vector<float> stereoDisparity;
                //std::vector<uint16_t> rgbdDepth;

                

		// global parameters for keyframes
		std::mutex globalParamMutex_;
		Sophus::SE3f camToWorld_;
		AffineLight affineLight_;		

		// flag to remove from window
		bool flaggedToDrop_;
                unsigned int total_nonzeros_last_BA;

		// error distribution in this frame
		std::shared_ptr<IDistribution> errorDistribution_;

		// outlier threshold for this frame
		// this is set during photometric bundle adjustment
		// based on the median value of the observations
		float energyThreshold_;

		// candidate points
		std::vector<std::unique_ptr<CandidatePoint>> candidates_;
                // candidates to become active points
                

		// active points
		std::vector<std::unique_ptr<ActivePoint>> activePoints_;					// active points in current window

		// ceres optimization parameters
		std::unique_ptr<FrameParameterBlock> frameBlock_;
	};


  class FrameStereo : public Frame {
    
  };

  class FrameRGBD : public Frame {
    FrameRGBD (int id,
               double timestamp,
               unsigned char* image,
               std::shared_ptr<cvo::ImageRGBD<float>> color_img,
               pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd,
               float depth_scale
               );
    FrameRGBD (int id,
               double timestamp,
               unsigned char* image,
               std::shared_ptr<cvo::ImageRGBD<uint16_t>> color_img,
               pcl::PointCloud<cvo::CvoPoint>::Ptr new_frame_pcd,
               float depth_scale
               );
    
  };
}

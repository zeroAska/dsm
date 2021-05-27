#pragma once

#include "Utils/EigenTypes.h"

#include "sophus/se3.hpp"

#include "opencv2/core.hpp"

#include <memory>

namespace dsm
{
  class Frame;
  class ModelSelector;
  class IVisualizer;
  class WorkerThreadPool;

  // Class to initialize DSM from a sequence of images
  class StereoInitializer
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    StereoInitializer(const std::shared_ptr<WorkerThreadPool>& parallelizer = nullptr, 
                    IVisualizer* outWrapper = nullptr);
    ~StereoInitializer();

    // flag for reset 
    bool isResetRequired() const;
    void reset();

    // initial frame as reference
    void setReference(const std::shared_ptr<Frame>& firstFrame);
    std::shared_ptr<Frame> getReference() {return reference;};

    // sequetial frames until convergence
    bool initialize(const std::shared_ptr<Frame>& frame, Sophus::SE3f& pose);

  private:

    void trackPointsInImage(const cv::Mat& refImage, 
                            const cv::Mat& newImage, 
                            std::vector<cv::Point2f>& refPoints, 
                            std::vector<cv::Point2f>& newPoints) const;

  private:

    // flag for reset
    bool resetRequired;

    // reference data
    std::shared_ptr<Frame> reference;
    std::vector<cv::Point2f> refPoints;
    cv::Mat refImage;

    // tracked points position
    std::vector<cv::Point2f> prevPoints;

    // motion model selector
    std::unique_ptr<ModelSelector> modelSelector;

    // calibration
    cv::Mat K;

    // parameters
    int minNumTracked;
    const int maxL1Error = 7;

    // visualization
    IVisualizer* outputWrapper;
  };
}

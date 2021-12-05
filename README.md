# CVO-DSM
 
This repo uses code from DSM as the starting point and integrates CVO. Apart from [unified_cvo](https://github.com/UMich-CURLY/unified_cvo), all the dependencies are already in the [Dockerfile](https://github.com/UMich-CURLY/docker_images/tree/master/cvo_gpu).

## Compile with CVO
Assuming your UnifiedCvo repository path is `/home/user/unified_cvo/`. Then to compile DSM, you need:
```
mkdir build
cd build
cmake .. -DUnifiedCvo_DIR=/home/user/unified_cvo/build/ -DCMAKE_BUILD_TYPE=Release 
make -j
```
It will compile two libraries `libdsm.so` and `libQtVisualizer.so` at **lib** folder, which can be linked from external projects. It will also create two executables `TumExample` at **bin** folder to run DSM in the Tum dataset and with custom videos respectively.

## Demo
To run Tum RGB-D demo: `bash scripts/tum.bash`. Please change the path of the dataset folder to your own.


## DSM Parameter options
The system parameter options can be found in `settings.h`. Those can also be loaded using an external `.txt` file, such as the one in `Examples/EurocData/settings.txt`. The most relevant parameters are:

* `blockUntilMapped`: blocks the tracking thread until the mapping thread finishes.
* `singleThreaded`: runs tracking and mapping sequentially in a single thread.
* `mappingThreads`: number of multithreads to parallelize the mapping.
* `minimizeMemory`: use a buffer pool for memory usage minimization.
* `useFixedHuberScale`: flag to control if a Huber loss function with a fixed scale in used.
* `huberScale`: value of the Huber loss function scale.
* `useTDistribution`: flag to control if the t-distribution is used. When it is enabled, Huber loss is disabled.
* `nuFixed`: the t-distribution degrees of freedom. When <= 0 it is dinamically computed, otherwise it is fixed.
* `muFixed`: the t-distribution location. When < 0 it is dinamically computer, otherwise it is fixed.
* `sigmaFixed`: the t-distribution scale. When <= 0 it is dinamically computer, otherwise it is fixed.
* `defaultNu`: default value fot the t-distribution degrees of freedom.
* `defaultMu`: default value fot the t-distribution location.
* `defaultSigma`: default value fot the t-distribution scale.
* `inlierPercentile`: percentile of inliers for energy threshold.
* `maxPixelOutlier`: maximum percentage of pixels to consider an observation as outlier.
* `maxPixelDiscard`: maximum percentage of pixels to discard an observations during optimization.
* `maxEnergyFit`: maximum energy to fit the t-distribution.
* `weightConstant`: gradient weight constant.
* `trackingMaxLevel`: maximum number of pyramid levels during frame tracking.
* `pointDetectionLevels`: number of pyramid levels to detect candidate points.
* `numCandidates`: number of candidate points per keyframe.
* `numBlocksPerDimension`: number of blocks per image dimension during candidate point detection.
* `minGradAdd`: threshold constant addition to histogram for candidate point detection.
* `maxUnmappedFrames`: maximum number of unmmaped frames in the queue.
* `maxEplLengthFactor`: maximum epipolar search length.
* `minEplLengthSkip`: minimum epipolar search length.
* `stereoMaxEnergy`: maximum energy for epipolar search.
* `secondBestRadius`: radius to the second best match during epipolar search.
* `epiLineSigma`: epipolar line uncertainty.
* `subpixelIterations`: number of iterations during subpixel computation.
* `subpixelStepThreshold`: threshold to stop the subpixel optimization.
* `maxViewChange`: maximum parallax to consider a point as visible. 
* `candidateOptIterations`: maximum number of iterations for candidate refinement.
* `minDistToActivate`: initial minimum distance to activate new active points.
* `maxCandidateUncertainty`: maximum  candidate point uncertainty to activate it.
* `minCandidateQuality`: minimum candidate point quality (distance to second best) to activate it
* `doOnlyTemporalOpt`: use only temporally connected keyframe in the photometric bundle adjustment.
* `printSummary`: print bundle adjustment summary
* `showFullReport`: print bundle adjustment full report
* `minOptimizationGrad`: minimum gradient to consider an observation as outlier
* `minBAIterations`: minimum number of photometric bundle adjustment iterations.
* `maxBAIterations`: maximum number of photometric bundle adjustment iterations.
* `optMaxLevel`: number of maximum levels during pyramidal photometric bundle adjustment.
* `varScaleRot`: rotation scale factor during optimizations.
* `varScaleTrans`: translation scale factor during optimizations.
* `varScaleAlpha`: light alpha scale factor during optimizations.
* `varScaleBeta`: light beta scale factor during optimizations.
* `varScaleIDepth`: inverse depth scale factor during optimization.
* `numActivePoints`: maximum number of observations in the latest keyframe.
* `maxTemporalKeyframes`: maximum number of temporal keyframes in the optimization window.
* `maxCovisibleKeyframes`: maximum number of covisible keyframes in the optimization window.
* `numAlwaysKeepKeyframes`: number of latest temporal keyframes to keep always in the optimization window.
* `minPointCovisible`: minimum ratio of visible points to consider a keyframe as covisible.
* `maxLightCovisible`: maximum light change to consider a keyframe covisible.
* `minNumKFToConsiderNew`: minimum number of keyframes to consider an active point as a new.
* `minNumGoodObservations`: minimum number of observation to consider an active point as inlier.
* `newKFDistWeight`: weight of the camera translation relative to scene depth during new keyframe selection.
* `newKFUsageWeight`: weight of the point usage by the frame tracker during new keyframe selection.
* `newKFAffineWeight`: weight of the light change in the scene during new keyframe selection.
* `newKFResidualWeight`: weight of the frame tracker residual during new keyframe selection.
* `minNumMappedFramesToCreateKF`: minimum number of tracked frames to create a new keyframe.
* `iDepthUncertainty`: Initial uncertainty given to the depth and stereo points
* `trackingCosLimit`: If cvo's `function_angle` returns a smaller value than this, the frame would be considered as a new keyframe.	

## 5. License

DSM is released under a [GPLv3 license](https://github.com/jzubizarreta/dsm/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/jzubizarreta/dsm/blob/master/Dependencies.md).

For a closed-source version of DSM for commercial purposes, please contact the authors.

If you use DSM in an academic work, please cite:

    @article{Zubizarreta2020,
      title={Direct Sparse Mapping},
      author={Zubizarreta, Jon, Aguinaga, Iker and Montiel, J. M. M.},
      journal={IEEE Transactions on Robotics},
      doi = {10.1109/TRO.2020.2991614},
      year={2020}
     }

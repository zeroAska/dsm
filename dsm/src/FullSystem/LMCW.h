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
#include <list>
#include <unordered_map>

namespace cvo {
  class CvoGPU;
  class CvoPointCloud;
}

namespace dsm
{
  class Frame;
  class DistanceTransform;
  class CeresPhotometricBA;
  class CovisibilityGraph;
  class CovisibilityNode;
  class IVisualizer;
  template <typename PointType>
  class VoxelMap;
  class ActivePoint;
  

  // optimization window

  class LMCW
  {
  public:

    LMCW(int width, int height, IVisualizer *visualizer = nullptr);
    LMCW(int width, int height, const cvo::CvoGPU * align, IVisualizer *visualizer); 
    LMCW(int width, int height, const cvo::CvoGPU * align, IVisualizer *visualizer, float voxelSize);         
    ~LMCW();

    // reset all keyframes
    void clear();

    // inserts new keyframe to the window
    void insertNewKeyframe(const std::shared_ptr<Frame>& newKeyframe);

    // selects active keyframes
    void selectWindow(const std::unique_ptr<CeresPhotometricBA>& photometricBA);

    // removes keyframes from active window
    void dropFlaggedKeyframes();

    // activates new points from the window
    void activatePoints(const std::unique_ptr<CeresPhotometricBA>& photometricBA);
    void activatePointsCvo();

    // remove outlier points after PBA
    void removeOutliers() const;

    // updates covisibilty connections
    void updateConnectivity() const;

    // delete temporal frame points from voxel map to prepare for BA
    void deleteTemporalVoxelPoints();

    // insert temporal frame points to voxel map after BA
    void insertTemporalVoxelPoints();

    // update the covisbility graph after BA
    void updateVoxelMapCovisGraph();

    // all active keyframes
    inline const std::vector<std::shared_ptr<Frame>>& activeWindow() const { return this->activeKeyframes_; }

    // all keyframes
    inline const std::vector<std::shared_ptr<Frame>>& allKeyframes() const { return this->allKeyframes_; }

    //void active_keyframes_to_cvoframes(std::vector<>)

    inline int getTemporalWindowSize() const {return temporalWindowIndex; }
    inline int getActiveWindowSize() const {return activeKeyframes_.size(); }
          
    // temporally connected keyframes
    inline std::vector<std::shared_ptr<Frame>> temporalWindow() const 
    { 
      return std::vector<std::shared_ptr<Frame>>(this->activeKeyframes_.begin() + temporalWindowIndex, this->activeKeyframes_.end());
    }

    // covisibility based connected keyframes
    inline std::vector<std::shared_ptr<Frame>> covisibleWindow() const 
    { 
      return std::vector<std::shared_ptr<Frame>>(this->activeKeyframes_.begin(), this->activeKeyframes_.begin() + temporalWindowIndex);
    }


    // updateCovis Debug use
    inline VoxelMap<ActivePoint>* voxelMap()
    {
      return this->voxelMap_.get();
    }

    inline CovisibilityGraph* covisGraph() {
      return this->covisibilityGraph_.get();
    }


  // private:

    // selects the temporal window part
    void selectTemporalWindow(const std::unique_ptr<CeresPhotometricBA>& photometricBA);
    void selectTemporalWindowCvo();

    // selects the covisible window part
    void selectCovisibleWindow(const std::unique_ptr<CeresPhotometricBA>& photometricBA);
    void selectCovisibleMap(cvo::CvoPointCloud & covisMapOutput);
    void selectSampledCovisibleMap(cvo::CvoPointCloud & covisMapCvo);    

    // voxelMap based covisbility selection
    std::list<std::pair<CovisibilityNode *, CovisibilityNode *>>  selectCovisibleWindowCvo();
    void selectCovisibleWindowCvo2();
 
  private:
		
    // active window
    // covisible [0, temporalWindowIndex)
    // temporal [temporalWindowIndex, size())
    std::vector<std::shared_ptr<Frame>> activeKeyframes_;
    int temporalWindowIndex;

    // number of active points in latest keyframe
    int numActivePoints;

    // minimum distance to activate new points
    int minDistToActivate;

    // all keyframes
    std::vector<std::shared_ptr<Frame>> allKeyframes_;

    // distance map
    std::unique_ptr<DistanceTransform> distanceMap_;

    // covisibility graph
    std::unique_ptr<CovisibilityGraph> covisibilityGraph_;

    // voxel map
    std::unique_ptr<VoxelMap<ActivePoint>> voxelMap_;

    // visualizer
    IVisualizer* const outputWrapper_;

    const cvo::CvoGPU * cvo_align;
  };
}

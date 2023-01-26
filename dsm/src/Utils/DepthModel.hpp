#pragma once
#include "DataStructures/VoxelMap.h"
namespace dsm {
  inline
  float squared_uncertainty(float input_dist) {
    float uncertainty = input_dist * input_dist * 0.005;
    return uncertainty;
  }

  template <PointT>
  Voxel<PointT> * sample_voxel_gaussian_observations(const std::vector<const Voxel<T>*> & observations) {
    
  }
}

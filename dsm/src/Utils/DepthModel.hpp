#pragma once
#include <Eigen/Dense>
#include <limits>
#include "DataStructures/VoxelMap.h"


namespace cvo {
  class CvoGPU;
  class CvoPointCloud;
  class BinaryState;
  class CvoGPU;
  } 

namespace dsm {

  void downsample_cvo_pc(const cvo::CvoPointCloud & input,
                         float voxel_size,
                         cvo::CvoPointCloud & output);

  void filter_based_on_inner_product(const cvo::CvoPointCloud & source,
                                     const Eigen::Matrix4f & T_source,
                                     const cvo::CvoPointCloud & target,
                                     const Eigen::Matrix4f & T_target,
                                     const cvo::CvoGPU & cvo_align,
                                     cvo::CvoPointCloud & filtered_source);


  const int pixel_raycast_pattern[8][2] = {{0,0}, {-2, 0},{-1,-1}, {-1,1}, {0,2},{0,-2},{1,1},{2,0} };
  
  
  inline
  float linear_uncertainty(float input_dist) {
    //float uncertainty = input_dist * input_dist * 0.005;
    float uncertainty = input_dist * 0.1;
    return uncertainty;
  }

  template <typename PointT>
  const PointT * sample_voxel_gaussian_observations(const std::vector<const Voxel<PointT >*> & observations,
                                                    const Sophus::SE3f & camToWorld,
                                                    const Eigen::Vector3f & p_local){
                 
    if (observations.size() == 0)
      return nullptr;

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    std::vector<float> depth;
    std::vector<const PointT *> inds;
    float mean_depth = 0;
    int counter = 0;
    for (int i = 0; i < observations.size(); i++) {
      for (int j = 0; j < observations[i]->voxPoints.size(); j++) {
        
        Eigen::Vector3f p_in_W = observations[i]->voxPoints[j]->getVector3fMap();
        counter++;
        mean = (mean + p_in_W).eval();
        float projected_d = ((camToWorld.inverse() * p_in_W)(2));
        depth.push_back(projected_d);
        mean_depth += projected_d;
        inds.push_back(observations[i]->voxPoints[j]);
      }
    }
    if (counter == 0) return nullptr;
    mean = (mean / counter).eval();
    mean_depth  = mean_depth / counter;

    int min_id = std::distance(std::begin(depth), std::min_element(std::begin(depth), std::end(depth)));

    return inds[min_id];
  }

  
}

#pragma once
#include <Eigen/Dense>
#include <limits>
#include "DataStructures/VoxelMap.h"
namespace dsm {
  inline
  float squared_uncertainty(float input_dist) {
    //float uncertainty = input_dist * input_dist * 0.005;
    float uncertainty = input_dist * 0.01;
    return uncertainty;
  }

  template <typename PointT>
  const PointT * sample_voxel_gaussian_observations(const std::vector<const Voxel<PointT >*> & observations,
                                                    const Sophus::SE3f & camToWorld,
                                                    const PointT * p){
                 
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

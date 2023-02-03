#include "DepthModel.hpp"
#include "DataStructures/VoxelMap.h"
#include "Utils/Settings.h"
#include "cvo/CvoGPU.hpp"
#include "cvo/Association.hpp"
#include "utils/CvoPointCloud.hpp"
#include <vector>
#include <unordered_set>

namespace dsm {


  void filter_based_on_inner_product(const cvo::CvoPointCloud & source,
                                     const Eigen::Matrix4f & T_source,
                                     const cvo::CvoPointCloud & target,
                                     const Eigen::Matrix4f & T_target,
                                     const cvo::CvoGPU & cvo_align,
                                     cvo::CvoPointCloud & filtered_source
                                     ) {
    const auto& settings = Settings::getInstance();
            
      cvo::Association association_mat;
      Eigen::Matrix3f kernel;
      //if (settings.enableDepthRegression) {
        kernel << settings.depthNormalEll, 0, 0,
          0, settings.depthNormalEll, 0,
          0,  0,   settings.depthDirEll;
        //} else {
        //}
      cvo_align.compute_association_gpu(source,
                                        target,
                                        T_target.inverse() * T_source,
                                        kernel,
                                        association_mat);
      int counter = 0;

      //if (settings.enableDepthRegression ) {
      filtered_source.reserve(association_mat.source_inliers.size(),
                              source.num_features(),
                              source.num_classes());
      for (int j = 0; j < association_mat.source_inliers.size(); j++) {
        int ind = association_mat.source_inliers[j];
        filtered_source.add_point(j, source.at(ind),
                                  source.feature_at(ind),
                                  source.label_at(ind),
                                  source.geometry_type_at(ind));
      }
      std::cout<<__func__<<": filter from "<<source.size()<<" to "<<filtered_source.size()<<" points\n";
    
  }
  
  void downsample_cvo_pc(const cvo::CvoPointCloud & input,
                         float voxel_size,
                         cvo::CvoPointCloud & output) {

    std::cout<<"Current num is "<<input.size()<<
      "Voxel Filter size "<<voxel_size<<"\n";
      
    VoxelMap<SimplePoint> voxel_filter(voxel_size);
    // add all pixels to voxel filter
    std::vector<SimplePoint> temp_pt_vec;
    temp_pt_vec.reserve(input.size());

    
    for (int i = 0; i < input.size(); i++) {
      
      temp_pt_vec.emplace_back(input.at(i)(0), input.at(i)(1), input.at(i)(2), i);
      voxel_filter.insert_point(&temp_pt_vec.back());
    }
    
    // Pick one point from every existing voxel
    const std::vector<SimplePoint*> downsampled = voxel_filter.sample_points();

    std::unordered_set<unsigned int> sampled_indices;    
    for (const SimplePoint* pt : downsampled) {
      sampled_indices.insert(pt->pixelIdx);
    }

    output.reserve(sampled_indices.size(), input.num_features(), input.num_classes());
    //for (int i = 0; i < sampled_indices.size(); i++) {
    int i = 0;
    for (auto ind_in_input: sampled_indices) {
      //unsigned int ind_in_input = sampled_indices[i];
      output.add_point(i, input.at(ind_in_input),
                       input.feature_at(ind_in_input),
                       input.label_at(ind_in_input),
                       input.geometry_type_at(ind_in_input));
      i++;
    }

  }
  
}

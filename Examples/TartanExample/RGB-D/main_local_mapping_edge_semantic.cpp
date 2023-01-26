#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <fstream>
#include <cmath>
#include <boost/filesystem.hpp>
#include <tbb/tbb.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "utils/CvoPointCloud.hpp"
#include "Mapping/bkioctomap.h"
#include "utils/Calibration.hpp"
#include "utils/ImageRGBD.hpp"
#include "utils/ImageStereo.hpp"
#include "dataset_handler/TartanAirHandler.hpp"
#include "dataset_handler/PoseLoader.hpp"

using namespace std;
using namespace boost::filesystem;

typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    MatrixXf_row;


void write_to_geotype_pcd(const cvo::CvoPointCloud & pc,
                          const std::string & name) {
  if (pc.num_geometric_types() < 2)
    return;
    
  pcl::PointCloud<pcl::PointXYZL> pcd;
  for (int i = 0; i < pc.size(); i++) {
    pcl::PointXYZL p;
    p.getVector3fMap() = pc.at(i);
    int l;
    pc.geometry_type_at(i).maxCoeff(&l);
    p.label = (uint32_t) l;
    pcd.push_back(p);
  }
  pcl::io::savePCDFileASCII(name ,pcd); 
  std::cout << "Finished write to label pcd" << std::endl; 
}


void map_to_pc(  semantic_bki::SemanticBKIOctoMap & map,
                 cvo::CvoPointCloud & pc){

  int num_pts = 0;
  for (auto it = map.begin_leaf(); it != map.end_leaf(); ++it) {
    if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
      num_pts++;
    }
  }
    //num_classes_ = num_classes;
  std::vector<std::vector<float>> features;
  std::vector<std::vector<float>> labels;
  pc.reserve(num_pts, 5, 0);
  int ind = 0;
  for (auto it = map.begin_leaf(); it != map.end_leaf(); ++it) {
    if (it.get_node().get_state() == semantic_bki::State::OCCUPIED) {
      // position
      semantic_bki::point3f p = it.get_loc();
      Eigen::Vector3f xyz;
      xyz << p.x(), p.y(), p.z();
      //positions_.push_back(xyz);
      // features
      std::vector<float> feature_vec(5, 0);
      it.get_node().get_features(feature_vec);
      Eigen::VectorXf feature = Eigen::Map<Eigen::VectorXf>(feature_vec.data(), 5);
      
      //features.push_back(feature);
      // labels
      //std::vector<float> label(num_classes_, 0);
      //it.get_node().get_occupied_probs(label);
      //labels.push_back(label);
      Eigen::VectorXf label, geometric_type;
      pc.add_point(ind, xyz, feature,label, geometric_type );
    }
  }
}

int main(int argc, char *argv[]) {
  // list all files in current directory.
  //You could put any file path in here, e.g. "/home/me/mwah" to list that directory
  std::cout<<argc<<std::endl;

  cvo::TartanAirHandler tartan(argv[1]);
  int total_iters = tartan.get_total_number();

  string cvo_param_file(argv[2]);
  string calib_file;
  calib_file = string(argv[1] ) +"/cvo_calib_stereo.txt"; 
  //std::string output_dir(argv[3]);
  int start_frame = stoi(argv[3]);
  int num_frames = stoi(argv[4]);
  std::string odom_file(argv[5]);
  int num_class = std::stoi(argv[6]);   
  
  vector<string> files;
  std::cout<<" cycle through the directory\n";
  int total_num = 0;
  int last_frame = std::min(start_frame+num_frames-1, total_iters-1);
  tartan.set_start_index(start_frame);
  cvo::Calibration  calib(calib_file, cvo::Calibration::STEREO);

  // Mapping
  // Set parameters
  int block_depth = 1;
  double sf2 = 1.0;
  double ell = 0.5;
  float prior = 0.0f;
  float var_thresh = 1.0f;
  double free_thresh = 0.8;
  double occupied_thresh = 0.95;
  double resolution = 0.1;
  double free_resolution = 0.1;
  double ds_resolution = -1;
  double max_range = -1;

  // Read camera poses
  std::vector<Eigen::Matrix4d,
              Eigen::aligned_allocator<Eigen::Matrix4d>> poses;
  cvo::read_pose_file_tartan_format(odom_file, start_frame, last_frame, poses);
  std::cout<<"Just read poses\n";
  
  // Build map
  //std::vector<cvo::CvoPointCloud> pc_vec(files.size());
  semantic_bki::SemanticBKIOctoMap map_csm(resolution, block_depth, num_class + 1, 5+1, sf2, ell, prior, var_thresh, free_thresh, occupied_thresh);
  cvo::CvoPointCloud pc_full(5,num_class);
  int i = 0;
  for (; i+start_frame <= last_frame; i++) {
    
    std::cout << "Read Frame " << i+start_frame << std::endl;
    //pc_vec[i].read_cvo_pointcloud_from_file(f);
    cv::Mat source_left, source_right;
    std::vector<float> semantics;
    tartan.read_next_stereo(source_left, source_right, num_class, semantics);

    std::shared_ptr<cvo::ImageStereo> source_stereo(new cvo::ImageStereo(source_left, source_right, num_class, semantics));
    std::shared_ptr<cvo::CvoPointCloud> pc_with_semantics(new cvo::CvoPointCloud(*source_stereo,
                                                                                 calib
                                                                                 ,cvo::CvoPointCloud::DSO_EDGES
                                                                                 ));


    /*
    cvo::CvoPointCloud pc_with_semantics(pc_original->num_features(), num_class);
    pc_with_semantics.reserve(pc_original->size(), pc_original->num_features(), num_class);
    for (int k = 0; k < pc_original->size(); k++) {
      Eigen::VectorXf label(num_class);
      if (num_class)
        label = Eigen::VectorXf::Zero(num_class);
      Eigen::VectorXf geometric_type(2);
      geometric_type << 1, 0;
      pc_with_semantics.add_point(k, pc_original->at(k), pc_original->feature_at(k),
                                  label, geometric_type);
    }
    */
    
    // transform point cloud
    Eigen::Matrix4f transform = poses[i].cast<float>();
    cvo::CvoPointCloud transformed_pc(5,num_class);
    cvo::CvoPointCloud::transform(transform,*pc_with_semantics, transformed_pc);
    pc_full += transformed_pc;

    semantic_bki::point3f origin;
    origin.x() = transform(0, 3);
    origin.y() = transform(1, 3);
    origin.z() = transform(2, 3);

    // insert point cloud
    map_csm.insert_pointcloud_csm(&transformed_pc, origin, ds_resolution, free_resolution, max_range);
    transformed_pc.write_to_color_pcd(std::to_string(i+start_frame)+".pcd");
    tartan.next_frame_index();
    
  }
  
  // Map to CVOPointCloud
  //cvo::CvoPointCloud cloud_out(&map_csm, num_class);
  cvo::CvoPointCloud pc_map(5,num_class);
  semantic_bki::map_to_pc(map_csm, pc_map, 5, num_class, 2);
  //pc_vec[0].write_to_color_pcd(output_dir + "/" + "input_color.pcd");
  //pc_vec[0].write_to_label_pcd(output_dir + "/" + "input_semantics.pcd");
  pc_map.write_to_color_pcd("map.pcd");
  pc_map.write_to_label_pcd("map_label.pcd");
  write_to_geotype_pcd(pc_map, "map_geotype.pcd");
  pc_full.write_to_color_pcd("stacked_pc.pcd");
  //cloud_out.write_to_label_pcd(output_dir + "/" + "test_semantics.pcd");

  return 0;
}

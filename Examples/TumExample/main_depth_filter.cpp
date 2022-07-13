
#include <iostream>
#include <thread>
#include <memory>

#include "glog/logging.h"
#include "opencv2/imgproc.hpp"
#include "FullSystem/FullSystem.h"
#include "FullSystem/LMCW.h"
#include "Optimization/PhotometricBA.h"
#include "Utils/Settings.h"
#include "DataStructures/ActivePoint.h"
#include "Utils/GlobalCalibration.h"

// used by cvo point cloud registration
#include "dataset_handler/TumHandler.hpp"
#include "utils/ImageRGBD.hpp"
#include "utils/Calibration.hpp"
#include "utils/CvoPointCloud.hpp"
#include "cvo/CvoGPU.hpp"
#include "cvo/CvoParams.hpp"
#include "utils/CvoPoint.hpp"
#include <pcl/point_cloud.h>
// Use the best GPU available for rendering (visualization)


void writeImageWithPts(std::vector<std::shared_ptr<dsm::Frame>> &frames,
                       const Eigen::Matrix3f  & K,
                       std::string & pre_str,
                       bool drawEpipolarLine=false){
  std::vector<cv::Mat> images(frames.size());
  for (int i = 0; i < frames.size(); i++) {
    images[i] = frames[i]-> getRawImage()->image().clone();
  }

  for (int j = 0; j < frames[0]->activePoints().size(); j++) {
    Eigen::Vector4f pt_f0;
    pt_f0.head(3) = frames[0]->activePoints()[j]->xyz();
    pt_f0(3) = 1;
    Eigen::Vector4f pt_half_f0;
    pt_half_f0.head(3) = pt_f0.head(3) /10;
    pt_half_f0(3) = 1;

    cv::Scalar color(rand() % 255,rand() % 255,rand() % 255 );

    for (int i = 1; i < frames.size(); i++) {
      Sophus::SE3f fi_to_f0_sophus = frames[i]->camToWorld().inverse() * frames[0]->camToWorld();
      
      Eigen::Matrix<float, 3, 4> fi_to_f0 = fi_to_f0_sophus.matrix().block<3,4>(0,0);

      Eigen::Matrix<float, 3, 4> P = K * fi_to_f0;

      Eigen::Vector3f uv_i = P*pt_f0;
      uv_i = (uv_i/uv_i(2)).eval();
      Eigen::Vector3f uv_half_i = P*pt_half_f0;
      uv_half_i = (uv_half_i/uv_half_i(2)).eval();

      cv::Point uv, uv_half;
      uv.x = uv_i(0);
      uv.y = uv_i(1);
      uv_half.x = uv_half_i(0);
      uv_half.y = uv_half_i(1);

      cv::line(images[i], uv, uv_half, color, 2);
      cv::circle( images[i],
                  uv,
                  5,
                  color,
                  1,
                  cv::LINE_8 );
      cv::putText(images[i], //target image
                  cv::String(std::to_string(pt_f0(2)) + std::string("m")), //text
                  uv,
                  cv::FONT_HERSHEY_DUPLEX,
                  0.5,
                  color, //font color
                  1.0);
      
    }
    
  }

  for (int i = 0; i < frames.size(); i++) {
    cv::imwrite(pre_str + std::to_string(i)+".png", images[i]); 
  }
  

}

void writeFramePc(std::vector<std::shared_ptr<dsm::Frame>> & frames,
                  std::string & fname,
                  bool candidateOrActive // true: candidate. false: active
                  ) {
  pcl::PointCloud<pcl::PointXYZRGB> pc_all;
  for (auto && frame : frames ) {
    cvo::CvoPointCloud pc_cvo, pc;
    if (candidateOrActive)
      frame->candidatesToCvoPointCloud(pc_cvo);
    else
      frame->activePointsToCvoPointCloud(pc_cvo);
    Sophus::SE3f pose_s = frame->camToWorld();
    Eigen::Matrix4f pose = pose_s.matrix();
    
    cvo::CvoPointCloud::transform(pose, pc_cvo, pc);
    pcl::PointCloud<pcl::PointXYZRGB> pc_pcl;
    pc.export_to_pcd<pcl::PointXYZRGB>(pc_pcl);
    pc_all += pc_pcl;
  }
  pcl::io::savePCDFileASCII (fname.c_str(), pc_all);
}


void read_tum_pose_file(const std::string & tracking_fname,
                          std::vector<int> & frame_inds,
                          std::vector<Sophus::SE3f,
                          Eigen::aligned_allocator<Sophus::SE3f>> & poses_all) {

  poses_all.resize(frame_inds.size());
  std::ifstream gt_file(tracking_fname);

  std::string line;
  int line_ind = 0, curr_frame_ind = 0;

  //std::string gt_file_subset(selected_pose_fname);
  //ofstream outfile(gt_file_subset);
  
  while (std::getline(gt_file, line)) {
    
    if (line_ind < frame_inds[curr_frame_ind]) {
      line_ind ++;
      continue;
    }

    std::cout <<"read line "<<line_ind<<":"<< line<<std::endl;
    
    std::stringstream line_stream(line);
    std::string substr;
    std::string timestamp;
    float xyz[3];
    float q[4]; // x y z w
    line_stream >> timestamp;
    std::string xyz_str[3];
    line_stream >> xyz_str[0] >> xyz_str[1] >> xyz_str[2];
    xyz[0] = std::stof(xyz_str[0]);
    xyz[1] = std::stof(xyz_str[1]);
    xyz[2] = std::stof(xyz_str[2]);
    std::string q_str[4];
    line_stream >> q_str[0] >> q_str[1] >> q_str[2] >> q_str[3];
    q[0] = stod(q_str[0]);
    q[1] = stod(q_str[1]);
    q[2] = stod(q_str[2]);
    q[3] = stod(q_str[3]);
    Eigen::Quaternionf q_eigen(q[3], q[0], q[1], q[2]);
    Sophus::SO3f quat(q_eigen);
    Eigen::Vector3f trans = Eigen::Map<Eigen::Vector3f>(xyz);
    Sophus::SE3f pose_sophus(quat, trans);

    std::cout<<"Read pose \n"<<pose_sophus.matrix()<<std::endl;
    poses_all[curr_frame_ind] = pose_sophus;     
    //Eigen::Matrix<double, 4,4, Eigen::RowMajor> pose_id = Eigen::Matrix<double, 4,4, Eigen::RowMajor>::Identity();
    //poses_all[curr_frame_ind] = pose_id.block<3,4>(0,0);    
    //if (curr_frame_ind == 2) {
    //  std::cout<<"read: line "<<frame_inds[curr_frame_ind]<<" pose is "<<poses_all[curr_frame_ind]<<std::endl;
    //}
    
    line_ind ++;
    curr_frame_ind++;
    //if (line_ind == frame_inds.size())
    if (curr_frame_ind == frame_inds.size())
      break;
  }

  ///  outfile.close();
  gt_file.close();
}




void static_depth_stereo(std::vector<int> frame_inds,
                         std::vector<Sophus::SE3f, Eigen::aligned_allocator<Sophus::SE3f>> & poses,
                         cvo::TumHandler & reader,
                         std::string & settingsFile,
                         cvo::Calibration & cvo_calib,
                         std::string& cvoConfigFile
                         ) {


  // create DSM
  std::unique_ptr<dsm::FullSystem> DSM;
  std::vector<std::shared_ptr<dsm::Frame>> frames;
  std::vector<cvo::CvoPointCloud> pcs_global(frame_inds.size());
  
  for (int i = 0; i < frame_inds.size(); i++) {
    int id = frame_inds[i];
    reader.set_start_index(id);
      
    cv::Mat source_left, source_dep;
    std::cout<< " Read new image "<<id<<std::endl;
    bool read_fails = reader.read_next_rgbd(source_left, source_dep);
    if (read_fails ) {
      std::cout <<" Read fails\n";
      return;
    }
    std::vector<uint16_t> source_dep_data(source_dep.begin<uint16_t>(), source_dep.end<uint16_t>());

    std::shared_ptr<cvo::ImageRGBD<uint16_t>> source_raw(new cvo::ImageRGBD<uint16_t>(source_left, source_dep_data));
    pcl::PointCloud<cvo::CvoPoint>::Ptr source_pcd(new pcl::PointCloud<cvo::CvoPoint>); 
    cvo::CvoPointCloud source_cvo(*source_raw, cvo_calib, cvo::CvoPointCloud::DSO_EDGES);
    std::cout<<"DSO PointSelector selects "<<source_cvo.size()<<" num of points\n";
    
    std::shared_ptr<cvo::CvoPointCloud>  source_full(new cvo::CvoPointCloud(*source_raw, cvo_calib,cvo::CvoPointCloud::FULL ));
    cvo::CvoPointCloud_to_pcl(source_cvo, *source_pcd);
    
    double time = (double)cv::getTickCount();
    
    //gray image from source
    auto & color_img  = source_left;
    cv::Mat gray_img;
    cv::cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);

    if (i == 0)
      DSM = std::make_unique<dsm::FullSystem>(color_img.cols,
                                              color_img.rows,
                                              cvo_calib, cvoConfigFile,
                                              settingsFile,
                                              nullptr);
    
    double timestamp = static_cast<double>(i);
    std::shared_ptr<dsm::Frame> trackingNewFrame = std::make_shared<dsm::Frame>(id, timestamp, gray_img.data, source_raw, source_pcd, cvo_calib.scaling_factor(),
                                                                                source_full);


    //DSM->createCandidatesWithInitDepth(trackingNewFrame);
    DSM->createCandidates(trackingNewFrame);
    
    trackingNewFrame->dump_candidates_to_pcd(std::to_string(i)+"before_tracking.pcd");
    Eigen::Matrix4f pose_eigen = poses[i].matrix();
    cvo::CvoPointCloud pc_candidate;
    trackingNewFrame->candidatesToCvoPointCloud(pc_candidate);
    cvo::CvoPointCloud::transform(pose_eigen, pc_candidate, pcs_global[i]);
    

    if (i)  {
      const dsm::Frame * parent = frames[0].get();
      const Sophus::SE3f parent_pose = frames[0]->camToWorld();
      const Sophus::SE3f poseToParent = parent_pose.inverse()  * poses[i];      
      trackingNewFrame->setTrackingResult(frames[0].get(), poseToParent );
    }
    DSM->trackCandidates(trackingNewFrame,
                         frames);
    trackingNewFrame->dump_candidates_to_pcd(std::to_string(i)+".pcd");
    
    frames.push_back(trackingNewFrame);
    trackingNewFrame->evolveToKeyframe();
    trackingNewFrame->activate();
    trackingNewFrame->setKeyframeID(i);
    trackingNewFrame->setActiveID(i);
    
    trackingNewFrame->setCamToWorld(poses[i]);
    trackingNewFrame->setFrameBlockPose(poses[i]);
    //DSM->trackFrame(id, timestamp, trackingNewFrame);
    

  }


  const auto& calib = dsm::GlobalCalibration::getInstance();
  const Eigen::Matrix3f& K = calib.matrix3f(0);
  
  //activate all point clouds
  std::string fname = "before_tracking.pcd";
  pcl::PointCloud<pcl::PointXYZRGB> pcs_all;
  for (int i = 0; i < frame_inds.size(); i++) {
    pcl::PointCloud<pcl::PointXYZRGB> pc;
    pcs_global[i].export_to_pcd(pc);
    pcs_all += pc;
  }
  pcl::io::savePCDFileASCII(fname.c_str(), pcs_all);

  
  //frames.pop_back();
  fname = "before_refinement.pcd";
  writeFramePc(frames, fname, true);
  std::string preStr="before_";
  writeImageWithPts(frames,
                    K,
                    preStr
                    );


  DSM->refineCandidates(frames, frames);
  DSM->get_lmcw()->activatePointsCvo(DSM->ceresOptimizer, frames, 0);  
  //DSM->lmcw->selectTemporalFrameCvo(DSM->ceresOptimizer);
   
  fname = "after_refinement.pcd";
  writeFramePc(frames, fname, true);
   
  
  DSM->ceresOptimizer->solve(frames);
  fname = "after_BA.pcd";
  writeFramePc(frames, fname, false);
  frames[1]->dump_active_points_to_pcd("2.after.pcd");

  preStr="after_";
  writeImageWithPts(frames,
                    K,
                    preStr
                    );
    
  
}


int main(int argc, char *argv[])
{
  // input arguments
  std::string imageFolder, settingsFile, trackingFname, selectedFramesGraphFile, cvoConfigFile;
  
  // Configuration
  if (argc == 6)
  {
    imageFolder = argv[1];
    settingsFile = argv[2];
    trackingFname = argv[3];
    selectedFramesGraphFile = argv[4];
    cvoConfigFile = argv[5];
  }
  else
  {
    std::cout << "The main_depth_filter requires at least 4 arguments: imageFolder, settingsFile, trackingFname, selectedFramesGraphFile;\n";
    return 0;
  }

  // Initialize logging
  google::InitGoogleLogging(argv[0]);
  //FLAGS_logtostderr = 1;
  //FLAGS_v = 9;

  std::cout << "\n";

  // read sequence
  std::vector<int> frame_inds;
  std::ifstream graphF(selectedFramesGraphFile);
  std::cout<<"Read frame ";
  while (!graphF.eof()) {
    int ind;
    graphF >> ind;
    frame_inds.push_back(ind);
    std::cout<<ind<<", ";
  }
  std::cout<<"\n";

  frame_inds.pop_back();
  
  std::vector<Sophus::SE3f,
              Eigen::aligned_allocator<Sophus::SE3f>> poses;
  read_tum_pose_file(trackingFname, frame_inds, poses);

  // run processing in a second thread
  cvo::TumHandler tum(imageFolder);
  std::string cvo_calib_file = imageFolder + "/cvo_calib.txt"; 
  cvo::Calibration calib(cvo_calib_file, cvo::Calibration::RGBD);
  
  static_depth_stereo(frame_inds, poses, tum,  settingsFile,
                      calib, cvoConfigFile);


  std::cout << "Finished!" << std::endl;

  return 0;
}

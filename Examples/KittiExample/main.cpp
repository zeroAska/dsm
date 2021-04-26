
#include <iostream>
#include <thread>
#include <memory>

#include "glog/logging.h"

#include "opencv2/imgproc.hpp"

#include "QtVisualizer.h"
#include "FullSystem/FullSystem.h"

// used by cvo point cloud registration
#include "dataset_handler/KittiHandler.hpp"
#include "utils/RawImage.hpp"
#include "utils/Calibration.hpp"
#include "utils/CvoPointCloud.hpp"
#include "cvo/CvoGPU.hpp"
#include "cvo/CvoParams.hpp"
#include "utils/CvoPoint.hpp"
#include <pcl/point_cloud.h>
// Use the best GPU available for rendering (visualization)
#ifdef WIN32
extern "C"
{
  __declspec(dllexport) uint32_t NvOptimusEnablement = 0x00000001;
  __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

namespace cvo {
  
  void CvoPointCloud_to_pcl(const CvoPointCloud & cvo_cloud,
                            pcl::PointCloud<CvoPoint> &pcl_cloud
                            ) {
    int num_points = cvo_cloud.num_points();
    const ArrayVec3f & positions = cvo_cloud.positions();
    const Eigen::Matrix<float, Eigen::Dynamic, FEATURE_DIMENSIONS> & features = cvo_cloud.features();
    const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> & normals = cvo_cloud.normals();
    // const Eigen::Matrix<float, Eigen::Dynamic, 2> & types = cvo_cloud.types();
    auto & labels = cvo_cloud.labels();
    // set basic informations for pcl_cloud
    pcl_cloud.resize(num_points);

    //int actual_num = 0;
    for(int i=0; i<num_points; ++i){
      //memcpy(&host_cloud[i], &cvo_cloud[i], sizeof(CvoPoint));
      (pcl_cloud)[i].x = positions[i](0);
      (pcl_cloud)[i].y = positions[i](1);
      (pcl_cloud)[i].z = positions[i](2);
      if (FEATURE_DIMENSIONS >= 3) {
        (pcl_cloud)[i].r = (uint8_t)std::min(255.0, (features(i,0) * 255.0));
        (pcl_cloud)[i].g = (uint8_t)std::min(255.0, (features(i,1) * 255.0));
        (pcl_cloud)[i].b = (uint8_t)std::min(255.0, (features(i,2) * 255.0));
      }

      for (int j = 0; j < FEATURE_DIMENSIONS; j++)
        pcl_cloud[i].features[j] = features(i,j);

      if (cvo_cloud.num_classes() > 0) {
        labels.row(i).maxCoeff(&pcl_cloud[i].label);
        for (int j = 0; j < cvo_cloud.num_classes(); j++)
          pcl_cloud[i].label_distribution[j] = labels(i,j);
      }
      
      if (normals.rows() > 0 && normals.cols()>0) {
        for (int j = 0; j < 3; j++)
          pcl_cloud[i].normal[j] = normals(i,j);
      }

      if (cvo_cloud.covariance().size() > 0 )
        memcpy(pcl_cloud[i].covariance, cvo_cloud.covariance().data()+ i*9, sizeof(float)*9  );
      if (cvo_cloud.eigenvalues().size() > 0 )
        memcpy(pcl_cloud[i].cov_eigenvalues, cvo_cloud.eigenvalues().data() + i*3, sizeof(float)*3);

      //if (i == 1000) {
      //  printf("Total %d, Raw input from pcl at 1000th: \n", num_points);
      //  print_point(pcl_cloud[i]);
      //}
      
    }
    //gpu_cloud->points = host_cloud;

    /*
      #ifdef IS_USING_COVARIANCE    
      auto covariance = &cvo_cloud.covariance();
      auto eigenvalues = &cvo_cloud.eigenvalues();
      thrust::device_vector<float> cov_gpu(cvo_cloud.covariance());
      thrust::device_vector<float> eig_gpu(cvo_cloud.eigenvalues());
      copy_covariances<<<host_cloud.size()/256 +1, 256>>>(thrust::raw_pointer_cast(cov_gpu.data()),
      thrust::raw_pointer_cast(eig_gpu.data()),
      host_cloud.size(),
      thrust::raw_pointer_cast(gpu_cloud->points.data()));
      #endif    
    */
    return;
  }



}


namespace dsm
{
  class KittiProcessor
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    inline KittiProcessor() { this->shouldStop = false; }
    inline ~KittiProcessor() { this->join(); }

    inline void run(cvo::KittiHandler & reader, QtVisualizer& visualizer, std::string& settingsFile,
                    std::string & cvoConfigFile, cvo::Calibration  &cvo_calib
                    )
    {
      this->processThread = std::make_unique<std::thread>(&KittiProcessor::doRun, this,
                                                          std::ref(reader),
                                                          std::ref(visualizer),
                                                          std::ref(settingsFile),
                                                          std::ref(cvoConfigFile),
                                                          std::ref(cvo_calib));
    }

    inline void join()
    {
      this->shouldStop = true;

      // wait the thread to exit
      if (this->processThread->joinable())
      {
        std::cout << "Waiting Processor to finish.." << std::endl;

        this->processThread->join();

        std::cout << " .. Processor has finished." << std::endl;
      }
    }

  private:

    inline void doRun(cvo::KittiHandler & reader,
                      QtVisualizer& visualizer,
                      std::string& settingsFile,
                      std::string & cvoConfigFile,
                      cvo::Calibration  &cvo_calib)
    {
      int id = 0;
      cv::Mat image;
      double timestamp;

      const double fps = 0.1;//reader.fps();

      // create DSM
      std::unique_ptr<FullSystem> DSM;

      while (!this->shouldStop)
      {
        // reset
        if (visualizer.getDoReset())
        {
          // reset slam system
          DSM.reset();

          // reset variables
          id = 0;
          timestamp = 0;
          image.release();

          // reset visualizer
          visualizer.reset();

          // reset dataset reader
          reader.set_start_index(0);
        }

        cv::Mat source_left, source_right;
        //std::vector<float> semantics_source;
        //reader.read_next_stereo(source_left, source_right, NUM_CLASSES, semantics_source);
        std::cout<< " Read new image "<<id<<std::endl;
        bool read_fails = reader.read_next_stereo(source_left, source_right);
        //kitti.read_next_stereo(source_left, source_right);
        //std::shared_ptr<cvo::RawImage> source_raw(new cvo::RawImage(source_left, NUM_CLASSES, semantics_source));
        //std::shared_ptr<cvo::RawImage> source_raw(new cvo::RawImage(source_left));
        if (read_fails) this->shouldStop = true;
        //cvo::RawImage source_raw(source_left));

        
        std::shared_ptr<cvo::RawImage> source_raw(new cvo::RawImage(source_left));
        pcl::PointCloud<cvo::CvoPoint>::Ptr source(new pcl::PointCloud<cvo::CvoPoint>); 
        cvo::CvoPointCloud source_cvo(*source_raw, source_right, cvo_calib);
        cvo::CvoPointCloud_to_pcl(source_cvo, *source);
        
        // capture
        //if (visualizer.getDoProcessing() && !read_fails)

        if ( !read_fails)
        {
          double time = (double)cv::getTickCount();

          //gray image from source
          auto & color_img  = source_left;
          cv::Mat gray_img;
          cv::cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);


          // undistort
          //undistorter.undistort(image, image);


          
          if (DSM == nullptr)
          {
            DSM = std::make_unique<FullSystem>(color_img.cols,
                                               color_img.rows,
                                               cvo_calib.intrinsic(), cvoConfigFile,
                                               settingsFile,
                                               &visualizer);
          }

          // process
          //DSM->trackFrame(id, timestamp, gray_img.data);
          DSM->trackFrame(id, timestamp, gray_img.data, source_raw, source);
          
          //cv::imwrite("new_tracked_gray.png", gray_img);
          // visualize image
          visualizer.publishLiveFrame(gray_img);
          //visualizer.publishLiveFrame(color_img);

          // increase counter
          ++id;
          reader.next_frame_index();

          // wait 
          time = 1000.0*(cv::getTickCount() - time) / cv::getTickFrequency();
          const double delay = (1000.0 / fps) - time;

          if (delay > 0.0)
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<unsigned int>(delay)));
          }
        }
        else
        {
          // allow other threads to run
          std::this_thread::yield();
        }
      }

      // print log
      if (DSM)
      {
        DSM->printLog();
      }
    }

  private:

    bool shouldStop;

    std::unique_ptr<std::thread> processThread;
  };
}

int main(int argc, char *argv[])
{
  // input arguments
  std::string imageFolder, cvoConfigFile, calibFile, settingsFile;

  // Configuration
  if (argc == 4)
  {
    imageFolder = argv[1];
    cvoConfigFile = argv[2];
    settingsFile = argv[3];
  }
  else
  {
    std::cout << "The KittiExample requires at least 3 arguments: imageFolder, cvoSettingFile, dsmCettingsFile (optional)\n";
    return 0;
  }

  // Initialize logging
  google::InitGoogleLogging(argv[0]);
  //FLAGS_logtostderr = 1;
  //FLAGS_v = 9;

  std::cout << "\n";

  // Create the application before the window always!
  // create visualizer in the main thread
  QApplication app(argc, argv);
  dsm::QtVisualizer visualizer(app);

  std::cout << "\n";

  // read sequence
  cvo::KittiHandler kitti(imageFolder, 0);

  std::string cvo_calib_file = imageFolder + "/cvo_calib.txt"; 
  cvo::Calibration calib(cvo_calib_file);

  // add image size to the visualizer
  visualizer.setImageSize(calib.image_cols(), calib.image_rows());

  // run processing in a second thread
  dsm::KittiProcessor processor;
  processor.run(kitti,  visualizer, settingsFile, cvoConfigFile, calib);

  // run main window
  // it will block the main thread until closed
  visualizer.run();

  // join processing thread
  processor.join();

  std::cout << "Finished!" << std::endl;

  return 0;
}

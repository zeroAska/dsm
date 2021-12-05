
#include <iostream>
#include <thread>
#include <memory>

#include "glog/logging.h"

#include "opencv2/imgproc.hpp"

#include "QtVisualizer.h"
#include "FullSystem/FullSystem.h"

// used by cvo point cloud registration
#include "dataset_handler/TumHandler.hpp"
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


namespace dsm
{
  class TumProcessor
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    inline TumProcessor() { this->shouldStop = false; }
    inline ~TumProcessor() { this->join(); }

    inline void run(cvo::TumHandler & reader, QtVisualizer& visualizer, std::string& settingsFile,
                    std::string & cvoConfigFile, cvo::Calibration  &cvo_calib, int startFrameId,
                    std::string & trajFileName
                    )
    {
      this->processThread = std::make_unique<std::thread>(&TumProcessor::doRun, this,
                                                          std::ref(reader),
                                                          std::ref(visualizer),
                                                          std::ref(settingsFile),
                                                          std::ref(cvoConfigFile),
                                                          std::ref(cvo_calib),
                                                          startFrameId,
                                                          std::ref(trajFileName));
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

    inline void doRun(cvo::TumHandler & reader,
                      QtVisualizer& visualizer,
                      std::string& settingsFile,
                      std::string & cvoConfigFile,
                      cvo::Calibration  &cvo_calib,
                      int startFrameId,
                      std::string & trajFileName)
    {
      int id = startFrameId;
      cv::Mat image;
      double timestamp;

      std::ofstream trajFile(trajFileName);

      const double fps = 0.1;//reader.fps();

      // create DSM
      std::unique_ptr<FullSystem> DSM;
      reader.set_start_index(id);
      
      while (!this->shouldStop)
      {
        // reset
        if (visualizer.getDoReset())
        {
          // reset slam system
          DSM.reset();

          // reset variables
          id = startFrameId;
          timestamp = 0;
          image.release();

          // reset visualizer
          visualizer.reset();

          // reset dataset reader
          reader.set_start_index(id);
        }

        cv::Mat source_left, source_dep;
        //std::vector<float> semantics_source;
        //reader.read_next_stereo(source_left, source_right, NUM_CLASSES, semantics_source);
        std::cout<< " Read new image "<<id<<std::endl;
        bool read_fails = reader.read_next_rgbd(source_left, source_dep);
        //kitti.read_next_stereo(source_left, source_right);
        //std::shared_ptr<cvo::RawImage> source_raw(new cvo::RawImage(source_left, NUM_CLASSES, semantics_source));
        //std::shared_ptr<cvo::RawImage> source_raw(new cvo::RawImage(source_left));
        if (read_fails) this->shouldStop = true;
        //cvo::RawImage source_raw(source_left));

        if ( !read_fails)
        {
          
          std::shared_ptr<cvo::RawImage> source_raw(new cvo::RawImage(source_left));
          std::vector<uint16_t> source_dep_data(source_dep.begin<uint16_t>(), source_dep.end<uint16_t>());
          
          pcl::PointCloud<cvo::CvoPoint>::Ptr source_pcd(new pcl::PointCloud<cvo::CvoPoint>); 
          cvo::CvoPointCloud source_cvo(*source_raw, source_dep_data, cvo_calib);

          //if (id <= 2) source_cvo.write_to_color_pcd("color_"+std::to_string(id)+".pcd");

          cvo::CvoPointCloud_to_pcl(source_cvo, *source_pcd);
          
          double time = (double)cv::getTickCount();

          //gray image from source
          auto & color_img  = source_left;
          cv::Mat gray_img;
          cv::cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);


          
          if (DSM == nullptr)
          {
            DSM = std::make_unique<FullSystem>(color_img.cols,
                                               color_img.rows,
                                               cvo_calib, cvoConfigFile,
                                               settingsFile,
                                               &visualizer);
          }

          // process
          //DSM->trackFrame(id, timestamp, gray_img.data);
          std::shared_ptr<Frame> trackingNewFrame = std::make_shared<Frame>(id, timestamp, gray_img.data, source_raw, source_pcd, source_dep_data, cvo_calib.scaling_factor());
    
          DSM->trackFrame(id, timestamp, trackingNewFrame);
          
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
          if (id) {
            std::cout<<"Time statistics: "
                     <<"\ntracking time: "<<DSM->getCamTrackingMeanTime()
                     <<"\ndepth filter time: "<<DSM->getPointTrackingMeanTime()
                     <<"\ntotal Backend time "<<DSM->getTotalBackendMeanTime()
                     <<"\nwindow construction time "<<DSM->getWindowConstructionMeanTime()
                     <<"\ncvo BA time "<<DSM->getCvoBAMeanTime()
                     <<"\nPhotometric BA time "<<DSM->getLocalBAMeanTime()
                     <<"\n\n";
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


        //std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses;
        std::vector<Eigen::Matrix4f> poses;
        std::vector<double> timestamps;
        DSM->getTrajectory(poses, timestamps);

        int l = 0;
        for (auto && accum_mat : poses) {

          Eigen::Quaternionf q(accum_mat.block<3,3>(0,0));
          trajFile<<timestamps[l]<<" ";
          trajFile<<accum_mat(0,3)<<" "<<accum_mat(1,3)<<" "<<accum_mat(2,3)<<" "; 
          trajFile<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<"\n";
          trajFile.flush();
          
          DSM->printLog();          
          l++;

        }
        
        
      }


      
      
      trajFile.close();
      
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
  int startFrameId;
  // Configuration
  if (argc >= 5)
  {
    imageFolder = argv[1];
    cvoConfigFile = argv[2];
    settingsFile = argv[3];
    startFrameId = std::stoi(argv[4]);
  }
  else
  {
    std::cout << "The TumExample requires at least 4 arguments: imageFolder, cvoConfigFile, dsmCettingsFile, startFrameIndex\n";
    return 0;
  }

  std::string trajFileName;
  if (argc == 6)
    trajFileName = std::string(argv[5]);

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
  cvo::TumHandler tum(imageFolder);


  std::string cvo_calib_file = imageFolder + "/cvo_calib.txt"; 
  cvo::Calibration calib(cvo_calib_file, cvo::Calibration::RGBD);

  // add image size to the visualizer
  visualizer.setImageSize(calib.image_cols(), calib.image_rows());

  // run processing in a second thread
  dsm::TumProcessor processor;
  processor.run(tum,  visualizer, settingsFile, cvoConfigFile, calib, startFrameId, trajFileName);

  // run main window
  // it will block the main thread until closed
  visualizer.run();

  // join processing thread
  processor.join();

  std::cout << "Finished!" << std::endl;

  app.exec();
  
  return 0;
}

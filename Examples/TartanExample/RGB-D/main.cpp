
#include <iostream>
#include <thread>
#include <memory>

#include "glog/logging.h"

#include "opencv2/imgproc.hpp"

#include "QtVisualizer.h"

#include "FullSystem/FullSystem.h"

// used by cvo point cloud registration
#include "dataset_handler/TartanAirHandler.hpp"
#include "utils/RawImage.hpp"
#include "utils/ImageRGBD.hpp"
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
  class TartanProcessor
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    inline TartanProcessor() { this->shouldStop = false; }
    inline ~TartanProcessor() { this->join(); }

    inline void run(cvo::TartanAirHandler& reader, QtVisualizer& visualizer, std::string& settingsFile,
                    std::string& cvoConfigFile, cvo::Calibration& cvo_calib, int startFrameId,
                    std::string& trajFileName
                    )
    {
      this->processThread = std::make_unique<std::thread>(&TartanProcessor::doRun, this,
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
    inline void doRun(cvo::TartanAirHandler& reader,
                      QtVisualizer& visualizer,
                      std::string& settingsFile,
                      std::string& cvoConfigFile,
                      cvo::Calibration& cvo_calib,
                      int startFrameId,
                      std::string& trajFileName)
    {
      int id = startFrameId;
      cv::Mat image;
      double timestamp;

      std::ofstream trajFile(trajFileName);
      //trajFile << std::setprecision(6) << std::endl;

      trajFile.close();
      const double fps = 0.1;//reader.fps();

      // create DSM
      std::unique_ptr<FullSystem> DSM;
      reader.set_start_index(id);

      while (!this->shouldStop)
      {
        //reset
        if (visualizer.getDoReset())
        {
          DSM.reset();
          id = startFrameId;
          timestamp = 0;
          image.release();

          visualizer.reset();

          reader.set_start_index(id);
        }

        cv::Mat source_left;
        std::vector<float> source_dep;        
        std::cout<< " Read new image "<<id<<std::endl;
        bool read_fails = reader.read_next_rgbd(source_left, source_dep);

        if (read_fails) this->shouldStop = true;

        if (!read_fails)
        {

          std::shared_ptr<cvo::ImageRGBD<float>> source_raw(new cvo::ImageRGBD(source_left, source_dep));
          pcl::PointCloud<cvo::CvoPoint>::Ptr source_pcd(new pcl::PointCloud<cvo::CvoPoint>);

          //cvo::CvoPointCloud source_cvo(*source_raw, cvo_calib, cvo::CvoPointCloud::CANNY_EDGES);
          cvo::CvoPointCloud source_cvo(*source_raw, cvo_calib, cvo::CvoPointCloud::DSO_EDGES);
          std::shared_ptr<cvo::CvoPointCloud> source_full(new cvo::CvoPointCloud(*source_raw, cvo_calib, cvo::CvoPointCloud::FULL));
          source_full->write_to_color_pcd("source_full.pcd");

          cvo::CvoPointCloud_to_pcl(source_cvo, *source_pcd);

          double time = (double)cv::getTickCount();

          auto& color_img = source_left;
          cv::Mat gray_img;
          cv::cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);

          // TL: terminate at 10th frame
          // if (id == startFrameId + 10) this->shouldStop = true; 

          if (DSM == nullptr)
          {
            DSM = std::make_unique<FullSystem>(color_img.cols,
                                               color_img.rows,
                                               cvo_calib, cvoConfigFile,
                                               settingsFile,
                                               &visualizer);
          }

          // process
          // TODO: check timestamp
          std::shared_ptr<Frame> trackingNewFrame = std::make_shared<Frame>(id, timestamp, gray_img.data, source_raw, source_pcd, cvo_calib.scaling_factor(),
                                                                            source_full);
          DSM->trackFrame(id, timestamp, trackingNewFrame);
          visualizer.publishLiveFrame(gray_img);

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


        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses;
        // std::vector<Eigen::Matrix4f> poses;
        std::vector<double> timestamps;
        std::vector<int> ids;
        DSM->getFullTrajectory(poses, timestamps, ids);

        int l = 0;
        for (auto && accum_mat : poses) {
          std::ofstream trajFile(trajFileName, std::ios::app);
          Eigen::Quaternionf q(accum_mat.block<3,3>(0,0));
          trajFile<<std::fixed << std::setprecision(18) << std::scientific << accum_mat(0,3)<<" "<<accum_mat(1,3)<<" "<<accum_mat(2,3)<<" "; 
          trajFile<<q.x()<<" "<<q.y()<<" "<<q.z()<<" "<<q.w()<<"\n";
          trajFile.flush();
          trajFile.close();          
          DSM->printLog();          
          l++;

        }
        std::cout<<"Wrote "<<l<<"lines of poses to the file\n";
        
        
      }

      sleep(2);
      exit(0);
    }

  private:
    bool shouldStop;
    std::unique_ptr<std::thread> processThread;
  };
}

int main(int argc, char *argv[])
{
  std::string imageFolder, cvoConfigFile, calibFile, settingsFile;
  int startFrameId;
  if (argc >= 5)
  {
    imageFolder = argv[1];
    cvoConfigFile = argv[2];
    settingsFile = argv[3];
    startFrameId = std::stoi(argv[4]);
  }
  else
  {
    std::cout << "The TartanExample requires at least 4 arguments: imageFolder, cvoConfigFile, dsmCettingsFile, startFrameIndex\n";
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
  cvo::TartanAirHandler tartan(imageFolder);
  tartan.set_depth_folder_name("deep_depth");

  std::string cvo_calib_file = imageFolder + "/cvo_calib_deep_depth.txt";
  std::cout<<"cvo_calib file is "<<cvo_calib_file<<"\n";
  cvo::Calibration calib(cvo_calib_file, cvo::Calibration::RGBD);

  // add image size to the visualizer
  visualizer.setImageSize(calib.image_cols(), calib.image_rows());

  // run processing in a second thread
  dsm::TartanProcessor processor;
  processor.run(tartan,  visualizer, settingsFile, cvoConfigFile, calib, startFrameId, trajFileName);

  // run main window
  // it will block the main thread until closed
  visualizer.run();

  // join processing thread
  processor.join();

  std::cout << "Finished!" << std::endl;

  app.exec();

  return 0;
}

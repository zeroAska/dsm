
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
  class KittiProcessor
  {
  public:

    inline Processor() { this->shouldStop = false; }
    inline ~Processor() { this->join(); }

    inline void run(cvo::KittiHandler & reader, QtVisualizer& visualizer, std::string& settingsFile,
                    cvo::CvoGPU & cvo_align, cvo::Calibration  &cvo_calib
                    )
    {
      this->processThread = std::make_unique<std::thread>(&Processor::doRun, this,
                                                          std::ref(reader), std::ref(undistorter), std::ref(visualizer), std::ref(settingsFile));
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

    inline void doRun(cvo::KittiHandler & reader, QtVisualizer& visualizer, std::string& settingsFile,                                                                                                    cvo::CvoGPU & cvo_align, cvo::Calibration  &cvo_calib)
    {
      int id = 0;
      cv::Mat image;
      double timestamp;

      const double fps = 1;//reader.fps();

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
        bool read_fails = reader.read_next_stereo(source_left, source_right);
        //kitti.read_next_stereo(source_left, source_right);
        //std::shared_ptr<cvo::RawImage> source_raw(new cvo::RawImage(source_left, NUM_CLASSES, semantics_source));
        std::shared_ptr<cvo::RawImage> source_raw(new cvo::RawImage(source_left));
        std::shared_ptr<cvo::CvoPointCloud> source(new cvo::CvoPointCloud(*source_raw, source_right, cvo_calib));
        
        // capture
        if (visualizer.getDoProcessing() && !read_fails)
        {
          double time = (double)cv::getTickCount();

          //gray image from source
          auto & color_img  = source_raw.color();
          cv::Mat gray_img;
          cv::cvtColor(color_img, gray_img, cv::COLOR_BGR2GRAY);


          // undistort
          //undistorter.undistort(image, image);


          
          if (DSM == nullptr)
          {
            DSM = std::make_unique<FullSystem>(color_img.cols,
                                               color_img.rows,
                                               cvo_calib.intrinsic(), settingsFile,
                                               &visualizer);
          }

          // process
          //DSM->trackFrame(id, timestamp, gray_img);
          DSM->trackFrame(id, timestamp, gray_img, source_raw, source);
          

          // visualize image
          visualizer.publishLiveFrame(gray_img);

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
  if (argc == 5)
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
  cvo::CvoGPU cvo_align(cvoConfigFile );
  dsm::KittiProcessor processor;
  processor.run(kitti,  visualizer, settingsFile, cvo_align, calib);

  // run main window
  // it will block the main thread until closed
  visualizer.run();

  // join processing thread
  processor.join();

  std::cout << "Finished!" << std::endl;

  return 0;
}

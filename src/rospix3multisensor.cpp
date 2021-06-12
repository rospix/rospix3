/* includes //{ */

#include <memory>
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <pluginlib/class_list_macros.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>

#include <clusteringapi.h>
#include <c_callback_wrapper.h>

#include <boost/filesystem.hpp>

#include <rad_msgs/ClusterList.h>

//}

#define SINGLE_CHIP_PIXSIZE 65536
#define ERRMSG_BUFF_SIZE 512

namespace rospix3
{

/* class Rospix3Multisensor //{ */

/* typedef void (clMessageCallback_t)(bool error, const char* message, void* user_data); */

class Rospix3Multisensor : public nodelet::Nodelet {

public:
  virtual void onInit();

  ros::NodeHandle nh_;

private:
  bool is_initialized_ = false;

  // | ---------------------- general stuff --------------------- |

  bool        _verbose_ = false;
  std::string _frame_prefix_;

  int                      n_sensors_;
  std::vector<std::string> _sensors_;
  std::vector<std::string> _sensor_aliases_;

  // path to a directory where raw t3pa data is going to be stored
  std::string _raw_data_path_;
  bool        _save_raw_ = false;

  // _rad_folder_path_ with the current session subfolder
  std::vector<std::string> raw_data_full_path_;

  bool _use_calibration_ = false;

  std::string _calibration_path_;

  bool _noisy_pixels_identification_enabled_;
  bool _noisy_pixels_masking_enabled_;

  double _acquisition_duration_ = 0;
  double _measurement_duration_ = 0;

  // | ------------------------- timers ------------------------- |

  void       timerMeasurement(const ros::TimerEvent& te);
  ros::Timer timer_measurement_;
  double     _rate_timer_measurement_;

  void       timerPublisher(const ros::TimerEvent& te);
  ros::Timer timer_publisher_;
  double     _rate_timer_publisher_;

  // | ----------------------- publishers ----------------------- |

  std::vector<ros::Publisher> pub_cluster_list_;

  // | ------------------------- Rospix3Multisensor ------------------------ |

  std::vector<std::unique_ptr<clhandle_t>> timepix_handlers_;

  void callbackTimepixMessage0(bool error, const char* message, void* user_data);
  void callbackTimepixProgress0(bool finished, double progress, void* user_data);
  void callbackTimepixNewClustersWithPixels0(PXPClusterWithPixels* clusters, size_t cluster_count, size_t acq_index, void* user_data);
  void callbackTimepixAcquisitionStart0(int acqIndex, void* userData);
  void callbackTimepixAcquisitionFinished0(int acqIndex, void* userData);

  void callbackTimepixMessage1(bool error, const char* message, void* user_data);
  void callbackTimepixProgress1(bool finished, double progress, void* user_data);
  void callbackTimepixNewClustersWithPixels1(PXPClusterWithPixels* clusters, size_t cluster_count, size_t acq_index, void* user_data);
  void callbackTimepixAcquisitionStart1(int acqIndex, void* userData);
  void callbackTimepixAcquisitionFinished1(int acqIndex, void* userData);

  std::vector<int>       acquisition_counter_;
  std::vector<bool>      acquisition_in_progress_;
  std::vector<ros::Time> acquisition_start_time_;

  // | ------------------- cluster list queue ------------------- |

  std::vector<std::unique_ptr<std::list<rad_msgs::Cluster>>> cluster_list_;
  std::vector<std::unique_ptr<std::mutex>>                   mutex_cluster_list_;
};

//}

/* onInit() //{ */

void Rospix3Multisensor::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[ComptonConeGenerator]: initializing");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  mrs_lib::ParamLoader param_loader(nh_, "Rospix3Multisensor");

  param_loader.loadParam("sensors", _sensors_);
  n_sensors_ = _sensors_.size();
  param_loader.loadParam("sensor_aliases", _sensor_aliases_);

  param_loader.loadParam("verbose", _verbose_);
  param_loader.loadParam("frame_prefix", _frame_prefix_);

  param_loader.loadParam("raw_data_path", _raw_data_path_);
  param_loader.loadParam("save_raw_data", _save_raw_);

  param_loader.loadParam("use_calibration", _use_calibration_);
  param_loader.loadParam("calibration_path", _calibration_path_);

  param_loader.loadParam("acquisition_duration", _acquisition_duration_);
  param_loader.loadParam("measurement_duration", _measurement_duration_);

  param_loader.loadParam("measurement_timer_rate", _rate_timer_measurement_);
  param_loader.loadParam("publisher_timer_rate", _rate_timer_publisher_);

  param_loader.loadParam("noisy_pixel_masking/identify", _noisy_pixels_identification_enabled_);
  param_loader.loadParam("noisy_pixel_masking/mask", _noisy_pixels_identification_enabled_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Rospix3Multisensor]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  for (int i = 0; i < n_sensors_; i++) {
    acquisition_counter_.push_back(0);
    acquisition_start_time_.push_back(ros::Time(0));
    acquisition_in_progress_.push_back(false);
    mutex_cluster_list_.push_back(std::make_unique<std::mutex>());
    cluster_list_.push_back(std::make_unique<std::list<rad_msgs::Cluster>>());
  }

  // | ------------------- load the pixet core ------------------ |

  int rc;  // return code

  rc = pxpClLoadPixetCore(std::string(ros::package::getPath("rospix3") + "/lib/x64/pxcore.so").c_str());
  if (rc == 0) {
    ROS_INFO("[Rospix3Multisensor]: pixet core loaded");
  } else {
    ROS_ERROR("[Rospix3Multisensor]: pixet core loading error: %d", rc);
  }

  for (int i = 0; i < n_sensors_; i++) {

    timepix_handlers_.push_back(std::make_unique<clhandle_t>(pxpClCreate(i)));

    // | ---------------- load detector calibration --------------- |

    if (_use_calibration_) {

      std::string calibration_paths;

      calibration_paths += _calibration_path_ + "/" + _sensors_[i] + "/a.txt" + "|";
      calibration_paths += _calibration_path_ + "/" + _sensors_[i] + "/b.txt" + "|";
      calibration_paths += _calibration_path_ + "/" + _sensors_[i] + "/c.txt" + "|";
      calibration_paths += _calibration_path_ + "/" + _sensors_[i] + "/t.txt";

      rc = pxpClLoadCalibrationFromFiles(*(timepix_handlers_[i]), calibration_paths.c_str());

      if (rc == 0) {
        ROS_INFO("[Rospix3Multisensor]: (%s): calibration loaded", _sensors_[i].c_str());
      } else {
        ROS_ERROR("[Rospix3Multisensor]: (%s): calibration loading error: %d", _sensors_[i].c_str(), rc);
      }
    }

    // | ------------------ noisy pixel handling ------------------ |

    rc = pxpClEnableFilteringOfNoisyPixels(*(timepix_handlers_[i]), _noisy_pixels_identification_enabled_);

    if (rc == 0) {
      ROS_INFO("[Rospix3Multisensor]: (%s): noisy pixel identification set", _sensors_[i].c_str());
    } else {
      ROS_ERROR("[Rospix3Multisensor]: (%s): error while setting noisy pixel identification: %d", _sensors_[i].c_str(), rc);
    }

    // | ------------- bind the timepix api callbacks ------------- |

    if (i == 0) {

      ClMessageCallback callback_message_ptr =
          GETCB(ClMessageCallback, Rospix3Multisensor)(boost::bind(&Rospix3Multisensor::callbackTimepixMessage0, this, _1, _2, _3));

      ClProgressCallback callback_progress_ptr =
          GETCB(ClProgressCallback, Rospix3Multisensor)(boost::bind(&Rospix3Multisensor::callbackTimepixProgress0, this, _1, _2, _3));

      ClNewClustersWithPixelsCallback callback_new_clusters_with_pixels_ptr = GETCB(
          ClNewClustersWithPixelsCallback, Rospix3Multisensor)(boost::bind(&Rospix3Multisensor::callbackTimepixNewClustersWithPixels0, this, _1, _2, _3, _4));

      ClAcqStartedCallback callback_acquisition_start_ptr =
          GETCB(ClAcqStartedCallback, Rospix3Multisensor)(boost::bind(&Rospix3Multisensor::callbackTimepixAcquisitionStart0, this, _1, _2));

      ClAcqFinishedCallback callback_acquisition_finished_ptr =
          GETCB(ClAcqFinishedCallback, Rospix3Multisensor)(boost::bind(&Rospix3Multisensor::callbackTimepixAcquisitionFinished0, this, _1, _2));

      pxpClSetMessageCallback(*(timepix_handlers_[i]), callback_message_ptr, nullptr);
      pxpClSetProgressCallback(*(timepix_handlers_[i]), callback_progress_ptr, nullptr);
      pxpClSetNewClustersWithPixelsCallback(*(timepix_handlers_[i]), callback_new_clusters_with_pixels_ptr, nullptr);
      pxpClSetAcqStartedCallback(*(timepix_handlers_[i]), callback_acquisition_start_ptr, nullptr);
      pxpClSetAcqFinishedCallback(*(timepix_handlers_[i]), callback_acquisition_finished_ptr, nullptr);

    } else {

      ClMessageCallback callback_message_ptr =
          GETCB(ClMessageCallback, Rospix3Multisensor)(boost::bind(&Rospix3Multisensor::callbackTimepixMessage1, this, _1, _2, _3));

      ClProgressCallback callback_progress_ptr =
          GETCB(ClProgressCallback, Rospix3Multisensor)(boost::bind(&Rospix3Multisensor::callbackTimepixProgress1, this, _1, _2, _3));

      ClNewClustersWithPixelsCallback callback_new_clusters_with_pixels_ptr = GETCB(
          ClNewClustersWithPixelsCallback, Rospix3Multisensor)(boost::bind(&Rospix3Multisensor::callbackTimepixNewClustersWithPixels1, this, _1, _2, _3, _4));

      ClAcqStartedCallback callback_acquisition_start_ptr =
          GETCB(ClAcqStartedCallback, Rospix3Multisensor)(boost::bind(&Rospix3Multisensor::callbackTimepixAcquisitionStart1, this, _1, _2));

      ClAcqFinishedCallback callback_acquisition_finished_ptr =
          GETCB(ClAcqFinishedCallback, Rospix3Multisensor)(boost::bind(&Rospix3Multisensor::callbackTimepixAcquisitionFinished1, this, _1, _2));

      pxpClSetMessageCallback(*(timepix_handlers_[i]), callback_message_ptr, nullptr);
      pxpClSetProgressCallback(*(timepix_handlers_[i]), callback_progress_ptr, nullptr);
      pxpClSetNewClustersWithPixelsCallback(*(timepix_handlers_[i]), callback_new_clusters_with_pixels_ptr, nullptr);
      pxpClSetAcqStartedCallback(*(timepix_handlers_[i]), callback_acquisition_start_ptr, nullptr);
      pxpClSetAcqFinishedCallback(*(timepix_handlers_[i]), callback_acquisition_finished_ptr, nullptr);
    }

    // | ------------ create a fodler for the raw data ------------ |

    if (_save_raw_) {

      std::stringstream ss;
      ss << std::fixed;
      ss << int(ros::Time::now().toSec());
      raw_data_full_path_.push_back(_raw_data_path_ + "/" + _sensor_aliases_[i] + "/" + ss.str());
      ROS_INFO("[Rospix3Multisensor]: (%s): raw_folder: %s", _sensors_[i].c_str(), raw_data_full_path_[i].c_str());

      if (!boost::filesystem::create_directories(raw_data_full_path_[i])) {
        ROS_ERROR("[Rospix3Multisensor]: (%s): could not create directory '%s' for the raw data!", _sensors_[i].c_str(), raw_data_full_path_[i].c_str());
        ros::shutdown();
      }
    }

    // | ----------------------- publishers ----------------------- |

    pub_cluster_list_.push_back(nh_.advertise<rad_msgs::ClusterList>(_sensor_aliases_[i] + "/cluster_list", 1));
  }

  // | ----------------------- main timer ----------------------- |

  timer_measurement_ = nh_.createTimer(ros::Rate(_rate_timer_measurement_), &Rospix3Multisensor::timerMeasurement, this);
  timer_publisher_   = nh_.createTimer(ros::Rate(_rate_timer_publisher_), &Rospix3Multisensor::timerPublisher, this);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO_ONCE("[Rospix3Multisensor]: initialized");
}

//}

// | ------------------ Timepix API callbacks ----------------- |

// 0

/* callbackTimepixMessage0() //{ */

void Rospix3Multisensor::callbackTimepixMessage0(bool error, const char* message, [[maybe_unused]] void* user_data) {

  int i = 0;

  if (error) {
    ROS_ERROR("[Rospix3Multisensor]: (%s): %s", _sensors_[i].c_str(), message);
  } else {
    ROS_INFO("[Rospix3Multisensor]: (%s): %s", _sensors_[i].c_str(), message);
  }
}

//}

/* callbackTimepixProgress0() //{ */

void Rospix3Multisensor::callbackTimepixProgress0(bool finished, double progress, [[maybe_unused]] void* user_data) {

  int sensor = 0;

  ROS_INFO("[Rospix3Multisensor]: (%s): callbackProgress(): %.2f", _sensors_[sensor].c_str(), progress);

  if (finished) {
    ROS_INFO("[Rospix3Multisensor]: (%s): callbackProgress(): finished", _sensors_[sensor].c_str());
  }
}

//}

/* callbackTimepixNewClustersWithPixels0() //{ */

void Rospix3Multisensor::callbackTimepixNewClustersWithPixels0(PXPClusterWithPixels* clusters, size_t cluster_count, size_t acq_index,
                                                               [[maybe_unused]] void* user_data) {

  int sensor = 0;

  if (_verbose_) {
    ROS_INFO("[Rospix3Multisensor]: (%s): callbackClusterWithPixels(): new clusters: count=%u, acq_index=%u\n", _sensors_[sensor].c_str(),
             static_cast<unsigned>(cluster_count), static_cast<unsigned>(acq_index));
  }

  std::scoped_lock lock(*(mutex_cluster_list_[sensor]));

  for (size_t i = 0; i < cluster_count; i++) {

    rad_msgs::Cluster cluster;

    _PXPClusterWithPixels& cl = clusters[i];

    cluster.stamp = acquisition_start_time_[sensor] + ros::Duration(cl.toa / 1e9);

    cluster.energy    = cl.energy;
    cluster.height    = cl.height;
    cluster.id        = cl.eventID;
    cluster.roundness = cl.roundness;
    cluster.size      = cl.size;
    cluster.toa       = cl.toa;
    cluster.x         = cl.x;
    cluster.y         = cl.y;

    for (size_t j = 0; j < cl.size; j++) {

      rad_msgs::Pixel pixel;
      pixel.x      = cl.pixels[j].x;
      pixel.y      = cl.pixels[j].y;
      pixel.energy = cl.pixels[j].energy;
      pixel.toa    = cl.pixels[j].toa;

      cluster.pixels.push_back(pixel);
    }

    cluster_list_[sensor]->push_back(cluster);
  }
}

//}

/* callbackTimepixAcquisitionStart0() //{ */

void Rospix3Multisensor::callbackTimepixAcquisitionStart0([[maybe_unused]] int acqIndex, [[maybe_unused]] void* userData) {

  int sensor = 0;

  acquisition_in_progress_[sensor] = true;
  acquisition_start_time_[sensor]  = ros::Time::now();

  if (_verbose_) {
    ROS_INFO("[Rospix3Multisensor]: (%s): acquisition %d started", _sensors_[sensor].c_str(), acqIndex);
  }
}

//}

/* callbackTimepixAcquisitionFinished0() //{ */

void Rospix3Multisensor::callbackTimepixAcquisitionFinished0(int acqIndex, [[maybe_unused]] void* userData) {

  int sensor = 0;

  acquisition_counter_[sensor] = acqIndex;

  if (_verbose_) {
    ROS_INFO("[Rospix3Multisensor]: (%s): acquisition %d finished", _sensors_[sensor].c_str(), acqIndex);
  }

  if (_noisy_pixels_identification_enabled_ && _noisy_pixels_masking_enabled_) {

    int rc = pxpClMaskNoisyPixels(*(timepix_handlers_[sensor]));

    if (rc == 0) {
      ROS_INFO("[Rospix3Multisensor]: (%s): masking noisy pixels", _sensors_[sensor].c_str());
    } else {
      ROS_ERROR("[Rospix3Multisensor]: (%s) error while masking noisy pixels: %d", _sensors_[sensor].c_str(), rc);
    }
  }
}

//}

// 1

/* callbackTimepixMessage1() //{ */

void Rospix3Multisensor::callbackTimepixMessage1(bool error, const char* message, [[maybe_unused]] void* user_data) {

  int i = 1;

  if (error) {
    ROS_ERROR("[Rospix3Multisensor]: (%s): %s", _sensors_[i].c_str(), message);
  } else {
    ROS_INFO("[Rospix3Multisensor]: (%s): %s", _sensors_[i].c_str(), message);
  }
}

//}

/* callbackTimepixProgress1() //{ */

void Rospix3Multisensor::callbackTimepixProgress1(bool finished, double progress, [[maybe_unused]] void* user_data) {

  int sensor = 1;

  ROS_INFO("[Rospix3Multisensor]: (%s): callbackProgress(): %.2f", _sensors_[sensor].c_str(), progress);

  if (finished) {
    ROS_INFO("[Rospix3Multisensor]: (%s): callbackProgress(): finished", _sensors_[sensor].c_str());
  }
}

//}

/* callbackTimepixNewClustersWithPixels1() //{ */

void Rospix3Multisensor::callbackTimepixNewClustersWithPixels1(PXPClusterWithPixels* clusters, size_t cluster_count, size_t acq_index,
                                                               [[maybe_unused]] void* user_data) {

  int sensor = 1;

  if (_verbose_) {
    ROS_INFO("[Rospix3Multisensor]: (%s): callbackClusterWithPixels(): new clusters: count=%u, acq_index=%u\n", _sensors_[sensor].c_str(),
             static_cast<unsigned>(cluster_count), static_cast<unsigned>(acq_index));
  }

  std::scoped_lock lock(*(mutex_cluster_list_[sensor]));

  for (size_t i = 0; i < cluster_count; i++) {

    rad_msgs::Cluster cluster;

    _PXPClusterWithPixels& cl = clusters[i];

    cluster.stamp = acquisition_start_time_[sensor] + ros::Duration(cl.toa / 1e9);

    cluster.energy    = cl.energy;
    cluster.height    = cl.height;
    cluster.id        = cl.eventID;
    cluster.roundness = cl.roundness;
    cluster.size      = cl.size;
    cluster.toa       = cl.toa;
    cluster.x         = cl.x;
    cluster.y         = cl.y;

    for (size_t j = 0; j < cl.size; j++) {

      rad_msgs::Pixel pixel;
      pixel.x      = cl.pixels[j].x;
      pixel.y      = cl.pixels[j].y;
      pixel.energy = cl.pixels[j].energy;
      pixel.toa    = cl.pixels[j].toa;

      cluster.pixels.push_back(pixel);
    }

    cluster_list_[sensor]->push_back(cluster);
  }
}

//}

/* callbackTimepixAcquisitionStart1() //{ */

void Rospix3Multisensor::callbackTimepixAcquisitionStart1([[maybe_unused]] int acqIndex, [[maybe_unused]] void* userData) {

  int sensor = 1;

  acquisition_in_progress_[sensor] = true;
  acquisition_start_time_[sensor]  = ros::Time::now();

  if (_verbose_) {
    ROS_INFO("[Rospix3Multisensor]: (%s): acquisition %d started", _sensors_[sensor].c_str(), acqIndex);
  }
}

//}

/* callbackTimepixAcquisitionFinished1() //{ */

void Rospix3Multisensor::callbackTimepixAcquisitionFinished1(int acqIndex, [[maybe_unused]] void* userData) {

  int sensor = 1;

  acquisition_counter_[sensor] = acqIndex;

  if (_verbose_) {
    ROS_INFO("[Rospix3Multisensor]: (%s): acquisition %d finished", _sensors_[sensor].c_str(), acqIndex);
  }

  if (_noisy_pixels_identification_enabled_ && _noisy_pixels_masking_enabled_) {

    int rc = pxpClMaskNoisyPixels(*(timepix_handlers_[sensor]));

    if (rc == 0) {
      ROS_INFO("[Rospix3Multisensor]: (%s): masking noisy pixels", _sensors_[sensor].c_str());
    } else {
      ROS_ERROR("[Rospix3Multisensor]: (%s) error while masking noisy pixels: %d", _sensors_[sensor].c_str(), rc);
    }
  }
}

//}

// | --------------------- timer callbacks -------------------- |

/* timerMeasurement() //{ */

void Rospix3Multisensor::timerMeasurement([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  ROS_INFO_ONCE("[Rospix3Multisensor]: main timer spinning");

  for (int i = 0; i < n_sensors_; i++) {

    if (pxpClIsRunning(*(timepix_handlers_[i]))) {
      return;
    }

    std::stringstream ss;
    ss << std::fixed;
    ss << std::setprecision(2);
    ss << ros::Time::now().toSec();

    std::string t3pa_filename = "";

    if (_save_raw_) {
      t3pa_filename = raw_data_full_path_[i] + "/" + ss.str() + ".t3pa";
    }

    int rc = pxpClStartMeasurement(*(timepix_handlers_[i]), _acquisition_duration_, _measurement_duration_, t3pa_filename.c_str());

    if (rc == 0) {
      ROS_INFO("[Rospix3Multisensor]: (%s): measurement started", _sensors_[i].c_str());
    } else {
      ROS_ERROR("[Rospix3Multisensor]: (%s): measurement failed, %d", _sensors_[i].c_str(), rc);
    }
  }
}

//}

/* timerPublisher() //{ */

void Rospix3Multisensor::timerPublisher([[maybe_unused]] const ros::TimerEvent& te) {

  for (int sensor = 0; sensor < n_sensors_; sensor++) {

    std::scoped_lock lock(*(mutex_cluster_list_[sensor]));

    rad_msgs::ClusterList cluster_list_msg;

    cluster_list_msg.header.stamp    = acquisition_start_time_[sensor];
    cluster_list_msg.header.frame_id = _frame_prefix_ + "/minipix_" + _sensor_aliases_[sensor];

    for (size_t i = 0; i < cluster_list_[sensor]->size(); i++) {
      cluster_list_msg.clusters.push_back(cluster_list_[sensor]->front());
      cluster_list_[sensor]->pop_front();
    }

    try {
      pub_cluster_list_[sensor].publish(cluster_list_msg);
    }
    catch (...) {
      ROS_ERROR("exception caught during publishing topic '%s'", pub_cluster_list_[sensor].getTopic().c_str());
    }
  }
}

//}

// | -------------------- support functions ------------------- |

}  // namespace rospix3

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(rospix3::Rospix3Multisensor, nodelet::Nodelet);

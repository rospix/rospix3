/* includes //{ */

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <pluginlib/class_list_macros.h>

#include <dynamic_reconfigure/server.h>
#include <rospix3/rospix3Config.h>

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

/* class Rospix3 //{ */

/* typedef void (clMessageCallback_t)(bool error, const char* message, void* user_data); */

class Rospix3 : public nodelet::Nodelet {

public:
  virtual void onInit();

  ros::NodeHandle nh_;

private:
  bool is_initialized_ = false;

  // | ---------------------- general stuff --------------------- |

  bool        _verbose_ = false;
  std::string _frame_id_;

  // path to a directory where raw t3pa data is going to be stored
  std::string _raw_data_path_;
  bool        _save_raw_ = false;

  // _rad_folder_path_ with the current session subfolder
  std::string raw_data_full_path_;

  bool _use_calibration_ = false;

  std::string _calibration_path_;

  ros::Time  measurement_start_time_;
  std::mutex mutex_measurement_start_time_;

  bool _noisy_pixels_identification_enabled_;

  // | ------------------------- timers ------------------------- |

  void       timerMeasurement(const ros::TimerEvent& te);
  ros::Timer timer_measurement_;
  double     _rate_timer_measurement_;

  void       timerPublisher(const ros::TimerEvent& te);
  ros::Timer timer_publisher_;
  double     _rate_timer_publisher_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher pub_cluster_list_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef rospix3::rospix3Config                   DrsConfig_t;
  typedef dynamic_reconfigure::Server<DrsConfig_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(rospix3::rospix3Config& config, uint32_t level);
  DrsConfig_t                                      drs_params_;
  std::mutex                                       mutex_drs_params_;

  // | ------------------------- Rospix3 ------------------------ |

  std::unique_ptr<clhandle_t> timepix_handle_ptr_;

  void callbackTimepixMessage(bool error, const char* message, void* user_data);
  void callbackTimepixProgress(bool finished, double progress, void* user_data);
  void callbackTimepixNewClustersWithPixels(PXPClusterWithPixels* clusters, size_t cluster_count, size_t acq_index, void* user_data);
  void callbackTimepixAcquisitionStart(int acqIndex, void* userData);
  void callbackTimepixAcquisitionFinished(int acqIndex, void* userData);

  int       acquisition_counter_     = 0;
  bool      acquisition_in_progress_ = false;
  ros::Time acquisition_start_time_;

  // | ------------------- cluster list queue ------------------- |

  std::list<rad_msgs::Cluster> cluster_list_;
  std::mutex                   mutex_cluster_list_;
};

//}

/* onInit() //{ */

void Rospix3::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[ComptonConeGenerator]: initializing");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  mrs_lib::ParamLoader param_loader(nh_, "Rospix3");

  param_loader.loadParam("verbose", _verbose_);
  param_loader.loadParam("frame_id", _frame_id_);

  param_loader.loadParam("measuring", drs_params_.measuring);
  param_loader.loadParam("raw_data_path", _raw_data_path_);
  param_loader.loadParam("save_raw_data", _save_raw_);

  param_loader.loadParam("use_calibration", _use_calibration_);
  param_loader.loadParam("calibration_path", _calibration_path_);

  param_loader.loadParam("acquisition_duration", drs_params_.acquisition_duration);
  param_loader.loadParam("measurement_duration", drs_params_.measurement_duration);

  param_loader.loadParam("measurement_timer_rate", _rate_timer_measurement_);
  param_loader.loadParam("publisher_timer_rate", _rate_timer_publisher_);

  param_loader.loadParam("noisy_pixel_masking/identify", _noisy_pixels_identification_enabled_);
  param_loader.loadParam("noisy_pixel_masking/mask", drs_params_.mask_pixels);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Rospix3]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  int rc;  // return code

  // | ------------------- load the pixet core ------------------ |

  rc = pxpClLoadPixetCore(std::string(ros::package::getPath("rospix3") + "/lib/x64/pxcore.so").c_str());
  if (rc == 0) {
    ROS_INFO("[Rospix3]: pixet core loaded");
  } else {
    ROS_ERROR("[Rospix3]: pixet core loading error: %d", rc);
  }

  timepix_handle_ptr_ = std::make_unique<clhandle_t>(pxpClCreate(0));

  // | ---------------- load detector calibration --------------- |

  if (_use_calibration_) {

    std::string calibration_paths;

    calibration_paths += _calibration_path_ + "/a.txt" + "|";
    calibration_paths += _calibration_path_ + "/b.txt" + "|";
    calibration_paths += _calibration_path_ + "/c.txt" + "|";
    calibration_paths += _calibration_path_ + "/t.txt";

    rc = pxpClLoadCalibrationFromFiles(*timepix_handle_ptr_, calibration_paths.c_str());

    if (rc == 0) {
      ROS_INFO("[Rospix3]: calibration loaded");
    } else {
      ROS_ERROR("[Rospix3]: calibration loading error: %d", rc);
    }
  }

  // | ------------------ noisy pixel handling ------------------ |

  rc = pxpClEnableFilteringOfNoisyPixels(*timepix_handle_ptr_, _noisy_pixels_identification_enabled_);

  if (rc == 0) {
    ROS_INFO("[Rospix3]: noisy pixel identification set");
  } else {
    ROS_ERROR("[Rospix3]: error while setting noisy pixel identification: %d", rc);
  }

  // | ------------- bind the timepix api callbacks ------------- |

  ClMessageCallback callback_message_ptr =
      GETCB(ClMessageCallback, Rospix3)(std::bind(&Rospix3::callbackTimepixMessage, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  ClProgressCallback callback_progress_ptr = GETCB(
      ClProgressCallback, Rospix3)(std::bind(&Rospix3::callbackTimepixProgress, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  ClNewClustersWithPixelsCallback callback_new_clusters_with_pixels_ptr = GETCB(ClNewClustersWithPixelsCallback, Rospix3)(std::bind(
      &Rospix3::callbackTimepixNewClustersWithPixels, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  ClAcqStartedCallback callback_acquisition_start_ptr =
      GETCB(ClAcqStartedCallback, Rospix3)(std::bind(&Rospix3::callbackTimepixAcquisitionStart, this, std::placeholders::_1, std::placeholders::_2));

  ClAcqFinishedCallback callback_acquisition_finished_ptr =
      GETCB(ClAcqFinishedCallback, Rospix3)(std::bind(&Rospix3::callbackTimepixAcquisitionFinished, this, std::placeholders::_1, std::placeholders::_2));

  pxpClSetMessageCallback(*timepix_handle_ptr_, callback_message_ptr, nullptr);
  pxpClSetProgressCallback(*timepix_handle_ptr_, callback_progress_ptr, nullptr);
  pxpClSetNewClustersWithPixelsCallback(*timepix_handle_ptr_, callback_new_clusters_with_pixels_ptr, nullptr);
  pxpClSetAcqStartedCallback(*timepix_handle_ptr_, callback_acquisition_start_ptr, nullptr);
  pxpClSetAcqFinishedCallback(*timepix_handle_ptr_, callback_acquisition_finished_ptr, nullptr);

  // | ------------ create a fodler for the raw data ------------ |

  if (_save_raw_) {

    std::stringstream ss;
    ss << std::fixed;
    ss << int(ros::Time::now().toSec());
    raw_data_full_path_ = _raw_data_path_ + "/" + ss.str();
    ROS_INFO_STREAM("[Rospix3]: raw_folder: " << raw_data_full_path_);

    if (!boost::filesystem::create_directories(raw_data_full_path_)) {
      ROS_ERROR("[Rospix3]: could not create directory '%s' for the raw data!", raw_data_full_path_.c_str());
      ros::shutdown();
    }
  }

  // | ----------------------- publishers ----------------------- |

  pub_cluster_list_ = nh_.advertise<rad_msgs::ClusterList>("cluster_list_out", 1);

  // | ----------------------- main timer ----------------------- |

  timer_measurement_ = nh_.createTimer(ros::Rate(_rate_timer_measurement_), &Rospix3::timerMeasurement, this);
  timer_publisher_   = nh_.createTimer(ros::Rate(_rate_timer_publisher_), &Rospix3::timerPublisher, this);

  // | ------------------- dynamic reconfigure ------------------ |

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(drs_params_);
  Drs_t::CallbackType f = boost::bind(&Rospix3::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO_ONCE("[Rospix3]: initialized");
}

//}

// | ------------------ Timepix API callbacks ----------------- |

/* callbackTimepixMessage() //{ */

void Rospix3::callbackTimepixMessage(bool error, const char* message, [[maybe_unused]] void* user_data) {

  if (error) {
    ROS_ERROR("[Rospix3]: %s", message);
  } else {
    ROS_INFO("[Rospix3]: %s", message);
  }
}

//}

/* callbackTimepixProgress() //{ */

void Rospix3::callbackTimepixProgress(bool finished, double progress, [[maybe_unused]] void* user_data) {

  ROS_INFO("[Rospix3]: callbackProgress(): %.2f", progress);

  if (finished) {
    ROS_INFO("[Rospix3]: callbackProgress(): finished");
  }
}

//}

/* callbackTimepixNewClustersWithPixels() //{ */

void Rospix3::callbackTimepixNewClustersWithPixels(PXPClusterWithPixels* clusters, size_t cluster_count, size_t acq_index, [[maybe_unused]] void* user_data) {

  if (_verbose_) {
    ROS_INFO("[Rospix3]: callbackClusterWithPixels(): new clusters: count=%u, acq_index=%u\n", static_cast<unsigned>(cluster_count),
             static_cast<unsigned>(acq_index));
  }

  std::scoped_lock lock(mutex_cluster_list_);

  for (size_t i = 0; i < cluster_count; i++) {

    rad_msgs::Cluster cluster;

    _PXPClusterWithPixels& cl = clusters[i];

    cluster.stamp = acquisition_start_time_ + ros::Duration(cl.toa / 1e9);

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

    cluster_list_.push_back(cluster);
  }
}

//}

/* callbackTimepixAcquisitionStart() //{ */

void Rospix3::callbackTimepixAcquisitionStart([[maybe_unused]] int acqIndex, [[maybe_unused]] void* userData) {

  acquisition_in_progress_ = true;
  acquisition_start_time_  = ros::Time::now();

  if (_verbose_) {
    ROS_INFO("[Rospix3]: acquisition %d started", acqIndex);
  }
}

//}

/* callbackTimepixAcquisitionFinished() //{ */

void Rospix3::callbackTimepixAcquisitionFinished(int acqIndex, [[maybe_unused]] void* userData) {

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  acquisition_counter_ = acqIndex;

  if (_verbose_) {
    ROS_INFO("[Rospix3]: acquisition %d finished", acqIndex);
  }

  if (_noisy_pixels_identification_enabled_ && drs_params.mask_pixels) {

    int rc = pxpClMaskNoisyPixels(*timepix_handle_ptr_);

    if (rc == 0) {
      ROS_INFO("[Rospix3]: masking noisy pixels");
    } else {
      ROS_ERROR("[Rospix3]: error while masking noisy pixels: %d", rc);
    }
  }
}

//}

// | ------------------- dynamic reconfigure ------------------ |

/* callbackDrs() //{ */

void Rospix3::callbackDrs(rospix3::rospix3Config& config, [[maybe_unused]] uint32_t level) {

  {
    std::scoped_lock lock(mutex_drs_params_);

    drs_params_ = config;
  }

  ROS_INFO("[Rospix3]: DRS params updated");
}

//}

// | --------------------- timer callbacks -------------------- |

/* timerMeasurement() //{ */

void Rospix3::timerMeasurement([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  ROS_INFO_ONCE("[Rospix3]: main timer spinning");

  auto drs_params = mrs_lib::get_mutexed(mutex_drs_params_, drs_params_);

  if (pxpClIsRunning(*timepix_handle_ptr_)) {
    return;
  }

  if (drs_params.measuring) {

    std::stringstream ss;
    ss << std::fixed;
    ss << std::setprecision(2);
    ss << ros::Time::now().toSec();

    std::string t3pa_filename = "";

    if (_save_raw_) {
      t3pa_filename = raw_data_full_path_ + "/" + ss.str() + ".t3pa";
    }

    int rc = pxpClStartMeasurement(*timepix_handle_ptr_, drs_params.acquisition_duration, drs_params.measurement_duration, t3pa_filename.c_str());

    {
      std::scoped_lock lock(mutex_measurement_start_time_);

      measurement_start_time_ = ros::Time::now();
    }

    if (rc == 0) {
      ROS_INFO("[Rospix3]: measurement started");
    } else {
      ROS_ERROR("[Rospix3]: measurement failed, %d", rc);
    }
  }
}

//}

/* timerPublisher() //{ */

void Rospix3::timerPublisher([[maybe_unused]] const ros::TimerEvent& te) {

  std::scoped_lock lock(mutex_cluster_list_);

  rad_msgs::ClusterList cluster_list_msg;

  cluster_list_msg.header.stamp    = acquisition_start_time_;
  cluster_list_msg.header.frame_id = _frame_id_;

  for (size_t i = 0; i < cluster_list_.size(); i++) {
    cluster_list_msg.clusters.push_back(cluster_list_.front());
    cluster_list_.pop_front();
  }

  try {
    pub_cluster_list_.publish(cluster_list_msg);
  }
  catch (...) {
    ROS_ERROR("exception caught during publishing topic '%s'", pub_cluster_list_.getTopic().c_str());
  }
}

//}


// | -------------------- support functions ------------------- |

}  // namespace rospix3

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(rospix3::Rospix3, nodelet::Nodelet);

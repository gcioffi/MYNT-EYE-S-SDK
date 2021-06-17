// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/distortion_models.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <opencv2/calib3d/calib3d.hpp>

#include <mynt_eye_ros_wrapper/GetInfo.h>

#define _USE_MATH_DEFINES
#include <cmath>
#include <map>
#include <string>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"
#include "mynteye/device/context.h"
#include "mynteye/device/device.h"
#define CONFIGURU_IMPLEMENTATION 1
#include "configuru.hpp"
using namespace configuru;  // NOLINT

#define PIE 3.1416
#define MATCH_CHECK_THRESHOLD 3

#define FULL_PRECISION \
  std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10)

static const std::size_t MAXSIZE = 4;

MYNTEYE_BEGIN_NAMESPACE

namespace enc = sensor_msgs::image_encodings;
inline double compute_time(const double end, const double start) {
  return end - start;
}

class ROSWrapperNodelet : public nodelet::Nodelet {
 public:
  ROSWrapperNodelet() :
  mesh_position_x(0.),
  mesh_position_y(-0.176),
  mesh_position_z(0.),
  mesh_rotation_x(PIE/2),
  mesh_rotation_y(0.0),
  mesh_rotation_z(PIE/2),
  skip_tag(-1),
  skip_tmp_left_tag(0),
  skip_tmp_right_tag(0) {
    unit_hard_time *= 10;
  }

  ~ROSWrapperNodelet() {
    // std::cout << __func__ << std::endl;
    if (api_) {
      //api_->Stop(Source::ALL);
      api_->Stop(Source::VIDEO_STREAMING);
    }
    if (time_beg_ != -1) {
      double time_end = ros::Time::now().toSec();

      LOG(INFO) << "Time elapsed: " << compute_time(time_end, time_beg_)
                << " s";
      if (left_time_beg_ != -1) {
        LOG(INFO) << "Left count: " << left_count_ << ", fps: "
                  << (left_count_ / compute_time(time_end, left_time_beg_));
      }
      if (right_time_beg_ != -1) {
        LOG(INFO) << "Right count: " << right_count_ << ", fps: "
                  << (right_count_ / compute_time(time_end, right_time_beg_));
      }
      if (imu_time_beg_ != -1) {
          if (model_ == Model::STANDARD) {
            LOG(INFO) << "Imu count: " << imu_count_ << ", hz: "
                      << (imu_count_ /
                      compute_time(time_end, imu_time_beg_));
          } else {
          if (publish_imu_by_sync_) {
            LOG(INFO) << "imu_sync_count: " << imu_sync_count_ << ", hz: "
                      << (imu_sync_count_ /
                      compute_time(time_end, imu_time_beg_));
          } else {
            LOG(INFO) << "Imu count: " << imu_count_ << ", hz: "
                      << (imu_count_ /
                      compute_time(time_end, imu_time_beg_));
          }
        }
      }

      // ROS messages could not be reliably printed here, using glog instead :(
      // ros::Duration(1).sleep();  // 1s
      // https://answers.ros.org/question/35163/how-to-perform-an-action-at-nodelet-unload-shutdown/
    }
  }

  ros::Time hardTimeToSoftTime(std::uint64_t _hard_time) {
    static bool isInited = false;
    static double soft_time_begin(0);
    static std::uint64_t hard_time_begin(0);

    if (false == isInited) {
      soft_time_begin = ros::Time::now().toSec();
      hard_time_begin = _hard_time;
      isInited = true;
    }

    std::uint64_t time_ns_detal = (_hard_time - hard_time_begin);
    std::uint64_t time_ns_detal_s = time_ns_detal / 1000000;
    std::uint64_t time_ns_detal_ns = time_ns_detal % 1000000;
    double time_sec_double =
      ros::Time(time_ns_detal_s, time_ns_detal_ns * 1000).toSec();

    return ros::Time(soft_time_begin + time_sec_double);
  }

  // ros::Time hardTimeToSoftTime(std::uint64_t _hard_time) {
  //   static bool isInited = false;
  //   static double soft_time_begin(0);
  //   static std::uint64_t hard_time_begin(0);

  //   if (false == isInited) {
  //     soft_time_begin = ros::Time::now().toSec();
  //     hard_time_begin = _hard_time;
  //     isInited = true;
  //   }

  //   return ros::Time(
  //       static_cast<double>(soft_time_begin +
  //       static_cast<double>(_hard_time - hard_time_begin) * 0.000001f));
  // }

  inline bool is_overflow(std::uint64_t now,
      std::uint64_t pre) {

    return (now < pre) && ((pre - now) > (unit_hard_time / 2));
  }

  inline bool is_repeated(std::uint64_t now,
      std::uint64_t pre) {
    return now == pre;
  }

  inline bool is_abnormal(std::uint32_t now,
      std::uint32_t pre) {

    return (now < pre) && ((pre - now) < (unit_hard_time / 4));
  }

  ros::Time checkUpTimeStamp(std::uint64_t _hard_time,
      const Stream &stream) {
    static std::map<Stream, std::uint64_t> hard_time_now;
    static std::map<Stream, std::uint64_t> acc;

    if (is_overflow(_hard_time, hard_time_now[stream])) {
      acc[stream]++;
    }

    hard_time_now[stream] = _hard_time;

    return hardTimeToSoftTime(
        acc[stream] * unit_hard_time + _hard_time);
  }

  ros::Time checkUpImuTimeStamp(std::uint64_t _hard_time) {
    static std::uint64_t hard_time_now(0), acc(0);

    if (is_overflow(_hard_time, hard_time_now)) {
      acc++;
    }

    hard_time_now = _hard_time;

    return hardTimeToSoftTime(
        acc * unit_hard_time + _hard_time);
  }

  void onInit() override {
    nh_ = getMTNodeHandle();
    private_nh_ = getMTPrivateNodeHandle();

    initDevice();
    NODELET_FATAL_COND(api_ == nullptr, "No MYNT EYE device selected :(");

    pthread_mutex_init(&mutex_data_, nullptr);

    // publishers
    image_transport::ImageTransport it_mynteye(nh_);
    camera_publishers_[Stream::LEFT] = it_mynteye.advertiseCamera("left", 1);
    camera_publishers_[Stream::RIGHT] = it_mynteye.advertiseCamera("right", 1);

    //ros::Rate loop_rate(frame_rate_);
    ros::Rate loop_rate(100);
    while (private_nh_.ok()) {
      publishTopics();
      loop_rate.sleep();
    }
  }

  void publishTopics() {
    if (!is_started_) {
      time_beg_ = ros::Time::now().toSec();
      //api_->Start(Source::ALL);
      
      // debug
      std::cout << " before video streaming\n";
      // end
      
      api_->Start(Source::VIDEO_STREAMING);

      // debug
      std::cout << " after video streaming\n";
      // end
      
      is_started_ = true;
    }

    // debug
    std::cout << " before wait streaming\n";
    // end
    api_->WaitForStreams();
    // debug
    std::cout << " after wait streaming\n";
    // end

    auto &&left_data = api_->GetStreamData(Stream::LEFT);
    auto &&right_data = api_->GetStreamData(Stream::RIGHT);
    if (left_data.frame.empty() || right_data.frame.empty()) 
    {
      return;
    }
    else
    {
      std::cout << " I have data\n";
    }
    
    
    ++left_count_;
    ++right_count_;

    if (left_count_ > 10)
    {
      ros::Time left_stamp = checkUpTimeStamp(left_data.img->timestamp, Stream::LEFT);
      //publishCamera(Stream::LEFT, left_data, left_count_, left_stamp);
    }

    if (right_count_ > 10)
    {
      ros::Time right_stamp = checkUpTimeStamp(right_data.img->timestamp, Stream::RIGHT);
      //publishCamera(Stream::RIGHT, right_data, right_count_, right_stamp);
    }

  }

  void publishCamera(
      const Stream &stream, const api::StreamData &data, std::uint32_t seq,
      ros::Time stamp) {
        
    std_msgs::Header header;
    header.seq = seq;
    header.stamp = stamp;
    header.frame_id = "world";
    
    pthread_mutex_lock(&mutex_data_);
    cv::Mat img = data.frame;
    auto &&msg =
        cv_bridge::CvImage(header, enc::MONO8, img).toImageMsg();
    pthread_mutex_unlock(&mutex_data_);

    auto &&info = getCameraInfo(stream);
    // empty for now
    //sensor_msgs::CameraInfoPtr info;
    //info.header.stamp = msg->header.stamp;
    //info.header.frame_id = "world";

    camera_publishers_[stream].publish(msg, info);
  }

 private:
  void initDevice() {
    std::shared_ptr<Device> device = nullptr;

    device = selectDevice();

    api_ = API::Create(device);
    auto &&requests = device->GetStreamRequests();
    std::size_t m = requests.size();
    int request_index = 0;

    model_ = api_->GetModel();
    if (model_ == Model::STANDARD2 ||
        model_ == Model::STANDARD210A || model_ == Model::STANDARD200B) {
      private_nh_.getParamCached("standard2/request_index", request_index);
      switch (request_index) {
        case 0:
        case 4:
          frame_rate_ = 10;
          break;
        case 1:
        case 5:
          frame_rate_ = 20;
          break;
        case 2:
        case 6:
          frame_rate_ = 30;
          break;
        case 3:
          frame_rate_ = 60;
          break;
      }
    }
    if (model_ == Model::STANDARD) {
      private_nh_.getParamCached("standard/request_index", request_index);
      frame_rate_ = api_->GetOptionValue(Option::FRAME_RATE);
    }

    // It has to do with imu.
    // Remove for now
    /*std::int32_t process_mode = 0;
    if (model_ == Model::STANDARD2 ||
        model_ == Model::STANDARD210A || model_ == Model::STANDARD200B) {
      private_nh_.getParamCached("standard2/imu_process_mode", process_mode);
      api_->EnableProcessMode(process_mode);
    }*/

    NODELET_FATAL_COND(m <= 0, "No MYNT EYE devices :(");
    if (m <= 1) {
      NODELET_INFO_STREAM("Only one stream request, select index: 0");
      api_->ConfigStreamRequest(requests[0]);
    } else {
      if (request_index >= m) {
        NODELET_WARN_STREAM("Resquest_index out of range");
        api_->ConfigStreamRequest(requests[0]);
      } else {
        api_->ConfigStreamRequest(requests[request_index]);
      }
    }

    // remove for now
    //computeRectTransforms();
  }

  std::shared_ptr<Device> selectDevice() {
    NODELET_INFO_STREAM("Detecting MYNT EYE devices");

    Context context;
    auto &&devices = context.devices();

    size_t n = devices.size();
    NODELET_FATAL_COND(n <= 0, "No MYNT EYE devices :(");

    NODELET_INFO_STREAM("MYNT EYE devices:");
    for (size_t i = 0; i < n; i++) {
      auto &&device = devices[i];
      auto &&name = device->GetInfo(Info::DEVICE_NAME);
      auto &&serial_number = device->GetInfo(Info::SERIAL_NUMBER);
      NODELET_INFO_STREAM("  index: " << i << ", name: " <<
          name << ", serial number: " << serial_number);
    }

    bool is_multiple = false;
    private_nh_.getParam("is_multiple", is_multiple);
    if (is_multiple) {
      std::string sn;
      private_nh_.getParam("serial_number", sn);
      NODELET_FATAL_COND(sn.empty(), "Must set serial_number "
          "in mynteye_1.launch and mynteye_2.launch.");

      for (size_t i = 0; i < n; i++) {
        auto &&device = devices[i];
        auto &&name = device->GetInfo(Info::DEVICE_NAME);
        auto &&serial_number = device->GetInfo(Info::SERIAL_NUMBER);
        if (sn == serial_number)
          return device;
        NODELET_FATAL_COND(i == (n - 1), "No corresponding device was found,"
            " check the serial_number configuration. ");
      }
    } else {
      if (n <= 1) {
        NODELET_INFO_STREAM("Only one MYNT EYE device, select index: 0");
        return devices[0];
      } else {
        while (true) {
          size_t i;
          NODELET_INFO_STREAM(
              "There are " << n << " MYNT EYE devices, select index: ");
          std::cin >> i;
          if (i >= n) {
            NODELET_WARN_STREAM("Index out of range :(");
            continue;
          }
          return devices[i];
        }
      }
    }

    return nullptr;
  }

  sensor_msgs::CameraInfoPtr getCameraInfo(const Stream &stream) {
    if (camera_info_ptrs_.find(stream) != camera_info_ptrs_.end()) {
      return camera_info_ptrs_[stream];
    }
    ROS_ASSERT(api_);
    // http://docs.ros.org/kinetic/api/sensor_msgs/html/msg/CameraInfo.html
    sensor_msgs::CameraInfo *camera_info = new sensor_msgs::CameraInfo();
    camera_info_ptrs_[stream] = sensor_msgs::CameraInfoPtr(camera_info);
    auto info_pair = api_->GetCameraROSMsgInfoPair();
    camera_info->width = info_pair->left.width;
    camera_info->height = info_pair->left.height;
    if (is_intrinsics_enable_) {
      if (stream == Stream::RIGHT ||
          stream == Stream::RIGHT_RECTIFIED) {
        if (info_pair->right.distortion_model == "KANNALA_BRANDT") {
          camera_info->distortion_model = "KANNALA_BRANDT";
          for (size_t i = 0; i < 5; i++) {
            camera_info->D.push_back(info_pair->right.D[i]);
          }
        } else if (info_pair->right.distortion_model == "PINHOLE") {
          camera_info->distortion_model = "plumb_bob";
          for (size_t i = 0; i < 5; i++) {
            camera_info->D.push_back(info_pair->right.D[i]);
          }
        }
        for (size_t i = 0; i < 9; i++) {
          camera_info->K.at(i) = info_pair->right.K[i];
        }
        for (size_t i = 0; i < 9; i++) {
          camera_info->R.at(i) = info_pair->right.R[i];
        }
        for (size_t i = 0; i < 12; i++) {
          camera_info->P.at(i) = info_pair->right.P[i];
        }
      } else {
        if (info_pair->left.distortion_model == "KANNALA_BRANDT") {
          // compatible laserscan
          bool is_laserscan = false;
          private_nh_.getParamCached("is_laserscan", is_laserscan);
          if (!is_laserscan) {
            camera_info->distortion_model = "KANNALA_BRANDT";
            for (size_t i = 0; i < 5; i++) {
              camera_info->D.push_back(info_pair->left.D[i]);
            }
          } else {
            camera_info->distortion_model = "KANNALA_BRANDT";
            for (size_t i = 0; i < 4; i++) {
              camera_info->D.push_back(0.0);
            }
          }
        } else if (info_pair->left.distortion_model == "PINHOLE") {
          camera_info->distortion_model = "plumb_bob";
          for (size_t i = 0; i < 5; i++) {
            camera_info->D.push_back(info_pair->left.D[i]);
          }
        }
        for (size_t i = 0; i < 9; i++) {
          camera_info->K.at(i) = info_pair->left.K[i];
        }
        for (size_t i = 0; i < 9; i++) {
          camera_info->R.at(i) = info_pair->left.R[i];
        }
        for (size_t i = 0; i < 12; i++) {
          camera_info->P.at(i) = info_pair->left.P[i];
        }
      }
    }
    return camera_info_ptrs_[stream];
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  pthread_mutex_t mutex_data_;

  Model model_;
  std::map<Option, std::string> option_names_;
  // camera:
  //   LEFT, RIGHT, LEFT_RECTIFIED, RIGHT_RECTIFIED,
  //   DISPARITY, DISPARITY_NORMALIZED,
  //   DEPTH
  std::map<Stream, image_transport::CameraPublisher> camera_publishers_;
  std::map<Stream, sensor_msgs::CameraInfoPtr> camera_info_ptrs_;
  std::map<Stream, std::string> camera_encodings_;

  // image: LEFT_RECTIFIED, RIGHT_RECTIFIED, DISPARITY, DISPARITY_NORMALIZED,
  // DEPTH
  std::map<Stream, image_transport::Publisher> image_publishers_;
  std::map<Stream, std::string> image_encodings_;

  // mono: LEFT, RIGHT
  std::map<Stream, image_transport::Publisher> mono_publishers_;

  // pointcloud: POINTS
  ros::Publisher points_publisher_;

  ros::Publisher pub_imu_;
  ros::Publisher pub_temperature_;

  ros::Publisher pub_mesh_;  // < The publisher for camera mesh.
  visualization_msgs::Marker mesh_msg_;  // < Mesh message.

  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  ros::ServiceServer get_info_service_;

  // node params

  std::string base_frame_id_;
  std::string imu_frame_id_;
  std::string temperature_frame_id_;
  std::map<Stream, std::string> frame_ids_;

  double gravity_;

  // disparity type
  DisparityComputingMethod disparity_type_;
  // api

  std::shared_ptr<API> api_;

  // rectification transforms
  cv::Mat left_r_, right_r_, left_p_, right_p_, q_;
  cv::Rect left_roi_, right_roi_;

  double time_beg_ = -1;
  double left_time_beg_ = -1;
  double right_time_beg_ = -1;
  double imu_time_beg_ = -1;
  std::size_t left_count_ = 0;
  std::size_t right_count_ = 0;
  std::size_t imu_count_ = 0;
  std::size_t imu_sync_count_ = 0;
  std::shared_ptr<ImuData> imu_accel_;
  std::shared_ptr<ImuData> imu_gyro_;
  bool publish_imu_by_sync_ = true;
  std::map<Stream, bool> is_published_;
  bool is_motion_published_;
  bool is_started_;
  int frame_rate_;
  bool is_intrinsics_enable_;
  std::vector<ImuData> imu_align_;
  int skip_tag;
  int skip_tmp_left_tag;
  int skip_tmp_right_tag;
  double mesh_rotation_x;
  double mesh_rotation_y;
  double mesh_rotation_z;
  double mesh_position_x;
  double mesh_position_y;
  double mesh_position_z;
  std::vector<int64_t> left_timestamps;
  std::vector<int64_t> right_timestamps;

  std::uint64_t unit_hard_time = std::numeric_limits<std::uint32_t>::max();
};

MYNTEYE_END_NAMESPACE

#include <pluginlib/class_list_macros.h>  // NOLINT
PLUGINLIB_EXPORT_CLASS(mynteye::ROSWrapperNodelet, nodelet::Nodelet);

/**********************************************************************************************************************
Copyright (c) 2024 Seyond
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For Seyond LiDAR SDK Library
(2-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********************************************************************************************************************/

#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>

#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "driver_lidar.h"
#include "yaml_tools.hpp"
#include "seyond/SeyondPacket.h"
#include "seyond/SeyondScan.h"
#include "seyond/TransformConfig.h"
#include "src/multi_fusion/ros1_multi_fusion.hpp"

class ROSAdapter {
 public:
  ROSAdapter(std::shared_ptr<ros::NodeHandle> nh, std::shared_ptr<ros::NodeHandle> private_nh,
             const seyond::LidarConfig &lidar_config) {
    nh_ = nh;
    private_nh_ = private_nh;
    driver_ptr_ = std::make_unique<seyond::DriverLidar>(lidar_config);
    inno_scan_msg_ = std::make_unique<seyond::SeyondScan>();
    lidar_config_ = lidar_config;
  }

  ~ROSAdapter() {
    driver_ptr_.reset();
  }

  void init() {
    inno_frame_pub_ = nh_->advertise<sensor_msgs::PointCloud2>(lidar_config_.frame_topic.c_str(), 10);
    driver_ptr_->register_publish_frame_callback(
        std::bind(&ROSAdapter::publishFrame, this, std::placeholders::_1, std::placeholders::_2));

    if (lidar_config_.packet_mode || lidar_config_.replay_rosbag) {
      inno_pkt_pub_ = nh_->advertise<seyond::SeyondScan>(lidar_config_.packet_topic.c_str(), 100);
      inno_pkt_sub_ = nh_->subscribe(lidar_config_.packet_topic.c_str(), 100, &ROSAdapter::subscribePacket, this);
      driver_ptr_->register_publish_packet_callback(std::bind(&ROSAdapter::publishPacket, this, std::placeholders::_1,
                                                              std::placeholders::_2, std::placeholders::_3,
                                                              std::placeholders::_4));
    }

    if (lidar_config_.enable_imu_msg) {
      inno_imu_pub_ = nh_->advertise<sensor_msgs::Imu>("/iv_imu", 10);
      driver_ptr_->register_publish_imu_callback(
          std::bind(&ROSAdapter::publishImu, this, std::placeholders::_1, std::placeholders::_2));
    }

    hori_roi_sub_ = nh_->subscribe(lidar_config_.lidar_name + "_hori_roi", 10, &ROSAdapter::processHoriRoi, this,
                                   ros::TransportHints().tcpNoDelay(true));
    vert_roi_sub_ = nh_->subscribe(lidar_config_.lidar_name + "_vert_roi", 10, &ROSAdapter::processVertRoi, this,
                                   ros::TransportHints().tcpNoDelay(true));
  }

  void start() {
    driver_ptr_->start_lidar();
  }

  void stop() {
    driver_ptr_->stop_lidar();
  }

  void setTransformParam(seyond::TransformParam &param) {
    if (driver_ptr_->transform_matrix_ != "") {
      ROS_WARN("%s: transform_matrix is not empty, ignore dynamic params, please clear it and restart driver",
               lidar_config_.lidar_name.c_str());
      return;
    }
    driver_ptr_->x_ = param.x;
    driver_ptr_->y_ = param.y;
    driver_ptr_->z_ = param.z;
    driver_ptr_->pitch_ = param.pitch;
    driver_ptr_->yaw_ = param.yaw;
    driver_ptr_->roll_ = param.roll;

    driver_ptr_->transform_degree_flag_ = true;
    driver_ptr_->init_transform_matrix();
  }

 private:
  void subscribePacket(const seyond::SeyondScan::ConstPtr &msg) {
    for (const auto &pkt : msg->packets) {
      if (lidar_config_.replay_rosbag && pkt.has_table && !driver_ptr_->anglehv_table_init_) {
        driver_ptr_->anglehv_table_.resize(pkt.table.size());
        std::memcpy(driver_ptr_->anglehv_table_.data(), pkt.table.data(), pkt.table.size());
        driver_ptr_->anglehv_table_init_ = true;
      }
      driver_ptr_->convert_and_parse(reinterpret_cast<const int8_t *>(pkt.data.data()));
    }

    if (msg->is_last_scan) {
      sensor_msgs::PointCloud2 ros_msg;
      driver_ptr_->transform_pointcloud();
      pcl::toROSMsg(*driver_ptr_->pcl_pc_ptr, ros_msg);
      ros_msg.header.frame_id = lidar_config_.frame_id;
      ros_msg.header.stamp = ros::Time().fromSec(msg->timestamp * 1e-6);
      ros_msg.width = driver_ptr_->pcl_pc_ptr->width;
      ros_msg.height = driver_ptr_->pcl_pc_ptr->height;
      inno_frame_pub_.publish(std::move(ros_msg));
      driver_ptr_->pcl_pc_ptr->clear();
    }
  }

  void publishPacket(const int8_t *pkt, uint64_t pkt_len, double timestamp, bool next_idx) {
    if (next_idx) {
      frame_count_++;
      inno_scan_msg_->timestamp = timestamp;
      inno_scan_msg_->size = packets_width_;
      packets_width_ = 0;
      inno_scan_msg_->is_last_scan = true;
      inno_pkt_pub_.publish(std::move(*inno_scan_msg_));
      inno_scan_msg_ = std::make_unique<seyond::SeyondScan>();
    } else if (packets_width_ >= lidar_config_.aggregate_num) {
      inno_scan_msg_->is_last_scan = false;
      inno_scan_msg_->size = packets_width_;
      packets_width_ = 0;
      inno_pkt_pub_.publish(std::move(*inno_scan_msg_));
      inno_scan_msg_ = std::make_unique<seyond::SeyondScan>();
    }
    seyond::SeyondPacket msg;
    msg.data.resize(pkt_len);
    std::memcpy(msg.data.data(), pkt, pkt_len);
    msg.has_table = false;
    if ((frame_count_ == table_send_hz_) && driver_ptr_->anglehv_table_init_) {
      frame_count_ = 0;
      msg.has_table = true;
      msg.table.resize(driver_ptr_->anglehv_table_.size());
      std::memcpy(msg.table.data(), driver_ptr_->anglehv_table_.data(), driver_ptr_->anglehv_table_.size());
    }
    packets_width_++;
    inno_scan_msg_->packets.emplace_back(msg);
  }

  void publishFrame(const pcl::PointCloud<SeyondPoint> &frame, double timestamp) {
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(frame, ros_msg);
    ros_msg.header.frame_id = lidar_config_.frame_id;
    ros_msg.header.stamp = ros::Time().fromSec(timestamp * 1e-6);
    ros_msg.width = frame.width;
    ros_msg.height = frame.height;
    inno_frame_pub_.publish(std::move(ros_msg));
  }

  void publishImu(std::vector<float> &imu_data, uint64_t imu_ts_ns) {
    sensor_msgs::Imu ros_msg;
    ros_msg.header.frame_id = lidar_config_.frame_id;
    ros_msg.header.stamp.sec = imu_ts_ns / 1000000000;
    ros_msg.header.stamp.nsec = imu_ts_ns % 1000000000;
    ros_msg.linear_acceleration.x = imu_data[0];
    ros_msg.linear_acceleration.y = imu_data[1];
    ros_msg.linear_acceleration.z = imu_data[2];
    ros_msg.angular_velocity.x = imu_data[3];
    ros_msg.angular_velocity.y = imu_data[4];
    ros_msg.angular_velocity.z = imu_data[5];
    inno_imu_pub_.publish(std::move(ros_msg));
  }

  void processHoriRoi(const std_msgs::Float64::ConstPtr &msg) {
    ROS_INFO("%s: get hori_roi %f", lidar_config_.lidar_name.c_str(), msg->data);
    double hori_roi = msg->data;
    int32_t ret = driver_ptr_->lidar_set_roi(hori_roi, std::numeric_limits<double>::max());
    if (ret != 0) {
      ROS_ERROR("%s: set hori_roi failed", lidar_config_.lidar_name.c_str());
    }
  }

  void processVertRoi(const std_msgs::Float64::ConstPtr &msg) {
    ROS_INFO("%s: get vert_roi %f", lidar_config_.lidar_name.c_str(), msg->data);
    double vert_roi = msg->data;
    int32_t ret = driver_ptr_->lidar_set_roi(std::numeric_limits<double>::max(), vert_roi);
    if (ret != 0) {
      ROS_ERROR("%s: set vert_roi failed", lidar_config_.lidar_name.c_str());
    }
  }

 private:
  seyond::LidarConfig lidar_config_;
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::NodeHandle> private_nh_;
  std::unique_ptr<seyond::DriverLidar> driver_ptr_;

  ros::Publisher inno_frame_pub_;
  ros::Publisher inno_pkt_pub_;
  ros::Publisher inno_imu_pub_;
  ros::Subscriber inno_pkt_sub_;
  ros::Subscriber hori_roi_sub_;
  ros::Subscriber vert_roi_sub_;

  std::unique_ptr<seyond::SeyondScan> inno_scan_msg_;

  uint32_t frame_count_{0};
  uint32_t table_send_hz_{20};
  uint32_t packets_width_{0};
};

class ROSNode {
 public:
  void init() {
    nh_ = std::make_shared<ros::NodeHandle>();
    private_nh_ = std::make_shared<ros::NodeHandle>("~");
    std::string yaml_file;
    private_nh_->param("config_path", yaml_file, std::string(""));
    if (!yaml_file.empty()) {
      int32_t ret = seyond::YamlTools::parseConfig(yaml_file, lidar_configs_, common_config_);
      if (ret != 0) {
        ROS_ERROR("Parse config file failed");
        exit(0);
      }
    } else {
      parseParams();
    }

    lidar_num_ = lidar_configs_.size();
    ros_adapters_.resize(lidar_num_);

    seyond::DriverLidar::init_log_s(common_config_.log_level, &ROSNode::rosLogCallback);
    seyond::YamlTools::printConfig(lidar_configs_);

    bool need_transform = false;

    for (int32_t i = 0; i < lidar_num_; i++) {
      lidar_configs_[i].index = i;
      ros_adapters_[i] = std::make_unique<ROSAdapter>(nh_, private_nh_, lidar_configs_[i]);
      ros_adapters_[i]->init();
      need_transform = need_transform || lidar_configs_[i].transform_enable;
    }

    if (need_transform) {
      init_dynamic_ignore_ = true;
      initDynamicParams();
    }

    if (common_config_.fusion_enable) {
      fusion_ = std::make_unique<seyond::MultiFusion>(nh_, lidar_configs_, common_config_);
    }
  }

  void start() {
    for (int32_t i = 0; i < lidar_num_; i++) {
      ros_adapters_[i]->start();
    }
  }

  void parseParams() {
    seyond::LidarConfig lidar_config;
    // common
    private_nh_->param("log_level", common_config_.log_level, std::string("info"));
    common_config_.fusion_enable = false;

    // Parse parameters for ros
    private_nh_->param("replay_rosbag", lidar_config.replay_rosbag, false);
    private_nh_->param("packet_mode", lidar_config.packet_mode, false);
    private_nh_->param("aggregate_num", lidar_config.aggregate_num, 20);
    private_nh_->param("frame_id", lidar_config.frame_id, std::string("seyond"));
    private_nh_->param("frame_topic", lidar_config.frame_topic, std::string("iv_points"));
    private_nh_->param("packet_topic", lidar_config.packet_topic, std::string("iv_packets"));

    // Parse parameters for driver
    private_nh_->param("lidar_name", lidar_config.lidar_name, std::string("seyond"));
    private_nh_->param("lidar_ip", lidar_config.lidar_ip, std::string("172.168.1.10"));
    private_nh_->param("port", lidar_config.port, 8010);
    private_nh_->param("udp_port", lidar_config.udp_port, 8010);
    private_nh_->param("reflectance_mode", lidar_config.reflectance_mode, true);
    private_nh_->param("multiple_return", lidar_config.multiple_return, 1);
    private_nh_->param("enable_falcon_ring", lidar_config.enable_falcon_ring, false);
    private_nh_->param("enable_imu_msg", lidar_config.enable_imu_msg, false);

    private_nh_->param("continue_live", lidar_config.continue_live, false);

    private_nh_->param("inno_pc_file", lidar_config.inno_pc_file, std::string(""));
    private_nh_->param("pcap_file", lidar_config.pcap_file, std::string(""));
    private_nh_->param("hv_table_file", lidar_config.hv_table_file, std::string(""));
    private_nh_->param("packet_rate", lidar_config.packet_rate, 10000);
    private_nh_->param("file_rewind", lidar_config.file_rewind, 0);

    private_nh_->param("max_range", lidar_config.max_range, 2000.0);  // unit: meter
    private_nh_->param("min_range", lidar_config.min_range, 0.4);     // unit: meter
    private_nh_->param("name_value_pairs", lidar_config.name_value_pairs, std::string(""));
    private_nh_->param("coordinate_mode", lidar_config.coordinate_mode, 3);

    private_nh_->param("transform_enable", lidar_config.transform_enable, false);
    private_nh_->param("x", lidar_config.x, 0.0);
    private_nh_->param("y", lidar_config.y, 0.0);
    private_nh_->param("z", lidar_config.z, 0.0);
    private_nh_->param("pitch", lidar_config.pitch, 0.0);
    private_nh_->param("yaw", lidar_config.yaw, 0.0);
    private_nh_->param("roll", lidar_config.roll, 0.0);
    private_nh_->param("transform_matrix", lidar_config.transform_matrix, std::string(""));

    lidar_configs_.emplace_back(lidar_config);
  }

  void initDynamicParams() {
    dynamic_server_ = std::make_shared<dynamic_reconfigure::Server<seyond::TransformConfig>>(*private_nh_);
    seyond::TransformConfig initial_config = {};
    for (int32_t i = 0; i < lidar_num_; i++) {
      if (i == 0) {
        initial_config.lidar1_x = lidar_configs_[i].x;
        initial_config.lidar1_y = lidar_configs_[i].y;
        initial_config.lidar1_z = lidar_configs_[i].z;
        initial_config.lidar1_pitch = seyond::YamlTools::RadianToDegree(lidar_configs_[i].pitch);
        initial_config.lidar1_yaw = seyond::YamlTools::RadianToDegree(lidar_configs_[i].yaw);
        initial_config.lidar1_roll = seyond::YamlTools::RadianToDegree(lidar_configs_[i].roll);
      } else if (i == 1) {
        initial_config.lidar2_x = lidar_configs_[i].x;
        initial_config.lidar2_y = lidar_configs_[i].y;
        initial_config.lidar2_z = lidar_configs_[i].z;
        initial_config.lidar2_pitch = seyond::YamlTools::RadianToDegree(lidar_configs_[i].pitch);
        initial_config.lidar2_yaw = seyond::YamlTools::RadianToDegree(lidar_configs_[i].yaw);
        initial_config.lidar2_roll = seyond::YamlTools::RadianToDegree(lidar_configs_[i].roll);
      } else if (i == 2) {
        initial_config.lidar3_x = lidar_configs_[i].x;
        initial_config.lidar3_y = lidar_configs_[i].y;
        initial_config.lidar3_z = lidar_configs_[i].z;
        initial_config.lidar3_pitch = seyond::YamlTools::RadianToDegree(lidar_configs_[i].pitch);
        initial_config.lidar3_yaw = seyond::YamlTools::RadianToDegree(lidar_configs_[i].yaw);
        initial_config.lidar3_roll = seyond::YamlTools::RadianToDegree(lidar_configs_[i].roll);
      } else if (i == 3) {
        initial_config.lidar4_x = lidar_configs_[i].x;
        initial_config.lidar4_y = lidar_configs_[i].y;
        initial_config.lidar4_z = lidar_configs_[i].z;
        initial_config.lidar4_pitch = seyond::YamlTools::RadianToDegree(lidar_configs_[i].pitch);
        initial_config.lidar4_yaw = seyond::YamlTools::RadianToDegree(lidar_configs_[i].yaw);
        initial_config.lidar4_roll = seyond::YamlTools::RadianToDegree(lidar_configs_[i].roll);
      } else if (i == 4) {
        initial_config.lidar5_x = lidar_configs_[i].x;
        initial_config.lidar5_y = lidar_configs_[i].y;
        initial_config.lidar5_z = lidar_configs_[i].z;
        initial_config.lidar5_pitch = seyond::YamlTools::RadianToDegree(lidar_configs_[i].pitch);
        initial_config.lidar5_yaw = seyond::YamlTools::RadianToDegree(lidar_configs_[i].yaw);
        initial_config.lidar5_roll = seyond::YamlTools::RadianToDegree(lidar_configs_[i].roll);
      }
    }
    dynamic_server_->updateConfig(initial_config);
    dynamic_reconfigure::Server<seyond::TransformConfig>::CallbackType dynamic_callback =
        boost::bind(&ROSNode::transformCallback, this, _1, _2);
    dynamic_server_->setCallback(dynamic_callback);
  }

  void transformCallback(const seyond::TransformConfig& config, uint32_t level) {
    if (init_dynamic_ignore_) {
      // ignore the setting when init
      init_dynamic_ignore_ = false;
      return;
    }
    std::vector<seyond::TransformParam> transform_configs = {
        {config.lidar1_x, config.lidar1_y, config.lidar1_z, config.lidar1_pitch, config.lidar1_yaw, config.lidar1_roll},
        {config.lidar2_x, config.lidar2_y, config.lidar2_z, config.lidar2_pitch, config.lidar2_yaw, config.lidar2_roll},
        {config.lidar3_x, config.lidar3_y, config.lidar3_z, config.lidar3_pitch, config.lidar3_yaw, config.lidar3_roll},
        {config.lidar4_x, config.lidar4_y, config.lidar4_z, config.lidar4_pitch, config.lidar4_yaw, config.lidar4_roll},
        {config.lidar5_x, config.lidar5_y, config.lidar5_z, config.lidar5_pitch, config.lidar5_yaw, config.lidar5_roll},
    };

    int32_t max_lidar_num = 5;

    if (lidar_num_ < max_lidar_num) {
      max_lidar_num = lidar_num_;
    }

    for (int32_t i = 0; i < max_lidar_num; i++) {
      if (lidar_configs_[i].transform_enable) {
        ros_adapters_[i]->setTransformParam(transform_configs[i]);
      }
    }
  }

  void spin() {
    ros::spin();
  }

  static void rosLogCallback(int32_t level, const char *header2, const char *msg) {
    switch (level) {
      case 0:  // INNO_LOG_LEVEL_FATAL
      case 1:  // INNO_LOG_LEVEL_CRITICAL
        ROS_FATAL("%s %s", header2, msg);
        break;
      case 2:  // INNO_LOG_LEVEL_ERROR
      case 3:  // INNO_LOG_LEVEL_TEMP
        ROS_ERROR("%s %s", header2, msg);
        break;
      case 4:  // INNO_LOG_LEVEL_WARNING
      case 5:  // INNO_LOG_LEVEL_DEBUG
        ROS_WARN("%s %s", header2, msg);
        break;
      case 6:  // INNO_LOG_LEVEL_INFO
        ROS_INFO("%s %s", header2, msg);
        break;
      case 7:  // INNO_LOG_LEVEL_TRACE
      case 8:  // INNO_LOG_LEVEL_DETAIL
      default:
        ROS_DEBUG("%s %s", header2, msg);
    }
  }

 private:
  int32_t lidar_num_;
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<ros::NodeHandle> private_nh_;
  std::shared_ptr<dynamic_reconfigure::Server<seyond::TransformConfig>> dynamic_server_;
  bool init_dynamic_ignore_;

  seyond::CommonConfig common_config_;
  std::vector<seyond::LidarConfig> lidar_configs_;
  std::vector<std::unique_ptr<ROSAdapter>> ros_adapters_;
  std::unique_ptr<seyond::MultiFusion> fusion_;
};

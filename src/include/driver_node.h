//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef DRIVER_NODE_H_
#define DRIVER_NODE_H_

#include <memory>
#include <future>
#include <thread>

#include "include/ros_headers.h"
#include "lddc.h"
#include "std_srvs/SetBool.h"
#include "livox_lidar_def.h"

namespace livox_ros {

class DriverNode {
 public:
#ifdef BUILDING_ROS1
  DriverNode() {
    ros::NodeHandle nh;
    lidar_control_service_ = nh.advertiseService("livox_control", 
        &DriverNode::handleLidarControl, this);
  }
#elif defined BUILDING_ROS2
  explicit DriverNode(const rclcpp::NodeOptions & node_options);
#endif

  void SetLidarHandle(uint32_t handle) {
    current_handle_ = handle;
  }

  void getParam(const std::string& name, std::string& value) {
#ifdef BUILDING_ROS1
    ros::NodeHandle nh;
    nh.getParam(name, value);
#elif defined BUILDING_ROS2
    this->get_parameter(name, value);
#endif
  }

  ros::NodeHandle& GetNode() { return nh_; }

  std::promise<void> exit_signal_;
  std::future<void> future_;
  std::unique_ptr<Lddc> lddc_ptr_;
  std::shared_ptr<std::thread> pointclouddata_poll_thread_;
  std::shared_ptr<std::thread> imudata_poll_thread_;

  void PointCloudDataPollThread();
  void ImuDataPollThread();

 private:
  bool handleLidarControl(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    livox_status status;
    if (req.data) {
      // Turn on - Normal mode
      status = SetLivoxLidarWorkMode(current_handle_, kLivoxLidarNormal, nullptr, nullptr);
      if (status == kLivoxLidarStatusSuccess) {
        res.message = "Lidar turned on successfully";
        res.success = true;
      } else {
        res.message = "Failed to turn on lidar";
        res.success = false;
      }
    } else {
      // Turn off - WakeUp mode
      status = SetLivoxLidarWorkMode(current_handle_, kLivoxLidarWakeUp, nullptr, nullptr);
      if (status == kLivoxLidarStatusSuccess) {
        res.message = "Lidar turned off successfully";
        res.success = true;
      } else {
        res.message = "Failed to turn off lidar";
        res.success = false;
      }
    }
    return true;
  }

  ros::NodeHandle nh_;
  ros::ServiceServer lidar_control_service_;
  uint32_t current_handle_;
};

} // namespace livox_ros

#endif // DRIVER_NODE_H_ 
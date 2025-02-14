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

#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"
#include "call_back/livox_lidar_callback.h"
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"

// Include Livox SDK headers
extern "C" {
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"
}

namespace livox_ros {

DriverNode& DriverNode::GetNode() noexcept {
  return *this;
}

DriverNode::~DriverNode() {
  lddc_ptr_->lds_->RequestExit();
  exit_signal_.set_value();
  pointclouddata_poll_thread_->join();
  imudata_poll_thread_->join();
}

#ifdef BUILDING_ROS1

// ROS1 implementation
bool DriverNode::handleSetWorkMode(
    livox_ros_driver2::LidarSetWorkMode::Request &request,
    livox_ros_driver2::LidarSetWorkMode::Response &response) {
  
  ROS_INFO("Service callback triggered");  // Debug log
  
  if (lddc_ptr_ == nullptr) {
    ROS_ERROR("lddc_ptr_ is null");
    response.success = false;
    response.message = "Driver not initialized properly (lddc_ptr_ is null)";
    return true;  // Return true to send the response
  }

  if (lddc_ptr_->GetLds() == nullptr) {
    ROS_ERROR("GetLds() returned null");
    response.success = false;
    response.message = "Driver not initialized properly (LDS is null)";
    return true;  // Return true to send the response
  }

  // Get the first connected Lidar device
  LdsLidar* lds_lidar = lddc_ptr_->GetLds();
  ROS_INFO("Got LDS instance");  // Debug log
  
  // Check if any Lidar is connected by checking the first device's connection state
  if (lds_lidar->lidars_[0].handle == 0 || 
      lds_lidar->lidars_[0].connect_state == kConnectStateOff) {
    ROS_ERROR("No Lidar connected (handle: %u, state: %d)", 
              lds_lidar->lidars_[0].handle,
              lds_lidar->lidars_[0].connect_state);
    response.success = false;
    response.message = "No Lidar devices connected";
    return true;  // Return true to send the response
  }

  LidarDevice* lidar_device = &(lds_lidar->lidars_[0]);  // Get first Lidar
  uint32_t handle = lidar_device->handle;
  ROS_INFO("Found Lidar device with handle: %u", handle);  // Debug log

  LivoxLidarWorkMode work_mode = static_cast<LivoxLidarWorkMode>(
      request.work_mode == 0 ? kLivoxLidarNormal : kLivoxLidarWakeUp);

  ROS_INFO("Attempting to set work mode: mode=%s", 
           (work_mode == kLivoxLidarNormal ? "Normal" : "WakeUp"));

  bool success = LivoxLidarCallback::SetLidarWorkMode(
      handle, work_mode, lddc_ptr_->GetLds());

  response.success = success;
  response.message = success ? 
      "Successfully set work mode" : 
      "Failed to set work mode";

  ROS_INFO("Set Lidar work mode request: mode=%s, result=%s",
      (work_mode == kLivoxLidarNormal ? "Normal" : "WakeUp"),
      (success ? "Success" : "Failed"));
      
  return true;  // Always return true to send the response
}

DriverNode::DriverNode() : ros::NodeHandle() {
  ROS_INFO("Initializing DriverNode...");
  
  // Initialize service server
  set_work_mode_srv_ = advertiseService("/livox_lidar_set_mode",  // Use absolute path 
                                      &DriverNode::handleSetWorkMode, 
                                      this);
  
  if (set_work_mode_srv_) {
    ROS_INFO("Service /livox_lidar_set_mode registered successfully");
  } else {
    ROS_ERROR("Failed to register service /livox_lidar_set_mode");
  }
}

#elif defined BUILDING_ROS2

// ROS2 implementation
DriverNode::DriverNode(const rclcpp::NodeOptions& options) 
    : Node("livox_lidar_publisher", options) {
  // ... existing initialization code ...

  // Add service server
  set_work_mode_srv_ = this->create_service<livox_interfaces2::srv::LidarSetWorkMode>(
      "livox_lidar_set_mode",
      std::bind(&DriverNode::handleSetWorkMode, this, 
                std::placeholders::_1, std::placeholders::_2));
}

void DriverNode::handleSetWorkMode(
    const std::shared_ptr<livox_interfaces2::srv::LidarSetWorkMode::Request> request,
    std::shared_ptr<livox_interfaces2::srv::LidarSetWorkMode::Response> response) {
  
  LivoxLidarWorkMode work_mode = static_cast<LivoxLidarWorkMode>(
      request->work_mode == 0 ? kLivoxLidarNormal : kLivoxLidarWakeUp);

  bool success = LivoxLidarCallback::SetLidarWorkMode(
      request->handle, work_mode, lddc_ptr_->GetLds());

  response->success = success;
  response->message = success ? 
      "Successfully set work mode" : 
      "Failed to set work mode";

  RCLCPP_INFO(this->get_logger(), 
      "Set Lidar work mode request: handle=%u, mode=%s, result=%s",
      request->handle,
      (work_mode == kLivoxLidarNormal ? "Normal" : "WakeUp"),
      (success ? "Success" : "Failed"));
}

#endif

} // namespace livox_ros






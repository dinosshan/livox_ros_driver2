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
#include "call_back/livox_lidar_callback.h"
#include "livox_lidar_def.h"

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
  
  LivoxLidarWorkMode work_mode = (request.work_mode == 0) ? 
      kLivoxLidarNormal : kLivoxLidarWakeUp;

  bool success = LivoxLidarCallback::SetLidarWorkMode(
      request.handle, work_mode, lddc_ptr_->GetLds());

  response.success = success;
  response.message = success ? 
      "Successfully set work mode" : 
      "Failed to set work mode";

  ROS_INFO("Set Lidar work mode request: handle=%u, mode=%s, result=%s",
      request.handle,
      (work_mode == kLivoxLidarNormal ? "Normal" : "WakeUp"),
      (success ? "Success" : "Failed"));
      
  return success;
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
  
  LivoxLidarWorkMode work_mode = (request->work_mode == 0) ? 
      kLivoxLidarNormal : kLivoxLidarWakeUp;

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






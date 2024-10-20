// Copyright (c) 2024 Stogl Robotics Consulting UG (haftungsbeschränkt)
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

#ifndef MOTOROS2_HW_INTERFACES__MOTOROS2_HARDWARE_INTERFACE_HPP_
#define MOTOROS2_HW_INTERFACES__MOTOROS2_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motoros2_hw_interfaces/moving_avg.hpp"
#include "motoros2_hw_interfaces/visibility_control.h"
#include "motoros2_interfaces/srv/queue_traj_point.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace motoros2_hw_interfaces
{

class MotoROS2HardwareInterface : public hardware_interface::SystemInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  ~MotoROS2HardwareInterface()
  {
    if (joint_state_thread_.joinable())
    {
      stop_joint_state_thread_ = true;
      joint_state_thread_.join();
    }
  }

private:
  void joint_state_thread_func();

  // Helper function to check if two arrays are near each other
  bool are_arrays_near(
    const std::vector<double> & a, const std::vector<double> & b, double tolerance)
  {
    if (a.size() != b.size())
    {
      return false;
    }
    for (size_t i = 0; i < a.size(); ++i)
    {
      if (std::abs(a[i] - b[i]) > tolerance)
      {
        return false;
      }
    }
    return true;
  }

  void publish_write_stats()
  {
    uint64_t total_failed = write_motoros2_failed_ + write_futures_failed_;
    double failed_percentage =
      static_cast<double>(total_failed) * 100.0 / static_cast<double>(write_requests_total_);
    double write_futures_percentage = static_cast<double>(write_futures_failed_) * 100.0 /
                                      static_cast<double>(write_requests_total_);
    double write_motoros2_percentage = static_cast<double>(write_motoros2_failed_) * 100.0 /
                                       static_cast<double>(write_requests_total_);
    write_stats_msg_.data =
      "Total: " + std::to_string(write_requests_total_) +
      ", success: " + std::to_string(write_requests_total_ - total_failed) +
      ", failed: " + std::to_string(total_failed) + " ( " + std::to_string(failed_percentage) +
      "%, futures: " + std::to_string(write_futures_failed_) + " with " +
      std::to_string(write_futures_percentage) + "%, and " +
      " motoros2: " + std::to_string(write_motoros2_failed_) + " with " +
      std::to_string(write_motoros2_percentage) + "%)" +
      "; exec: " + std::to_string(moving_avg_write_exec_time_ms_.get_moving_avg()) + " ms";
    if (realtime_write_stats_pub_->trylock())
    {
      realtime_write_stats_pub_->msg_ = write_stats_msg_;
      realtime_write_stats_pub_->unlockAndPublish();
    }
  }

  double throttle_duration_ms_;
  bool commands_initialized_ = false;

  std::thread joint_state_thread_;
  std::atomic<bool> stop_joint_state_thread_{false};

  rclcpp::executors::SingleThreadedExecutor executor_;
  std::shared_ptr<rclcpp::Node> node_;
  std::vector<std::string> joint_names_;
  trajectory_msgs::msg::JointTrajectoryPoint state_current_;
  trajectory_msgs::msg::JointTrajectoryPoint command_current_;
  size_t num_joints_;

  std::string queue_traj_point_srv_name_;
  std::string joint_state_topic_name_;
  rclcpp::Client<motoros2_interfaces::srv::QueueTrajPoint>::SharedPtr queue_traj_point_client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<sensor_msgs::msg::JointState>> joint_state_buffer_;
  std::shared_ptr<motoros2_interfaces::srv::QueueTrajPoint_Request> request_;
  std::shared_ptr<rclcpp::Duration> time_from_start_;

  MovingAvg moving_avg_write_exec_time_ms_;
  uint64_t write_requests_total_;
  uint64_t write_futures_failed_;
  uint64_t write_motoros2_failed_;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> write_stats_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::String>>
    realtime_write_stats_pub_;
  std_msgs::msg::String write_stats_msg_;

  rclcpp::Time time_from_last_write_sec_;
};

}  // namespace motoros2_hw_interfaces

#endif  // MOTOROS2_HW_INTERFACES__MOTOROS2_HARDWARE_INTERFACE_HPP_

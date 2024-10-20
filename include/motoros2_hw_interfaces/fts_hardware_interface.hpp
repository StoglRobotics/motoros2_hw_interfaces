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

#ifndef MOTOROS2_HW_INTERFACES__FTS_HARDWARE_INTERFACE_HPP_
#define MOTOROS2_HW_INTERFACES__FTS_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motoros2_hw_interfaces/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace motoros2_hw_interfaces
{
class FTSHardwareInterface : public hardware_interface::SensorInterface
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

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  ~FTSHardwareInterface()
  {
    // Ensure the thread is properly joined on destruction
    if (spin_thread_.joinable())
    {
      rclcpp::shutdown();
      spin_thread_.join();
    }
  }

private:
  void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);

  double throttle_duration_ms_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription_;
  std::string fts_topic_;

  double force_x_;
  double force_y_;
  double force_z_;
  double torque_x_;
  double torque_y_;
  double torque_z_;

  std::string force_x_name_;
  std::string force_y_name_;
  std::string force_z_name_;
  std::string torque_x_name_;
  std::string torque_y_name_;
  std::string torque_z_name_;

  std::mutex data_mutex_;
  std::thread spin_thread_;
};

}  // namespace motoros2_hw_interfaces

#endif  // MOTOROS2_HW_INTERFACES__FTS_HARDWARE_INTERFACE_HPP_

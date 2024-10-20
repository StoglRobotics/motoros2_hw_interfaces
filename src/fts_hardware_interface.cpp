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

#include <limits>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "motoros2_hw_interfaces/fts_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motoros2_hw_interfaces
{

constexpr char FORCE_X_NAME_KEY[] = "force_x_name";
constexpr char FORCE_Y_NAME_KEY[] = "force_y_name";
constexpr char FORCE_Z_NAME_KEY[] = "force_z_name";
constexpr char TORQUE_X_NAME_KEY[] = "torque_x_name";
constexpr char TORQUE_Y_NAME_KEY[] = "torque_y_name";
constexpr char TORQUE_Z_NAME_KEY[] = "torque_z_name";

constexpr char NODE_NAME[] = "fts_hardware_interface";
constexpr char FTS_TOPIC_KEY[] = "fts_topic";
constexpr char THROTTLE_DURATION_KEY[] = "throttle_duration_ms";
const double DEFAULT_THROTTLE_DURATION_MS = 5000.0;

hardware_interface::CallbackReturn FTSHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Initialize variables for storing the sensor data
  force_x_ = 0.0;
  force_y_ = 0.0;
  force_z_ = 0.0;
  torque_x_ = 0.0;
  torque_y_ = 0.0;
  torque_z_ = 0.0;

  node_ = std::make_shared<rclcpp::Node>(NODE_NAME);

  // get parameters
  if (info_.hardware_parameters.find(FTS_TOPIC_KEY) == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(
      node_->get_logger(), "Parameter '%s' not found in hardware parameters.", FTS_TOPIC_KEY);
    return CallbackReturn::ERROR;
  }
  fts_topic_ = info_.hardware_parameters[FTS_TOPIC_KEY];

  if (info_.hardware_parameters.find(FORCE_X_NAME_KEY) == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(
      node_->get_logger(), "Parameter '%s' not found in hardware parameters.", FORCE_X_NAME_KEY);
    return CallbackReturn::ERROR;
  }
  force_x_name_ = info_.hardware_parameters[FORCE_X_NAME_KEY];

  if (info_.hardware_parameters.find(FORCE_Y_NAME_KEY) == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(
      node_->get_logger(), "Parameter '%s' not found in hardware parameters.", FORCE_Y_NAME_KEY);
    return CallbackReturn::ERROR;
  }
  force_y_name_ = info_.hardware_parameters[FORCE_Y_NAME_KEY];

  if (info_.hardware_parameters.find(FORCE_Z_NAME_KEY) == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(
      node_->get_logger(), "Parameter '%s' not found in hardware parameters.", FORCE_Z_NAME_KEY);
    return CallbackReturn::ERROR;
  }
  force_z_name_ = info_.hardware_parameters[FORCE_Z_NAME_KEY];

  if (info_.hardware_parameters.find(TORQUE_X_NAME_KEY) == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(
      node_->get_logger(), "Parameter '%s' not found in hardware parameters.", TORQUE_X_NAME_KEY);
    return CallbackReturn::ERROR;
  }
  torque_x_name_ = info_.hardware_parameters[TORQUE_X_NAME_KEY];

  if (info_.hardware_parameters.find(TORQUE_Y_NAME_KEY) == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(
      node_->get_logger(), "Parameter '%s' not found in hardware parameters.", TORQUE_Y_NAME_KEY);
    return CallbackReturn::ERROR;
  }
  torque_y_name_ = info_.hardware_parameters[TORQUE_Y_NAME_KEY];

  if (info_.hardware_parameters.find(TORQUE_Z_NAME_KEY) == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(
      node_->get_logger(), "Parameter '%s' not found in hardware parameters.", TORQUE_Z_NAME_KEY);
    return CallbackReturn::ERROR;
  }
  torque_z_name_ = info_.hardware_parameters[TORQUE_Z_NAME_KEY];

  if (info_.hardware_parameters.find(THROTTLE_DURATION_KEY) == info_.hardware_parameters.end())
  {
    throttle_duration_ms_ = DEFAULT_THROTTLE_DURATION_MS;
    RCLCPP_WARN(
      node_->get_logger(),
      "Parameter '%s' not found in hardware parameters. Using default value: %f.",
      THROTTLE_DURATION_KEY, DEFAULT_THROTTLE_DURATION_MS);
  }

  // Subscribe to the /tcp_force_torque topic
  RCLCPP_INFO(node_->get_logger(), "Subscribing to topic: %s", fts_topic_.c_str());
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();
  subscription_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
    fts_topic_, subscribers_qos,
    std::bind(&FTSHardwareInterface::wrench_callback, this, std::placeholders::_1));

  // Start the spin thread to process incoming messages
  spin_thread_ = std::thread([this]() { rclcpp::spin(node_); });

  return CallbackReturn::SUCCESS;
}

void FTSHardwareInterface::wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
{
  // Update the force and torque variables with new data from the message
  std::lock_guard<std::mutex> lock(data_mutex_);
  force_x_ = msg->wrench.force.x;
  force_y_ = msg->wrench.force.y;
  force_z_ = msg->wrench.force.z;
  torque_x_ = msg->wrench.torque.x;
  torque_y_ = msg->wrench.torque.y;
  torque_z_ = msg->wrench.torque.z;
}

hardware_interface::CallbackReturn FTSHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FTSHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Register state interfaces for force and torque measurements
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(info_.sensors[0].name, force_x_name_, &force_x_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(info_.sensors[0].name, force_y_name_, &force_y_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(info_.sensors[0].name, force_z_name_, &force_z_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(info_.sensors[0].name, torque_x_name_, &torque_x_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(info_.sensors[0].name, torque_y_name_, &torque_y_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(info_.sensors[0].name, torque_z_name_, &torque_z_));

  return state_interfaces;
}

hardware_interface::CallbackReturn FTSHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FTSHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type FTSHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // No need to read anything here, the data is already updated in the callback
  return hardware_interface::return_type::OK;
}

}  // namespace motoros2_hw_interfaces

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motoros2_hw_interfaces::FTSHardwareInterface, hardware_interface::SensorInterface)

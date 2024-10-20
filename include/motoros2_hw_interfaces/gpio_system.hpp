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

#ifndef MOTOROS2_HW_INTERFACES__GPIO_SYSTEM_HPP_
#define MOTOROS2_HW_INTERFACES__GPIO_SYSTEM_HPP_

#include <optional>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "motoros2_hw_interfaces/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "motoros2_interfaces/srv/read_group_io.hpp"
#include "motoros2_interfaces/srv/read_single_io.hpp"
#include "motoros2_interfaces/srv/write_group_io.hpp"
#include "motoros2_interfaces/srv/write_single_io.hpp"

namespace motoros2_hw_interfaces
{
class GPIOSystem : public hardware_interface::SystemInterface
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

private:
  int get_group_io_address(int address, int group_multiplier, int bit_index)
  {
    return address * group_multiplier + bit_index;
  }

  std::vector<int> get_group_io_addresses(int address, int group_multiplier, int group_num_bits)
  {
    std::vector<int> addresses;
    for (int i = 0; i < group_num_bits; i++)
    {
      addresses.push_back(get_group_io_address(address, group_multiplier, i));
    }
    return addresses;
  }

  enum class IOType
  {
    SINGLE,
    GROUP
  };

  struct InterfaceEntry
  {
    std::string name;
    std::optional<int> address;
    std::optional<int> bit_index;
    double value;
  };

  struct GPIOEntry
  {
    std::string name;
    IOType io_type;
    std::optional<int> address;
    std::vector<InterfaceEntry> command_interface_entries;
    std::vector<InterfaceEntry> state_interfaces_entries;
  };

  std::vector<GPIOEntry> gpio_entries_;

  // read/write services
  std::string read_single_io_srv_name_;
  std::string write_single_io_srv_name_;
  std::string read_group_io_srv_name_;
  std::string write_group_io_srv_name_;

  double update_rate_;
  int future_timeout_ms_;
  rclcpp::Time last_read_time_;
  rclcpp::Time last_write_time_;

  rclcpp::Node::SharedPtr node_;

  rclcpp::Client<motoros2_interfaces::srv::ReadSingleIO>::SharedPtr read_single_io_client_;
  rclcpp::Client<motoros2_interfaces::srv::WriteSingleIO>::SharedPtr write_single_io_client_;
  rclcpp::Client<motoros2_interfaces::srv::ReadGroupIO>::SharedPtr read_group_io_client_;
  rclcpp::Client<motoros2_interfaces::srv::WriteGroupIO>::SharedPtr write_group_io_client_;

  double throttle_duration_ms_;
};

}  // namespace motoros2_hw_interfaces

#endif  // MOTOROS2_HW_INTERFACES__GPIO_SYSTEM_HPP_

// Example of a system configuration file:
// <ros2_control name="GPIOSystem2dof" type="system" is_async="true">
//   <hardware>
//     <plugin>motoros2_hw_interfaces/GPIOSystem</plugin>
//     <param name="read_single_io_srv">/read_single_io</param>
//     <param name="write_single_io_srv">/write_single_io</param>
//     <param name="read_group_io_srv">/read_group_io</param>
//     <param name="write_group_io_srv">/write_group_io</param>
//   </hardware>
//   <gpio name="gpio_1_single_io_type">
//     <param name="io_type">single</param>
//     <command_interface name="command_1">
//       <param name="address">1001</param>
//     </command_interface>
//     <state_interface name="state_1">
//       <param name="address">1001</param>
//     </state_interface>
//   </gpio>
//   <gpio name="gpio_2_group_io_type">
//     <param name="io_type">group</param>
//     <param name="address">1002</param>
//     <command_interface name="command_2">
//       <param name="bit_index">1</param>
//     </command_interface>
//     <state_interface name="state_2">
//       <param name="bit_index">1</param>
//     </state_interface>
//   </gpio>
// </ros2_control>

// hardware_interface::CallbackReturn GPIOSystem::on_init(
//   const hardware_interface::HardwareInfo & info)
// {
// std::unordered_map<uint32_t, double> single_states_storage_map_;
// std::unordered_map<uint32_t, std::array<double, 8>> group_states_storage_map_;

// std::vector<std::pair<uint32_t, double>> single_states_storage_;
// std::vector<std::pair<uint32_t, std::array<double, 8>>> group_states_storage_;
// if (address not in group_states_storage_)
// {
//   group_states_storage_.push_back(
//     std::pair<uint32_t, std::array<double, 8>>(address, std::array<double, 8>()));
// }

// for (const auto state_interface : gpio.state_interfaces)
// {
//   const auto bit_index = std::stoi(state_interface.parameters.at(BIT_INDEX_KEY));
//   group_states_storage_[address].second[bit_index] = std::numeric_limits<double>::quiet_NaN();
// }

// hardware_interface::return_type GPIOSystem::read(
//   const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
// {
//   std::vector<std::pair<uint32_t, std::array<double, 8>>> group_states_storage_;

//   for (auto state_storage : group_states_storage_)
//   {
//     // call read service for the address  state_storage.first();
//     response = service_call(state_storage.first());

//     for (int i = 0; i < 8; i++)
//     {
//       read_io_values[addr] = (read_value >> (addr - gpio_entries_[i].address.value())) & 1;
//     }
//   }

//   std::vector<std::pair<uint32_t, double>> single_states_storage_;

//   for (auto state_storage : single_states_storage_)
//   {
//     // call read service for the address  state_storage.first();
//     response = service_call_read_single(state_storage.first());

//     state_storage.second = response.value;
//   }

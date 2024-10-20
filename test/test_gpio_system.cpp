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

#include <gmock/gmock.h>

#include <string>

#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/node.hpp>
#include <ros2_control_test_assets/components_urdfs.hpp>
#include <ros2_control_test_assets/descriptions.hpp>

class TestGPIOSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    gpio_system_ =
      R"(
        <ros2_control name="gpio_system" type="system" is_async="true">
          <hardware>
            <plugin>motoros2_hw_interfaces/GPIOSystem</plugin>
            <param name="read_single_io_srv">/read_single_io</param>
            <param name="write_single_io_srv">/write_single_io</param>
            <param name="read_group_io_srv">/read_group_io</param>
            <param name="write_group_io_srv">/write_group_io</param>
          </hardware>
          <gpio name="gpio_1_single_io_type">
            <param name="io_type">single</param>
            <command_interface name="command_1">
              <param name="address">1001</param>
            </command_interface>
            <state_interface name="state_1">
              <param name="address">1001</param>
            </state_interface>
          </gpio>
          <gpio name="gpio_2_group_io_type">
            <param name="io_type">group</param>
            <param name="address">1002</param>
            <command_interface name="command_2">
              <param name="bit_index">1</param>
            </command_interface>
            <state_interface name="state_2">
              <param name="bit_index">1</param>
            </state_interface>
          </gpio>
        </ros2_control>
    )";
  }

  std::string gpio_system_;
  rclcpp::Node node_ = rclcpp::Node("test_gpio_system");
};

TEST_F(TestGPIOSystem, load_gpio_system)
{
  auto urdf =
    ros2_control_test_assets::urdf_head + gpio_system_ + ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(
    urdf, node_.get_node_clock_interface(), node_.get_node_logging_interface()));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

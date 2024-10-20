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

class TestFTSHardwareInterface : public ::testing::Test
{
protected:
  void SetUp() override
  {
    fts_hardware_interface_ =
      R"(
        <ros2_control name="fts_hw" type="sensor" is_async="true" >
          <hardware>
            <plugin>motoros2_hw_interfaces/FTSHardwareInterface</plugin>
            <param name="fts_topic">/tcp_force_torque</param>
            <param name="force_x_name">force.x</param>
            <param name="force_y_name">force.y</param>
            <param name="force_z_name">force.z</param>
            <param name="torque_x_name">torque.x</param>
            <param name="torque_y_name">torque.y</param>
            <param name="torque_z_name">torque.z</param>
          </hardware>
          <sensor name="force_torque_sensor">
            <state_interface name="force.x"/>
            <state_interface name="force.y"/>
            <state_interface name="force.z"/>
            <state_interface name="torque.x"/>
            <state_interface name="torque.y"/>
            <state_interface name="torque.z"/>
            <param name="frame_id">tool0</param>
          </sensor>
        </ros2_control>
    )";
  }

  std::string fts_hardware_interface_;
  rclcpp::Node node_ = rclcpp::Node("test_FTSHardwareInterface");
};

TEST_F(TestFTSHardwareInterface, load_fts_hardware_interface)
{
  auto urdf = ros2_control_test_assets::urdf_head + fts_hardware_interface_ +
              ros2_control_test_assets::urdf_tail;
  ASSERT_NO_THROW(hardware_interface::ResourceManager rm(
    urdf, node_.get_node_clock_interface(), node_.get_node_logging_interface()));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

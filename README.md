# Motoros2 Hardware Interfaces

This package contains the hardware interfaces for the Yaskawa robot arm. Specifically, it was written for the MOTOMAN HC20SDTP. However, the interfaces itself are not specific to this robot model and can be used with other MOTOMAN robot arms as well. Much more important is whether the MotoROS2 driver supports the robot arm model and/or the Yaskawa Motoman robot controller. For more information, please refer to the [MotoROS2](https://github.com/Yaskawa-Global/motoros2) repository. Additionally, it is possible that the provided ROS2 interfaces by MotoROS2 are already sufficient for your use case. In this case, you may not need to use this package.

## Components

This package provides state and command interfaces for the [ros2_control](https://github.com/ros-controls/ros2_control) framework. The following components are available:

- `FTSHardwareInterface`: This component is responsible for providing force-torque sensor data to the `ros2_control` framework. It is then possible to just use the `force_torque_sensor_broadcaster` to publish the force-torque sensor data for other controllers, e.g. the `admittance_controller`. The requirement for this component is that the force-torque sensor data is published on the `/tcp_force_torque` topic by the MotoROS2 driver.

- `GPIOSystem`: This component is responsible for providing state and command interfaces for digital input/output signals. This can be used to control external devices, such as grippers or other peripherals. The current states can be read from the `/dynamic_joint_states` topic. To control the digital outputs, the `forward_command_controller` can be used at fist. However, for more complex use cases, a custom controller may be necessary. The requirement for this component is that the services for the digital input/output signals are provided by the MotoROS2 driver (`/read_single_io`, `/write_single_io`, `/read_group_io`, `/write_group_io`).

- `MotoROS2HardwareInterface`: This component is responsible for providing joint state and command interfaces for the robot arm. In this way, it is possible to control the robot arm using ROS2 controllers, such as `joint_trajectory_controller` or `admittance_controller`. The requirement for this component is that the current joint states are published on the topic and `/queue_traj_point` service is provided by the MotoROS2 driver.

## Usage

To use the hardware interfaces, you need to add the following to your `URDF` file:

- For the `FTSHardwareInterface`:

```xml
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
  </sensor>
</ros2_control>
```

- For the `GPIOSystem`, please refer the corresponding file in the `test` directory.

- For the `MotoROS2HardwareInterface`:

```xml
<ros2_control name="motoros2_hw" type="system" is_async="true" >
  <hardware>
    <plugin>motoros2_hw_interfaces/MotoROS2HardwareInterface</plugin>
    <param name="queue_traj_point_srv_name">/queue_traj_point</param>
    <param name="joint_states_topic_name">/hc20sdtp_joint_states</param>
  </hardware>

  <joint name="joint_1_s">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0</param>
    </state_interface>
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="joint_2_l">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0</param>
    </state_interface>
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="joint_3_u">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0</param>
    </state_interface>
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="joint_4_r">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0</param>
    </state_interface>
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="joint_5_b">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0</param>
    </state_interface>
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="joint_6_t">
    <command_interface name="position"/>
    <state_interface name="position">
      <param name="initial_value">0</param>
    </state_interface>
    <command_interface name="velocity"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

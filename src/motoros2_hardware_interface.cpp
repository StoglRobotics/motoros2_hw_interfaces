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
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "motoros2_hw_interfaces/motoros2_hardware_interface.hpp"
#include "motoros2_interfaces/msg/queue_result_enum.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motoros2_hw_interfaces
{

constexpr char JOINT_STATE_NODE_NAME[] = "MotoROS2HardwareInterface_joint_state_node";
constexpr char NODE_NAME[] = "MotoROS2HardwareInterface_internal_node";
constexpr char QUEUE_TRAJ_POINT_SRV_NAME_KEY[] = "queue_traj_point_srv_name";
constexpr char JOINT_STATES_TOPIC_NAME_KEY[] = "joint_states_topic_name";
constexpr char THROTTLE_DURATION_KEY[] = "throttle_duration_ms";
constexpr char LOGGER_NAME[] = "MotoROS2HardwareInterface";
constexpr char WRITE_STATS_TOPIC_NAME[] = "~/write_stats";

const double DEFAULT_THROTTLE_DURATION_MS = 5000.0;
const double TOLERANCE = 1e-6;
const double DEFAULT_SMOOTHING_FACTOR = 0.1;
const int MIN_FUTURE_TIMEOUT_MS = 1;

hardware_interface::CallbackReturn MotoROS2HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  num_joints_ = info_.joints.size();

  for (const auto & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);

    if (
      joint.command_interfaces.size() != 2 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(LOGGER_NAME), "Joint '%s' has %zu command interfaces found. 2 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (
      joint.state_interfaces.size() != 2 ||
      joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
      joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(LOGGER_NAME), "Joint '%s' has %zu state interfaces found. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
  }

  state_current_.positions.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  state_current_.velocities.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  state_current_.accelerations.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  state_current_.effort.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());

  command_current_.positions.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  command_current_.velocities.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  command_current_.accelerations.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  command_current_.effort.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());

  request_ = std::make_shared<motoros2_interfaces::srv::QueueTrajPoint::Request>();
  request_->joint_names = joint_names_;
  // request->point = command_current_;
  request_->point.positions.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  request_->point.velocities.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  time_from_start_ = std::make_shared<rclcpp::Duration>(0, 0);

  node_ = std::make_shared<rclcpp::Node>(NODE_NAME);

  // Get parameters
  if (
    info_.hardware_parameters.find(QUEUE_TRAJ_POINT_SRV_NAME_KEY) ==
    info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(
      rclcpp::get_logger(LOGGER_NAME), "Parameter '%s' not found in hardware parameters.",
      QUEUE_TRAJ_POINT_SRV_NAME_KEY);
    return CallbackReturn::ERROR;
  }
  queue_traj_point_srv_name_ = info_.hardware_parameters[QUEUE_TRAJ_POINT_SRV_NAME_KEY];

  if (
    info_.hardware_parameters.find(JOINT_STATES_TOPIC_NAME_KEY) == info_.hardware_parameters.end())
  {
    RCLCPP_FATAL(
      rclcpp::get_logger(LOGGER_NAME), "Parameter '%s' not found in hardware parameters.",
      JOINT_STATES_TOPIC_NAME_KEY);
    return CallbackReturn::ERROR;
  }
  joint_state_topic_name_ = info_.hardware_parameters[JOINT_STATES_TOPIC_NAME_KEY];

  if (info_.hardware_parameters.find(THROTTLE_DURATION_KEY) == info_.hardware_parameters.end())
  {
    throttle_duration_ms_ = DEFAULT_THROTTLE_DURATION_MS;
    RCLCPP_WARN(
      rclcpp::get_logger(LOGGER_NAME),
      "Parameter '%s' not found in hardware parameters. Using default value: %f.",
      THROTTLE_DURATION_KEY, DEFAULT_THROTTLE_DURATION_MS);
  }

  moving_avg_write_exec_time_ms_.set_smoothing_factor(DEFAULT_SMOOTHING_FACTOR);

  write_stats_pub_ = node_->create_publisher<std_msgs::msg::String>(WRITE_STATS_TOPIC_NAME, 1);
  realtime_write_stats_pub_ =
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::String>>(write_stats_pub_);

  return CallbackReturn::SUCCESS;
}

void MotoROS2HardwareInterface::joint_state_thread_func()
{
  std::shared_ptr<rclcpp::Node> joint_state_node;
  joint_state_node = std::make_shared<rclcpp::Node>(JOINT_STATE_NODE_NAME);

  RCLCPP_INFO(
    rclcpp::get_logger(LOGGER_NAME), "Creating subscriber to topic '%s'.",
    joint_state_topic_name_.c_str());
  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();
  joint_state_sub_ = joint_state_node->create_subscription<sensor_msgs::msg::JointState>(
    joint_state_topic_name_, subscribers_qos, [&](const sensor_msgs::msg::JointState::SharedPtr msg)
    { joint_state_buffer_.writeFromNonRT(msg); });

  std::shared_ptr<sensor_msgs::msg::JointState> msg =
    std::make_shared<sensor_msgs::msg::JointState>();
  msg->name = joint_names_;
  msg->position.resize(num_joints_, 0.0);
  msg->velocity.resize(num_joints_, 0.0);
  msg->effort.resize(num_joints_, 0.0);
  joint_state_buffer_.writeFromNonRT(msg);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(joint_state_node);

  while (rclcpp::ok() && !stop_joint_state_thread_)
  {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

hardware_interface::CallbackReturn MotoROS2HardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Create client to queue trajectory point service
  RCLCPP_INFO(
    rclcpp::get_logger(LOGGER_NAME), "Creating client to service '%s'.",
    queue_traj_point_srv_name_.c_str());
  queue_traj_point_client_ =
    node_->create_client<motoros2_interfaces::srv::QueueTrajPoint>(queue_traj_point_srv_name_);

  // Start joint state subscriber thread
  joint_state_thread_ = std::thread(&MotoROS2HardwareInterface::joint_state_thread_func, this);

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotoROS2HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    state_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &state_current_.positions[i]);
    state_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &state_current_.velocities[i]);
    // state_interfaces.emplace_back(
    //   joint_names_[i], hardware_interface::HW_IF_ACCELERATION, &state_current_.accelerations[i]);
    // state_interfaces.emplace_back(
    //   joint_names_[i], hardware_interface::HW_IF_EFFORT, &state_current_.effort[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MotoROS2HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    command_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &command_current_.positions[i]);
    command_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &command_current_.velocities[i]);
    // command_interfaces.emplace_back(
    //   joint_names_[i], hardware_interface::HW_IF_ACCELERATION,
    //   &command_current_.accelerations[i]);
    // command_interfaces.emplace_back(
    //   joint_names_[i], hardware_interface::HW_IF_EFFORT, &command_current_.effort[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn MotoROS2HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  state_current_.positions.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  state_current_.velocities.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  state_current_.accelerations.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  state_current_.effort.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());

  command_current_.positions.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  command_current_.velocities.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  command_current_.accelerations.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  command_current_.effort.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());

  write_requests_total_ = 0;
  write_futures_failed_ = 0;
  write_motoros2_failed_ = 0;

  time_from_last_write_sec_ = node_->now();

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MotoROS2HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type MotoROS2HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto joint_state_msg_ptr = joint_state_buffer_.readFromRT();
  if (!joint_state_msg_ptr)
  {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger(LOGGER_NAME), *node_->get_clock(), throttle_duration_ms_,
      "No joint state message received on topic '%s'. Skipping.", joint_state_topic_name_.c_str());
    return hardware_interface::return_type::OK;
  }

  auto joint_state_msg = *joint_state_msg_ptr;
  if (!joint_state_msg)
  {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger(LOGGER_NAME), *node_->get_clock(), throttle_duration_ms_,
      "Joint state message is null on topic '%s'. Skipping.", joint_state_topic_name_.c_str());
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < joint_state_msg->name.size(); ++i)
  {
    auto it = std::find(joint_names_.begin(), joint_names_.end(), joint_state_msg->name[i]);
    if (it != joint_names_.end())
    {
      size_t index = std::distance(joint_names_.begin(), it);
      if (!std::isnan(joint_state_msg->position[i]))
      {
        state_current_.positions[index] = joint_state_msg->position[i];
      }
      if (!std::isnan(joint_state_msg->velocity[i]))
      {
        state_current_.velocities[index] = joint_state_msg->velocity[i];
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotoROS2HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  rclcpp::Time start = node_->now();

  if (!queue_traj_point_client_->service_is_ready())
  {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger(LOGGER_NAME), *node_->get_clock(), throttle_duration_ms_,
      "Service '%s' is not ready. Skipping.", queue_traj_point_srv_name_.c_str());
    publish_write_stats();
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < num_joints_; ++i)
  {
    if (std::isnan(command_current_.positions[i]) || std::isnan(command_current_.velocities[i]))
    {
      // RCLCPP_ERROR_THROTTLE(
      //   rclcpp::get_logger(LOGGER_NAME), *node_->get_clock(), throttle_duration_ms_,
      //   "Command for joint '%s' is not set. Skipping.", joint_names_[i].c_str());
      publish_write_stats();
      return hardware_interface::return_type::OK;
    }
  }

  request_->point.positions = command_current_.positions;
  request_->point.velocities = command_current_.velocities;
  (*time_from_start_) += period;
  request_->point.time_from_start = *time_from_start_;

  std::string joint_names_str = "[";
  std::string positions_str = "[";
  std::string velocities_str = "[";
  for (size_t i = 0; i < num_joints_; ++i)
  {
    joint_names_str += joint_names_[i] + (i < num_joints_ - 1 ? ", " : "]");
    positions_str +=
      std::to_string(request_->point.positions[i]) + (i < num_joints_ - 1 ? ", " : "]");
    velocities_str +=
      std::to_string(request_->point.velocities[i]) + (i < num_joints_ - 1 ? ", " : "]");
  }
  std::string log_msg = "Queueing trajectory point on service '" + queue_traj_point_srv_name_ +
                        "':\n" + "  - Joint names: " + joint_names_str + "\n" +
                        "  - Positions: " + positions_str + "\n" +
                        "  - Velocities: " + velocities_str + "\n";
  // RCLCPP_INFO(rclcpp::get_logger(LOGGER_NAME), log_msg.c_str());

  write_requests_total_++;
  auto result_future = queue_traj_point_client_->async_send_request(request_);
  double timeout_factor = 1.5;
  double timeout_ms = timeout_factor * period.seconds() * 1000.0;
  std::chrono::milliseconds timeout(static_cast<int>(timeout_ms));
  auto future_code = rclcpp::spin_until_future_complete(node_, result_future, timeout);

  double exec_time_ms = (node_->now() - start).seconds() * 1000.0;
  moving_avg_write_exec_time_ms_.update(exec_time_ms);

  // RCLCPP_INFO_THROTTLE(
  //   rclcpp::get_logger(LOGGER_NAME), *node_->get_clock(), throttle_duration_ms_,
  //   "Write execution time: %.3f ms (avg: %.3f ms)", exec_time_ms,
  //   moving_avg_write_exec_time_ms_.get_moving_avg());

  if (future_code != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger(LOGGER_NAME), *node_->get_clock(), throttle_duration_ms_,
      "Failed to call service '%s'. Skipping.", queue_traj_point_srv_name_.c_str());
    write_futures_failed_++;
    publish_write_stats();
    return hardware_interface::return_type::OK;
  }

  auto result = result_future.get();
  if (result->result_code.value != motoros2_interfaces::msg::QueueResultEnum::SUCCESS)
  {
    RCLCPP_ERROR_THROTTLE(
      rclcpp::get_logger(LOGGER_NAME), *node_->get_clock(), throttle_duration_ms_,
      "Failed to queue trajectory point on service '%s'. Result code: %d, message: %s",
      queue_traj_point_srv_name_.c_str(), result->result_code.value, result->message.c_str());
    write_motoros2_failed_++;
    publish_write_stats();
    return hardware_interface::return_type::OK;
  }

  time_from_last_write_sec_ = start;
  publish_write_stats();

  return hardware_interface::return_type::OK;
}

}  // namespace motoros2_hw_interfaces

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  motoros2_hw_interfaces::MotoROS2HardwareInterface, hardware_interface::SystemInterface)

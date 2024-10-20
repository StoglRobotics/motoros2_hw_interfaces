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
#include "motoros2_hw_interfaces/gpio_system.hpp"
#include "motoros2_interfaces/srv/read_group_io.hpp"
#include "motoros2_interfaces/srv/read_single_io.hpp"
#include "motoros2_interfaces/srv/write_group_io.hpp"
#include "motoros2_interfaces/srv/write_single_io.hpp"
#include "rclcpp/rclcpp.hpp"

namespace motoros2_hw_interfaces
{

constexpr char IO_TYPE_KEY[] = "io_type";
constexpr char IO_TYPE_SINGLE[] = "single";
constexpr char IO_TYPE_GROUP[] = "group";
constexpr char ADDRESS_KEY[] = "address";
constexpr char BIT_INDEX_KEY[] = "bit_index";
constexpr char UPDATE_RATE_KEY[] = "update_rate";
constexpr char FUTURE_TIMEOUT_KEY[] = "future_timeout";

constexpr char READ_SINGLE_IO_SERVICE_NAME_KEY[] = "read_single_io_srv";
constexpr char WRITE_SINGLE_IO_SERVICE_NAME_KEY[] = "write_single_io_srv";
constexpr char READ_GROUP_IO_SERVICE_NAME_KEY[] = "read_group_io_srv";
constexpr char WRITE_GROUP_IO_SERVICE_NAME_KEY[] = "write_group_io_srv";

constexpr char GPIO_SYSTEM_NODE_NAME[] = "gpio_system_node";

const int GROUP_NUM_BITS = 8;
const int GROUP_MULTIPLIER = 10;
const int DEFAULT_UPDATE_RATE = 2;                                     // [Hz]
const int DEFAULT_FUTURE_TIMEOUT_MS = 2 * DEFAULT_UPDATE_RATE * 1000;  // [ms]

const double DEFAULT_THROTTLE_DURATION_MS = 5000.0;

hardware_interface::CallbackReturn GPIOSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  read_single_io_srv_name_ = info_.hardware_parameters[READ_SINGLE_IO_SERVICE_NAME_KEY];
  write_single_io_srv_name_ = info_.hardware_parameters[WRITE_SINGLE_IO_SERVICE_NAME_KEY];
  read_group_io_srv_name_ = info_.hardware_parameters[READ_GROUP_IO_SERVICE_NAME_KEY];
  write_group_io_srv_name_ = info_.hardware_parameters[WRITE_GROUP_IO_SERVICE_NAME_KEY];

  node_ = std::make_shared<rclcpp::Node>(GPIO_SYSTEM_NODE_NAME);
  read_single_io_client_ =
    node_->create_client<motoros2_interfaces::srv::ReadSingleIO>(read_single_io_srv_name_);
  write_single_io_client_ =
    node_->create_client<motoros2_interfaces::srv::WriteSingleIO>(write_single_io_srv_name_);
  read_group_io_client_ =
    node_->create_client<motoros2_interfaces::srv::ReadGroupIO>(read_group_io_srv_name_);
  write_group_io_client_ =
    node_->create_client<motoros2_interfaces::srv::WriteGroupIO>(write_group_io_srv_name_);

  last_read_time_ = node_->now();
  last_write_time_ = node_->now();

  try
  {
    update_rate_ = std::stod(info_.hardware_parameters.at(UPDATE_RATE_KEY));
  }
  catch (const std::out_of_range & e)
  {
    update_rate_ = DEFAULT_UPDATE_RATE;
    RCLCPP_ERROR(
      node_->get_logger(), "Update rate parameter '%s' is missing. Using default value: %f [Hz]",
      UPDATE_RATE_KEY, update_rate_);
  }

  try
  {
    future_timeout_ms_ = std::stoi(info_.hardware_parameters.at(FUTURE_TIMEOUT_KEY));
  }
  catch (const std::out_of_range & e)
  {
    future_timeout_ms_ = DEFAULT_FUTURE_TIMEOUT_MS;
    RCLCPP_ERROR(
      node_->get_logger(), "Future timeout parameter '%s' is missing. Using default value: %d [ms]",
      FUTURE_TIMEOUT_KEY, future_timeout_ms_);
  }

  for (const auto & gpio : info_.gpios)
  {
    IOType io_type;
    try
    {
      const auto & io_type_param_value = gpio.parameters.at(IO_TYPE_KEY);
      if (io_type_param_value == IO_TYPE_SINGLE)
      {
        io_type = IOType::SINGLE;
      }
      else if (io_type_param_value == IO_TYPE_GROUP)
      {
        io_type = IOType::GROUP;
      }
      else
      {
        RCLCPP_ERROR(
          node_->get_logger(), "Invalid io_type parameter '%s' in GPIO '%s': %s", IO_TYPE_KEY,
          gpio.name.c_str(), io_type_param_value.c_str());
        return CallbackReturn::ERROR;
      }
    }
    catch (const std::out_of_range & e)
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Missing io_type parameter '%s' in GPIO '%s'", IO_TYPE_KEY,
        gpio.name.c_str());
      return CallbackReturn::ERROR;
    }

    uint32_t address{0};  // due to (warning: uninitvar), but we return error if it isn't in params
    if (io_type == IOType::GROUP)
    {
      try
      {
        address = static_cast<uint32_t>(std::stoul(gpio.parameters.at(ADDRESS_KEY)));
      }
      catch (const std::invalid_argument & e)
      {
        RCLCPP_ERROR(
          node_->get_logger(), "Invalid address parameter '%s' in GPIO '%s': %s. Expected integer",
          ADDRESS_KEY, gpio.name.c_str(), gpio.parameters.at(ADDRESS_KEY).c_str());
        return CallbackReturn::ERROR;
      }
      catch (const std::out_of_range & e)
      {
        RCLCPP_ERROR(
          node_->get_logger(), "Address parameter '%s' in GPIO '%s' is missing", ADDRESS_KEY,
          gpio.name.c_str());
        return CallbackReturn::ERROR;
      }
    }

    std::vector<InterfaceEntry> state_interface_entries;
    for (const auto & state_interface : gpio.state_interfaces)
    {
      InterfaceEntry entry;
      entry.name = state_interface.name;
      entry.value = std::numeric_limits<double>::quiet_NaN();
      if (io_type == IOType::SINGLE)
      {
        try
        {
          entry.address = std::stoi(state_interface.parameters.at(ADDRESS_KEY));
        }
        catch (const std::invalid_argument & e)
        {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Invalid address parameter '%s' in state interface '%s' of GPIO "
            "'%s': %s. Expected integer",
            ADDRESS_KEY, state_interface.name.c_str(), gpio.name.c_str(),
            state_interface.parameters.at(ADDRESS_KEY).c_str());
          return CallbackReturn::ERROR;
        }
        catch (const std::out_of_range & e)
        {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Address parameter '%s' in state interface '%s' of GPIO '%s' is "
            "missing",
            ADDRESS_KEY, state_interface.name.c_str(), gpio.name.c_str());
          return CallbackReturn::ERROR;
        }
      }
      else if (io_type == IOType::GROUP)
      {
        try
        {
          entry.bit_index = std::stoi(state_interface.parameters.at(BIT_INDEX_KEY));
        }
        catch (const std::invalid_argument & e)
        {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Invalid bit_index parameter '%s' in state interface '%s' of GPIO "
            "'%s': %s. Expected integer",
            BIT_INDEX_KEY, state_interface.name.c_str(), gpio.name.c_str(),
            state_interface.parameters.at(BIT_INDEX_KEY).c_str());
          return CallbackReturn::ERROR;
        }
        catch (const std::out_of_range & e)
        {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Bit index parameter '%s' in state interface '%s' of GPIO '%s' is "
            "missing",
            BIT_INDEX_KEY, state_interface.name.c_str(), gpio.name.c_str());
          return CallbackReturn::ERROR;
        }
      }
      state_interface_entries.push_back(entry);
    }

    std::vector<InterfaceEntry> command_interface_entries;
    for (const auto & command_interface : gpio.command_interfaces)
    {
      InterfaceEntry entry;
      entry.name = command_interface.name;
      entry.value = std::numeric_limits<double>::quiet_NaN();
      if (io_type == IOType::SINGLE)
      {
        try
        {
          entry.address = std::stoi(command_interface.parameters.at(ADDRESS_KEY));
        }
        catch (const std::invalid_argument & e)
        {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Invalid address parameter '%s' in command interface '%s' of GPIO "
            "'%s': %s. Expected integer",
            ADDRESS_KEY, command_interface.name.c_str(), gpio.name.c_str(),
            command_interface.parameters.at(ADDRESS_KEY).c_str());
          return CallbackReturn::ERROR;
        }
        catch (const std::out_of_range & e)
        {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Address parameter '%s' in command interface '%s' of GPIO '%s' is "
            "missing",
            ADDRESS_KEY, command_interface.name.c_str(), gpio.name.c_str());
          return CallbackReturn::ERROR;
        }
      }
      else if (io_type == IOType::GROUP)
      {
        try
        {
          entry.bit_index = std::stoi(command_interface.parameters.at(BIT_INDEX_KEY));
        }
        catch (const std::invalid_argument & e)
        {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Invalid bit_index parameter '%s' in command interface '%s' of GPIO "
            "'%s': %s. Expected integer",
            BIT_INDEX_KEY, command_interface.name.c_str(), gpio.name.c_str(),
            command_interface.parameters.at(BIT_INDEX_KEY).c_str());
          return CallbackReturn::ERROR;
        }
        catch (const std::out_of_range & e)
        {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Bit index parameter '%s' in command interface '%s' of GPIO '%s' is "
            "missing",
            BIT_INDEX_KEY, command_interface.name.c_str(), gpio.name.c_str());
          return CallbackReturn::ERROR;
        }

        if (entry.bit_index.value() < 0 || entry.bit_index.value() >= GROUP_NUM_BITS)
        {
          RCLCPP_ERROR(
            node_->get_logger(),
            "Invalid bit index parameter '%s' in command interface '%s' of GPIO "
            "'%s': %d. Expected integer between 0 and %d",
            BIT_INDEX_KEY, command_interface.name.c_str(), gpio.name.c_str(),
            entry.bit_index.value(), GROUP_NUM_BITS - 1);
          return CallbackReturn::ERROR;
        }
      }
      command_interface_entries.push_back(entry);
    }

    gpio_entries_.push_back(
      {gpio.name, io_type, address, command_interface_entries, state_interface_entries});
  }

  try
  {
    throttle_duration_ms_ = std::stod(info_.hardware_parameters.at("throttle_duration"));
  }
  catch (const std::out_of_range & e)
  {
    throttle_duration_ms_ = DEFAULT_THROTTLE_DURATION_MS;
    RCLCPP_WARN(
      node_->get_logger(), "Throttle duration parameter is missing. Using default value: %f [ms]",
      throttle_duration_ms_);
  }

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GPIOSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> GPIOSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < gpio_entries_.size(); ++i)
  {
    for (auto & state_interface_entry : gpio_entries_[i].state_interfaces_entries)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        gpio_entries_[i].name, state_interface_entry.name, &state_interface_entry.value));
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> GPIOSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < gpio_entries_.size(); ++i)
  {
    for (auto & command_interface_entry : gpio_entries_[i].command_interface_entries)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        gpio_entries_[i].name, command_interface_entry.name, &command_interface_entry.value));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn GPIOSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GPIOSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type GPIOSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // skip if update rate is not reached
  if (node_->now() - last_read_time_ < rclcpp::Duration::from_seconds(1.0 / update_rate_))
  {
    return hardware_interface::return_type::OK;
  }
  else
  {
    last_read_time_ = node_->now();
  }

  std::map<int, double> read_io_values;

  for (size_t i = 0; i < gpio_entries_.size(); ++i)
  {
    if (gpio_entries_[i].io_type == IOType::SINGLE)
    {
      for (size_t j = 0; j < gpio_entries_[i].state_interfaces_entries.size(); ++j)
      {
        // check if the address is already read
        auto it = read_io_values.find(gpio_entries_[i].state_interfaces_entries[j].address.value());
        if (it != read_io_values.end())
        {
          // if it is already read, use the value
          gpio_entries_[i].state_interfaces_entries[j].value = it->second;
          continue;
        }

        // if it is not read, read the value
        if (!read_single_io_client_->service_is_ready())
        {
          RCLCPP_WARN(
            node_->get_logger(), "Read single IO service '%s' is not ready",
            read_single_io_srv_name_.c_str());
          return hardware_interface::return_type::OK;
        }

        auto request = std::make_shared<motoros2_interfaces::srv::ReadSingleIO::Request>();
        request->address = gpio_entries_[i].state_interfaces_entries[j].address.value();
        auto result_future = read_single_io_client_->async_send_request(request);

        // wait for the response
        if (
          rclcpp::spin_until_future_complete(
            node_, result_future, std::chrono::milliseconds(future_timeout_ms_)) !=
          rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_WARN(
            node_->get_logger(), "Failed to read single IO '%d'",
            gpio_entries_[i].state_interfaces_entries[j].address.value());
          read_single_io_client_->remove_pending_request(result_future);
          return hardware_interface::return_type::OK;
        }

        int32_t read_value = result_future.get()->value;
        read_io_values[gpio_entries_[i].state_interfaces_entries[j].address.value()] = read_value;
        gpio_entries_[i].state_interfaces_entries[j].value = read_value;
      }
    }
    else if (gpio_entries_[i].io_type == IOType::GROUP)
    {
      // check if the addresses are already read
      bool all_read = true;
      for (int addr : get_group_io_addresses(
             gpio_entries_[i].address.value(), GROUP_MULTIPLIER, GROUP_NUM_BITS))
      {
        if (read_io_values.find(addr) == read_io_values.end())
        {
          all_read = false;
          break;
        }
      }

      if (!all_read)
      {
        // if it is not read, read the values
        if (!read_group_io_client_->service_is_ready())
        {
          RCLCPP_WARN(
            node_->get_logger(), "Read group IO service '%s' is not ready",
            read_group_io_srv_name_.c_str());
          return hardware_interface::return_type::OK;
        }

        auto request = std::make_shared<motoros2_interfaces::srv::ReadGroupIO::Request>();
        request->address = gpio_entries_[i].address.value();
        auto result_future = read_group_io_client_->async_send_request(request);

        // wait for the response
        if (
          rclcpp::spin_until_future_complete(
            node_, result_future, std::chrono::milliseconds(future_timeout_ms_)) !=
          rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_WARN(
            node_->get_logger(), "Failed to read group IO '%d'", gpio_entries_[i].address.value());
          read_group_io_client_->remove_pending_request(result_future);
          return hardware_interface::return_type::OK;
        }

        // update the read values
        uint8_t read_value = result_future.get()->value;
        auto addresses = get_group_io_addresses(
          gpio_entries_[i].address.value(), GROUP_MULTIPLIER, GROUP_NUM_BITS);
        for (size_t addr_i = 0; addr_i < addresses.size(); ++addr_i)
        {
          // idea: move the bit to the rightmost position and get the last bit
          read_io_values[addresses[addr_i]] = (read_value >> addr_i) & 1;
        }
      }

      // now go over the state interfaces and set the values
      for (size_t j = 0; j < gpio_entries_[i].state_interfaces_entries.size(); ++j)
      {
        int entry_address = get_group_io_address(
          gpio_entries_[i].address.value(), GROUP_MULTIPLIER,
          gpio_entries_[i].state_interfaces_entries[j].bit_index.value());
        gpio_entries_[i].state_interfaces_entries[j].value = read_io_values[entry_address];
      }
    }
    else
    {
      RCLCPP_ERROR(
        node_->get_logger(), "Invalid IO type for GPIO '%s'", gpio_entries_[i].name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  // log the read values
  std::string log_msg = "Read values: \n";
  for (const auto & entry : gpio_entries_)
  {
    for (const auto & si_entry : entry.state_interfaces_entries)
    {
      log_msg += "\t- " + entry.name + "/" + si_entry.name + ": " +
                 std::to_string(static_cast<int>(si_entry.value)) + "\n";
    }
  }
  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), throttle_duration_ms_, log_msg.c_str());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GPIOSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // skip if update rate is not reached
  if (node_->now() - last_write_time_ < rclcpp::Duration::from_seconds(1.0 / update_rate_))
  {
    return hardware_interface::return_type::OK;
  }
  else
  {
    last_write_time_ = node_->now();
  }

  std::string log_msg = "Write values: \n";
  // write only if the value has changed
  for (size_t i = 0; i < gpio_entries_.size(); ++i)
  {
    for (auto & si_entry : gpio_entries_[i].state_interfaces_entries)
    {
      for (auto & ci_entry : gpio_entries_[i].command_interface_entries)
      {
        // skip if
        // - the names are different
        // - the value is NaN
        // - the values are the same
        if (
          si_entry.name != ci_entry.name || std::isnan(ci_entry.value) ||
          si_entry.value == ci_entry.value)
        {
          continue;
        }

        int address;
        if (gpio_entries_[i].io_type == IOType::SINGLE)
        {
          address = ci_entry.address.value();
        }
        else
        {
          address = get_group_io_address(
            gpio_entries_[i].address.value(), GROUP_MULTIPLIER, ci_entry.bit_index.value());
        }

        if (!write_single_io_client_->service_is_ready())
        {
          RCLCPP_ERROR(
            node_->get_logger(), "Write single IO service '%s' is not ready, skipping",
            write_single_io_srv_name_.c_str());
          return hardware_interface::return_type::OK;
        }

        auto request = std::make_shared<motoros2_interfaces::srv::WriteSingleIO::Request>();
        request->address = address;
        request->value = static_cast<int32_t>(ci_entry.value);
        auto result_future = write_single_io_client_->async_send_request(request);

        // wait for the response
        if (
          rclcpp::spin_until_future_complete(
            node_, result_future, std::chrono::milliseconds(future_timeout_ms_)) !=
          rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to write single IO '%d', skipping", address);
          write_single_io_client_->remove_pending_request(result_future);
          return hardware_interface::return_type::OK;
        }

        log_msg += "\t- " + gpio_entries_[i].name + "/" + ci_entry.name + ": " +
                   std::to_string(static_cast<int>(ci_entry.value)) + "\n";
      }
    }
  }

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), throttle_duration_ms_, log_msg.c_str());

  return hardware_interface::return_type::OK;
}

}  // namespace motoros2_hw_interfaces

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(motoros2_hw_interfaces::GPIOSystem, hardware_interface::SystemInterface)

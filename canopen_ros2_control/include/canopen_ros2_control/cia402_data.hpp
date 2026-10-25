// Copyright (c) 2023, Fraunhofer IPA
// Copyright (c) 2022, StoglRobotics
// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
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

#ifndef CANOPEN_ROS2_CONTROL__CIA402_DATA_HPP_
#define CANOPEN_ROS2_CONTROL__CIA402_DATA_HPP_
#include <yaml-cpp/yaml.h>
#include <cstdint>
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include "canopen_402_driver/base.hpp"
#include "canopen_402_driver/cia402_driver.hpp"
#include "canopen_ros2_control/helpers.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using namespace ros2_canopen;
namespace canopen_ros2_control
{
struct Cia402Data
{
  uint8_t node_id;
  std::string joint_name;
  std::shared_ptr<ros2_canopen::Cia402Driver> driver;
  std::map<std::string, ros2_canopen::MotorBase::OperationMode> command_interface_to_operation_mode;
  std::vector<std::string> interfaces;
  std::vector<std::string> interfaces_to_start;
  std::vector<std::string> interfaces_to_running;
  std::vector<std::string> interfaces_to_stop;
  YAML::Node config;
  uint64_t command_epoch = 0;
  bool command_received = false;
  double lower = -std::numeric_limits<double>::infinity(), upper = std::numeric_limits<double>::infinity();
  double velocity_limit = std::numeric_limits<double>::infinity();

  // FROM MOTOR
  double actual_position = std::numeric_limits<double>::quiet_NaN();
  double actual_velocity = std::numeric_limits<double>::quiet_NaN();

  // TO MOTOR
  double target_position = std::numeric_limits<double>::quiet_NaN();
  double target_velocity = std::numeric_limits<double>::quiet_NaN();
  double target_torque = std::numeric_limits<double>::quiet_NaN();

  bool init_data(hardware_interface::ComponentInfo & joint, std::string device_dump)
  {
    joint_name = joint.name;
    for (const auto & interface : joint.command_interfaces) {
      if (interface.name == "position") {
        if (!interface.min.empty()) lower = std::stod(interface.min);
        if (!interface.max.empty()) upper = std::stod(interface.max);
      }
      if (interface.name == "velocity" && !interface.max.empty()) velocity_limit = std::stod(interface.max);
    }
    if (lower >= upper || velocity_limit <= 0) return false;
    auto config = YAML::Load(device_dump);
    const auto declares = [&joint](const std::string & name) {
      return std::any_of(joint.command_interfaces.begin(), joint.command_interfaces.end(),
        [&name](const auto & interface) { return interface.name == name; });
    };

    if (!config["node_id"])
    {
      RCLCPP_ERROR(rclcpp::get_logger(joint_name), "No node id for '%s'", joint.name.c_str());
      return false;
    }

    node_id = config["node_id"].as<uint16_t>();
    RCLCPP_INFO(
      rclcpp::get_logger(joint_name), "Node id for '%s' is '%u'", joint.name.c_str(), node_id);

    if (config["position_mode"] && declares("position"))
    {
      auto position_mode =
        (ros2_canopen::MotorBase::OperationMode)config["position_mode"].as<uint>();
      RCLCPP_INFO(
        rclcpp::get_logger(joint_name), "Registered position_mode '%u' for '%s'", position_mode,
        joint.name.c_str());
      command_interface_to_operation_mode.emplace(
        std::pair(joint.name + "/" + "position", position_mode));
    }
    if (config["velocity_mode"] && declares("velocity"))
    {
      auto velocity_mode =
        (ros2_canopen::MotorBase::OperationMode)config["velocity_mode"].as<uint>();
      RCLCPP_INFO(
        rclcpp::get_logger(joint_name), "Registered velocity_mode '%u' for '%s'", velocity_mode,
        joint.name.c_str());
      command_interface_to_operation_mode.emplace(
        std::pair(joint.name + "/" + "velocity", velocity_mode));
    }
    if (config["effort_mode"] && declares("effort"))
    {
      auto effort_mode = (ros2_canopen::MotorBase::OperationMode)config["effort_mode"].as<uint>();
      RCLCPP_INFO(
        rclcpp::get_logger(joint_name), "Registered effort_mode '%u' for '%s'", effort_mode,
        joint.name.c_str());
      command_interface_to_operation_mode.emplace(
        std::pair(joint.name + "/" + "effort", effort_mode));
    }
    return true;
  }

  void export_state_interface(std::vector<hardware_interface::StateInterface> & state_interfaces)
  {
    // actual position
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_name, hardware_interface::HW_IF_POSITION, &actual_position));

    // actual speed
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint_name, hardware_interface::HW_IF_VELOCITY, &actual_velocity));
  }

  void export_command_interface(
    std::vector<hardware_interface::CommandInterface> & command_interfaces)
  {
    if (
      command_interface_to_operation_mode.find(joint_name + "/" + "position") !=
      command_interface_to_operation_mode.end())
    {
      // target position
      command_interfaces.emplace_back(
        joint_name, hardware_interface::HW_IF_POSITION, &target_position);
      interfaces.push_back(joint_name + "/" + "position");
    }
    if (
      command_interface_to_operation_mode.find(joint_name + "/" + "velocity") !=
      command_interface_to_operation_mode.end())
    {
      // target velocity
      command_interfaces.emplace_back(
        joint_name, hardware_interface::HW_IF_VELOCITY, &target_velocity);
      interfaces.push_back(joint_name + "/" + "velocity");
    }
    if (
      command_interface_to_operation_mode.find(joint_name + "/" + "effort") !=
      command_interface_to_operation_mode.end())
    {
      // target effort
      command_interfaces.emplace_back(joint_name, hardware_interface::HW_IF_EFFORT, &target_torque);
      interfaces.push_back(joint_name + "/" + "effort");
    }
  }

  void read_state()
  {
    actual_position = driver->get_position();
    actual_velocity = driver->get_speed();
  }

  bool write_target()
  {
    return driver && write_target(*driver);
  }

  template <class Driver>
  bool write_target(Driver & motor)
  {
    if (interfaces_to_running.empty()) return true;
    if (interfaces_to_running.size() != 1) return false;
    const auto active = command_interface_to_operation_mode.find(interfaces_to_running.front());
    if (active == command_interface_to_operation_mode.end() || !motor.command_ready() ||
        motor.command_epoch() != command_epoch || motor.get_mode() != active->second)
    {
      interfaces_to_running.clear();
      clear_commands();
      return false;
    }
    const uint16_t mode = motor.get_mode();
    const double command = (mode == MotorBase::Profiled_Velocity || mode == MotorBase::Cyclic_Synchronous_Velocity) ? target_velocity :
      (mode == MotorBase::Profiled_Torque || mode == MotorBase::Cyclic_Synchronous_Torque) ? target_torque : target_position;
    if (!std::isfinite(command)) return std::isnan(command) && !command_received;
    command_received = true;
    switch (mode)
    {
      case MotorBase::No_Mode:
        break;
      case MotorBase::Profiled_Position:
      case MotorBase::Cyclic_Synchronous_Position:
      case MotorBase::Interpolated_Position:
        if (std::isfinite(target_position)) {
          if (target_position < lower || target_position > upper) return false;
          return motor.set_target(target_position, command_epoch);
        }
        break;
      case MotorBase::Profiled_Velocity:
      case MotorBase::Cyclic_Synchronous_Velocity:
        if (std::isfinite(target_velocity)) {
          if (std::abs(target_velocity) > velocity_limit ||
              (actual_position <= lower && target_velocity < 0) ||
              (actual_position >= upper && target_velocity > 0)) return false;
          return motor.set_target(target_velocity, command_epoch);
        }
        break;
      case MotorBase::Profiled_Torque:
      case MotorBase::Cyclic_Synchronous_Torque:
        if (std::isfinite(target_torque)) return motor.set_target(target_torque, command_epoch);
        break;
      default:
        return false;
    }
    return true;
  }

  void clear_commands()
  {
    command_received = false;
    target_position = target_velocity = target_torque = std::numeric_limits<double>::quiet_NaN();
  }

  bool perform_switch()
  {
    return driver && perform_switch(*driver);
  }

  template <class Driver>
  bool perform_switch(Driver & motor)
  {
    const auto start = interfaces_to_start;
    const auto stop = interfaces_to_stop;
    interfaces_to_start.clear();
    interfaces_to_stop.clear();
    if (start.size() > 1) return false;
    for (const auto & name : start)
      if (command_interface_to_operation_mode.count(name) == 0) return false;
    for (const auto & name : stop)
      if (command_interface_to_operation_mode.count(name) == 0) return false;

    const bool stopping = !interfaces_to_running.empty() &&
      std::find(stop.begin(), stop.end(), interfaces_to_running.front()) != stop.end();
    if (!start.empty() && !interfaces_to_running.empty() && !stopping) return false;
    if (start.empty() && !stopping) return true;

    // Revoke the old command session before any device operation, including failures.
    interfaces_to_running.clear();
    clear_commands();
    if (stopping && !motor.halt_motor()) return false;
    if (start.empty()) return true;
    const auto mode = command_interface_to_operation_mode.at(start.front());
    if (!motor.set_operation_mode(mode) || !motor.command_ready() || motor.get_mode() != mode)
      return false;
    command_epoch = motor.command_epoch();
    interfaces_to_running = start;
    return true;
  }

  bool check_id(uint8_t id)
  {
    if (node_id == id)
    {
      return true;
    }
    return false;
  }
};
}  // namespace canopen_ros2_control
#endif

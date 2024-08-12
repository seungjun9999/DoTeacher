// Copyright 2021 ros2_control Development Team
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

#include "doteacher/carlike_jetson.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace doteacher
{

bool CarLikeJetsonHardware::openPipe()
{
  pipe_.open(pipe_name_);
  if (!pipe_.is_open())
  {
    RCLCPP_ERROR(rclcpp::get_logger("CarLikeJetsonHardware"), "Failed to open pipe: %s", pipe_name_.c_str());
    return false;
  }
  return true;
}

void CarLikeJetsonHardware::writePipe(float steer, float throttle)
{
  if (pipe_.is_open())
  {
    pipe_ << steer << "," << throttle << std::endl;
    pipe_.flush();
    RCLCPP_INFO(rclcpp::get_logger("CarLikeJetsonHardware"), "Sent steer: %.2f, throttle: %.2f", steer, throttle);
  }
}

void CarLikeJetsonHardware::closePipe()
{
  if (pipe_.is_open())
  {
    pipe_.close();
  }
}

hardware_interface::CallbackReturn CarLikeJetsonHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CarLikeJetsonHardware"),
      "CarLikeJetsonHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("CarLikeJetsonHardware"), "Joint '%s' is a steering joint.",
        joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarLikeJetsonHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarLikeJetsonHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarLikeJetsonHardware"),
          "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarLikeJetsonHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("CarLikeJetsonHardware"), "Joint '%s' is a drive joint.",
        joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarLikeJetsonHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarLikeJetsonHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarLikeJetsonHardware"),
          "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarLikeJetsonHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarLikeJetsonHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarLikeJetsonHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // for (auto & joint : hw_interfaces_)
  // {
  //   state_interfaces.emplace_back(hardware_interface::StateInterface(
  //     joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

  //   if (joint.first == "throttle")
  //   {
  //     state_interfaces.emplace_back(hardware_interface::StateInterface(
  //       joint.second.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.second.state.velocity));
  //   }
  // }

  RCLCPP_INFO(
    rclcpp::get_logger("CarLikeJetsonHardware"), "Exported %zu state interfaces.",
    state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("CarLikeJetsonHardware"), "Exported state interface '%s'.",
      s.get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CarLikeJetsonHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // for (auto & joint : hw_interfaces_)
  // {
  //   if (joint.first == "steering")
  //   {
  //     command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //       joint.second.joint_name, hardware_interface::HW_IF_POSITION,
  //       &joint.second.command.position));
  //   }
  //   else if (joint.first == "throttle")
  //   {
  //     command_interfaces.emplace_back(hardware_interface::CommandInterface(
  //       joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
  //       &joint.second.command.velocity));
  //   }
  // }

  RCLCPP_INFO(
    rclcpp::get_logger("CarLikeJetsonHardware"), "Exported %zu command interfaces.",
    command_interfaces.size());

  for (auto i = 0u; i < command_interfaces.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("CarLikeJetsonHardware"), "Exported command interface '%s'.",
      command_interfaces[i].get_name().c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn CarLikeJetsonHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarLikeJetsonHardware"), "Activating ...please wait...");

  if (!openPipe())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // for (auto i = 0; i < hw_start_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("CarLikeJetsonHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  // }

  // for (auto & joint : hw_interfaces_)
  // {
  //   joint.second.state.position = 0.0;

  //   if (joint.first == "throttle")
  //   {
  //     joint.second.state.velocity = 0.0;
  //     joint.second.command.velocity = 0.0;
  //   }

  //   else if (joint.first == "steering")
  //   {
  //     joint.second.command.position = 0.0;
  //   }
  // }

  RCLCPP_INFO(rclcpp::get_logger("CarLikeJetsonHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarLikeJetsonHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("CarLikeJetsonHardware"), "Deactivating ...please wait...");

  // for (auto i = 0; i < hw_stop_sec_; i++)
  // {
  //   rclcpp::sleep_for(std::chrono::seconds(1));
  //   RCLCPP_INFO(
  //     rclcpp::get_logger("CarLikeJetsonHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  // }

  closePipe();

  // END: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("CarLikeJetsonHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarLikeJetsonHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

  // hw_interfaces_["steering"].state.position = hw_interfaces_["steering"].command.position;

  // hw_interfaces_["throttle"].state.velocity = hw_interfaces_["throttle"].command.velocity;
  // hw_interfaces_["throttle"].state.position +=
  //   hw_interfaces_["throttle"].state.velocity * period.seconds();

  // RCLCPP_INFO(
  //   rclcpp::get_logger("CarLikeJetsonHardware"), "Got position state: %.2f for joint '%s'.",
  //   hw_interfaces_["steering"].command.position, hw_interfaces_["steering"].joint_name.c_str());

  // RCLCPP_INFO(
  //   rclcpp::get_logger("CarLikeJetsonHardware"), "Got velocity state: %.2f for joint '%s'.",
  //   hw_interfaces_["throttle"].command.velocity, hw_interfaces_["throttle"].joint_name.c_str());

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type doteacher ::CarLikeJetsonHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

  // RCLCPP_INFO(
  //   rclcpp::get_logger("CarLikeJetsonHardware"), "Got position command: %.2f for joint '%s'.",
  //   hw_interfaces_["steering"].command.position, hw_interfaces_["steering"].joint_name.c_str());

  // RCLCPP_INFO(
  //   rclcpp::get_logger("CarLikeJetsonHardware"), "Got velocity command: %.2f for joint '%s'.",
  //   hw_interfaces_["throttle"].command.velocity, hw_interfaces_["throttle"].joint_name.c_str());


  // float steer = hw_interfaces_["steering"].command.position;
  // float throttle = hw_interfaces_["throttle"].command.velocity;

  // writePipe(steer, throttle);

  // END: This part here is for exemplary purposes - Please do not copy to your production code
  return hardware_interface::return_type::OK;
}

}  // namespace doteacher

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  doteacher::CarLikeJetsonHardware, hardware_interface::SystemInterface)

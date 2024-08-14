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

#include "doteacher/carlikebot_system.hpp"

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

  bool CarlikeBotSystemHardware::openPipe()
  {
    pipe_.open(pipe_name_);
    if (!pipe_.is_open())
    {
      RCLCPP_ERROR(rclcpp::get_logger("CarlikeBotSystemHardware"), "Failed to open pipe: %s", pipe_name_.c_str());
      return false;
    }
    return true;
  }

  void CarlikeBotSystemHardware::writePipe(float steer, float throttle)
  {
    if (pipe_.is_open())
    {
      // steer calc.

      float steer_deg = steer * (180.0 / M_PI) * config_.adj_steer; // radian을 degree로 변환

      // 95도를 기준으로 조향 각도 계산
      float adjusted_steer_deg = 100.0 - steer_deg;

      // ±30도 범위로 제한
      if (adjusted_steer_deg > 150.0)
      {
        adjusted_steer_deg = 150.0; // 최대 조향 각도로 제한
      }
      else if (adjusted_steer_deg < 30.0)
      {
        adjusted_steer_deg = 30.0; // 최소 조향 각도로 제한
      }

      float adj_throttle = 0;

      // speed_mps는 최대 속도로 제한됨, 방향은 부호로 표현
      float throttle_abs = abs(throttle);
      if (throttle_abs > 5.0)
      {
        adj_throttle = config_.adj_throttle_8_0;
      }
      else if (throttle_abs > 5.0)
      {
        adj_throttle = config_.adj_throttle_5_0;
      }
      else if (throttle_abs > 3.0)
      {
        adj_throttle = config_.adj_throttle_3_0;
      }
      else if (throttle_abs > 2.0)
      {
        adj_throttle = config_.adj_throttle_2_0;
      }
      else if (throttle_abs > 1.2)
      {
        adj_throttle = config_.adj_throttle_1_2;
      }
      else if (throttle_abs > 1.0)
      {
        adj_throttle = config_.adj_throttle_1_0;
      }
      else if (throttle_abs > 0.9)
      {
        adj_throttle = config_.adj_throttle_0_9;
      }
      else if (throttle_abs > 0.8)
      {
        adj_throttle = config_.adj_throttle_0_8;
      }
      else if (throttle_abs > 0.7)
      {
        adj_throttle = config_.adj_throttle_0_7;
      }
      else if (throttle_abs > 0.6)
      {
        adj_throttle = config_.adj_throttle_0_6;
      }
      else if (throttle_abs > 0.5)
      {
        adj_throttle = config_.adj_throttle_0_5;
      }
      else if (throttle_abs > 0.4)
      {
        adj_throttle = config_.adj_throttle_0_4;
      }
      else if (throttle_abs > 0.3)
      {
        adj_throttle = config_.adj_throttle_0_3;
      }
      else if (throttle_abs > 0.2)
      {
        adj_throttle = config_.adj_throttle_0_2;
      }
      else if (throttle_abs > 0.1)
      {
        adj_throttle = config_.adj_throttle_0_1;
      }
      else
      {
        adj_throttle = config_.adj_throttle_0_0;
      }

      float throttle_output = throttle / 2 / M_PI / gear_ratio_ / adj_throttle; // 기어비 및 제대로 된 계산 적용 // 8.8
      // RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "%.6f, %.6f", throttle, throttle_output);

      float throttle_ratio = throttle_output = throttle_output / max_speed_mps_; // -1 ~ 1 사이의 값
      // RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "%.6f", throttle_ratio);

      // throttle_output을 -1 ~ 1 사이로 제한
      if (throttle_ratio > 1.0)
      {
        throttle_ratio = 1.0;
      }
      else if (throttle_ratio < -1.0)
      {
        throttle_ratio = -1.0;
      }

      pipe_ << adjusted_steer_deg << "," << throttle_ratio << std::endl;
      pipe_.flush();

      RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Sent steer: %.2f, throttle: %.2f", adjusted_steer_deg, throttle_ratio);
    }
  }

  void CarlikeBotSystemHardware::closePipe()
  {
    if (pipe_.is_open())
    {
      pipe_.close();
    }
  }

  hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
      const hardware_interface::HardwareInfo &info)
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
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
          "because the number of joints %ld is not 2.",
          info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (const hardware_interface::ComponentInfo &joint : info_.joints)
    {
      bool joint_is_steering = joint.name.find("front") != std::string::npos;

      // Steering joints have a position command interface and a position state interface
      if (joint_is_steering)
      {
        RCLCPP_INFO(
            rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' is a steering joint.",
            joint.name.c_str());

        if (joint.command_interfaces.size() != 1)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
              joint.command_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
              joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 1)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
              joint.state_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
      else
      {
        RCLCPP_INFO(
            rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' is a drive joint.",
            joint.name.c_str());

        // Drive joints have a velocity command interface and a velocity state interface
        if (joint.command_interfaces.size() != 1)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
              joint.command_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
              joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 2)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
              joint.state_interfaces.size());
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
          return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_FATAL(
              rclcpp::get_logger("CarlikeBotSystemHardware"),
              "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
              joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
          return hardware_interface::CallbackReturn::ERROR;
        }
      }
    }

    // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production
    // code
    hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
    hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
    config_.adj_steer = std::stof(info_.hardware_parameters["adj_steer"]);
    config_.adj_throttle_8_0 = std::stof(info_.hardware_parameters["adj_throttle_8_0"]);
    config_.adj_throttle_5_0 = std::stof(info_.hardware_parameters["adj_throttle_5_0"]);
    config_.adj_throttle_3_0 = std::stof(info_.hardware_parameters["adj_throttle_3_0"]);
    config_.adj_throttle_2_0 = std::stof(info_.hardware_parameters["adj_throttle_2_0"]);
    config_.adj_throttle_1_2 = std::stof(info_.hardware_parameters["adj_throttle_1_2"]);
    config_.adj_throttle_1_0 = std::stof(info_.hardware_parameters["adj_throttle_1_0"]);
    config_.adj_throttle_0_9 = std::stof(info_.hardware_parameters["adj_throttle_0_9"]);
    config_.adj_throttle_0_8 = std::stof(info_.hardware_parameters["adj_throttle_0_8"]);
    config_.adj_throttle_0_7 = std::stof(info_.hardware_parameters["adj_throttle_0_7"]);
    config_.adj_throttle_0_6 = std::stof(info_.hardware_parameters["adj_throttle_0_6"]);
    config_.adj_throttle_0_5 = std::stof(info_.hardware_parameters["adj_throttle_0_5"]);
    config_.adj_throttle_0_4 = std::stof(info_.hardware_parameters["adj_throttle_0_4"]);
    config_.adj_throttle_0_3 = std::stof(info_.hardware_parameters["adj_throttle_0_3"]);
    config_.adj_throttle_0_2 = std::stof(info_.hardware_parameters["adj_throttle_0_2"]);
    config_.adj_throttle_0_1 = std::stof(info_.hardware_parameters["adj_throttle_0_1"]);
    config_.adj_throttle_0_0 = std::stof(info_.hardware_parameters["adj_throttle_0_0"]);
    // // END: This part here is for exemplary purposes - Please do not copy to your production code

    hw_interfaces_["steering"] = Joint("virtual_front_wheel_joint");

    hw_interfaces_["traction"] = Joint("virtual_rear_wheel_joint");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (auto &joint : hw_interfaces_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

      if (joint.first == "traction")
      {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            joint.second.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.second.state.velocity));
      }
    }

    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported %zu state interfaces.",
        state_interfaces.size());

    for (auto s : state_interfaces)
    {
      RCLCPP_INFO(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported state interface '%s'.",
          s.get_name().c_str());
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface>
  CarlikeBotSystemHardware::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (auto &joint : hw_interfaces_)
    {
      if (joint.first == "steering")
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.second.joint_name, hardware_interface::HW_IF_POSITION,
            &joint.second.command.position));
      }
      else if (joint.first == "traction")
      {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
            &joint.second.command.velocity));
      }
    }

    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported %zu command interfaces.",
        command_interfaces.size());

    for (auto i = 0u; i < command_interfaces.size(); i++)
    {
      RCLCPP_INFO(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported command interface '%s'.",
          command_interfaces[i].get_name().c_str());
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Activating ...please wait...");

    if (!openPipe())
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    for (auto i = 0; i < hw_start_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
    }

    for (auto &joint : hw_interfaces_)
    {
      joint.second.state.position = 0.0;

      if (joint.first == "traction")
      {
        joint.second.state.velocity = 0.0;
        joint.second.command.velocity = 0.0;
      }

      else if (joint.first == "steering")
      {
        joint.second.command.position = 0.0;
      }
    }

    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully activated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Deactivating ...please wait...");

    for (auto i = 0; i < hw_stop_sec_; i++)
    {
      rclcpp::sleep_for(std::chrono::seconds(1));
      RCLCPP_INFO(
          rclcpp::get_logger("CarlikeBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
    }

    closePipe();

    // END: This part here is for exemplary purposes - Please do not copy to your production code
    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully deactivated!");

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type CarlikeBotSystemHardware::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

    hw_interfaces_["steering"].state.position = hw_interfaces_["steering"].command.position;

    hw_interfaces_["traction"].state.velocity = hw_interfaces_["traction"].command.velocity;
    hw_interfaces_["traction"].state.position +=
        hw_interfaces_["traction"].state.velocity * period.seconds();

    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Got position state: %.2f for joint '%s'.",
        hw_interfaces_["steering"].command.position, hw_interfaces_["steering"].joint_name.c_str());

    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Got velocity state: %.2f for joint '%s'.",
        hw_interfaces_["traction"].command.velocity, hw_interfaces_["traction"].joint_name.c_str());

    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type doteacher ::CarlikeBotSystemHardware::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code

    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Got position command: %.2f for joint '%s'.",
        hw_interfaces_["steering"].command.position, hw_interfaces_["steering"].joint_name.c_str());

    RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Got velocity command: %.2f for joint '%s'.",
        hw_interfaces_["traction"].command.velocity, hw_interfaces_["traction"].joint_name.c_str());

    writePipe(hw_interfaces_["steering"].command.position, hw_interfaces_["traction"].command.velocity);

    // END: This part here is for exemplary purposes - Please do not copy to your production code

    return hardware_interface::return_type::OK;
  }

} // namespace doteacher

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    doteacher::CarlikeBotSystemHardware, hardware_interface::SystemInterface)

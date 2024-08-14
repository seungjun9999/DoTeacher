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

#ifndef CARLIKE_JETSON__CARLIKEBOT_SYSTEM_HPP_
#define CARLIKE_JETSON__CARLIKEBOT_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <cmath>

#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace doteacher
{
  struct JointValue
  {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
  };

  struct Joint
  {
    explicit Joint(const std::string &name) : joint_name(name)
    {
      state = JointValue();
      command = JointValue();
    }

    Joint() = default;

    std::string joint_name;
    JointValue state;
    JointValue command;
  };
  class CarlikeBotSystemHardware : public hardware_interface::SystemInterface
  {

    struct Config
    {
      float adj_steer = 0;
      float adj_throttle_8_0 = 0;
      float adj_throttle_5_0 = 0;
      float adj_throttle_3_0 = 0;
      float adj_throttle_2_0 = 0;
      float adj_throttle_1_2 = 0;
      float adj_throttle_1_0 = 0;
      float adj_throttle_0_9 = 0;
      float adj_throttle_0_8 = 0;
      float adj_throttle_0_7 = 0;
      float adj_throttle_0_6 = 0;
      float adj_throttle_0_5 = 0;
      float adj_throttle_0_4 = 0;
      float adj_throttle_0_3 = 0;
      float adj_throttle_0_2 = 0;
      float adj_throttle_0_1 = 0;
      float adj_throttle_0_0 = 0;
    };

    public : RCLCPP_SHARED_PTR_DEFINITIONS(CarlikeBotSystemHardware);

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    hardware_interface::return_type read(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  private:
    // Parameters for the CarlikeBot simulation
    double hw_start_sec_;
    double hw_stop_sec_;

    Config config_;
    // std::vector<std::tuple<std::string, double, double>>
    //   hw_interfaces_;  // name of joint, state, command
    std::map<std::string, Joint> hw_interfaces_;
    // pipe
    std::ofstream pipe_;
    const std::string pipe_name_ = "/tmp/steer_throttle_pipe";

    const float max_motor_rpm_ = 310.0;                            // 모터의 최대 RPM
    const float gear_ratio_ = 30.0 / 54.0;                         // 기어비 (모터 측 / 바퀴 측)
    const float wheel_diameter_m_ = 65.0 / 1000.0;                 // 바퀴의 지름 (m)
    const float wheel_circumference_m_ = M_PI * wheel_diameter_m_; // 바퀴의 둘레 (m)

    // 최대 차량 속도 (m/s)
    const float max_speed_mps_ = (max_motor_rpm_ / 60.0) * gear_ratio_ * wheel_circumference_m_;

    bool openPipe();
    void writePipe(float steer, float throttle);
    void closePipe();
  };

} // namespace doteacher

#endif // CARLIKE_JETSON__CARLIKEBOT_SYSTEM_HPP_

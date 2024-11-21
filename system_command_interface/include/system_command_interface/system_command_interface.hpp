// Copyright (c) 2024 Tohoku Univ. Space Robotics Lab.
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

#ifndef SYSTEM_COMMAND_INTERFACE__SYSTEM_COMMAND_INTERFACE_HPP_
#define SYSTEM_COMMAND_INTERFACE__SYSTEM_COMMAND_INTERFACE_HPP_

#include <functional>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include <math.h>


class SystemCommandInterface : public rclcpp::Node
{
public:
  SystemCommandInterface();

private:
  void showCommandList();
  void execInputCommand(const std::string command);

  /**
   * @brief This callback function runs when this node subscribes /joy topic.
   *
   * @param joy_cmd
   */
  void joyCallback(const sensor_msgs::msg::Joy & joy_cmd);

  void stringCallback(const std_msgs::msg::String & string);

  /**
   * @brief This function runs in certain rate.
   */
  void timerCallback();

  // Subscriber
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  // Publisher
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_command_pub_;
  sensor_msgs::msg::JointState current_joint_state;

  rclcpp::TimerBase::SharedPtr timer_;

  // Variables
  sensor_msgs::msg::Joy input_joy_cmd;
  int timer_loop_rate;  // [ms]

  float motor_angle;
};

#endif  // SYSTEM_COMMAND_INTERFACE__SYSTEM_COMMAND_INTERFACE_HPP_
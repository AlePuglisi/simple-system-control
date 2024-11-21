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

#include "system_command_interface/system_command_interface.hpp"

#define DEBUG_ENABLED false

SystemCommandInterface::SystemCommandInterface()
: Node("system_command_interface")
{
  std::cout << "SystemCommandInterface class is established." << std::endl;

  // Get namespace and node name.
  std::string topic_prefix = "/" + std::string(this->get_name());

  // Subscriber
  string_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/string", 1, std::bind(&SystemCommandInterface::stringCallback, this, std::placeholders::_1));
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 1, std::bind(&SystemCommandInterface::joyCallback, this, std::placeholders::_1));
  
  // Publisher 
  joint_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/trajectory_generation/joint_state", 1);

  // Set timer callback
  int timer_frequency = 20;  // [Hz]
  timer_loop_rate = 1000 / timer_frequency;  // [ms]
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_loop_rate),
    std::bind(&SystemCommandInterface::timerCallback, this));

  // Initialize input_joy_cmd to execute timerCallback.
  input_joy_cmd.header.stamp = rclcpp::Clock().now();
  input_joy_cmd.axes = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  input_joy_cmd.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  motor_angle = 0; 

  showCommandList();
}

void SystemCommandInterface::stringCallback(const std_msgs::msg::String & string)
{
  execInputCommand(string.data);
}

void SystemCommandInterface::timerCallback()
{
#if DEBUG_ENABLED
  std::cout << "timerCallback" << std::endl;
#endif  // DEBUG_ENABLED
  // bool square_button_pressed = input_joy_cmd.buttons.at(3) == 1;
  // bool X_button_pressed = input_joy_cmd.buttons.at(0) == 1;
  // bool circle_button_pressed = input_joy_cmd.buttons.at(1) == 1;
  // bool triangle_button_pressed = input_joy_cmd.buttons.at(2) == 1;
  bool L1_pressed = input_joy_cmd.buttons.at(4) == 1;
  bool L2_pressed = input_joy_cmd.buttons.at(6) == 1;
  bool R1_pressed = input_joy_cmd.buttons.at(5) == 1;
  bool R2_pressed = input_joy_cmd.buttons.at(7) == 1;
  // bool option_button_pressed = input_joy_cmd.buttons.at(9) == 1;
  // bool share_button_pressed = input_joy_cmd.buttons.at(8) == 1;

  // bool left_joystick_input = input_joy_cmd.axes.at(0) != 0 || input_joy_cmd.axes.at(1) != 0;
  // bool right_joystick_input = input_joy_cmd.axes.at(3) != 0 || input_joy_cmd.axes.at(4) != 0;
  // bool L1_R1_are_pressed = input_joy_cmd.buttons.at(4) == 1 && input_joy_cmd.buttons.at(5) == 1;
  // bool L1_or_L2_pressed = L1_pressed || L2_pressed;
  // bool R1_or_R2_pressed = R1_pressed || R2_pressed;
  // bool L2_R2_are_pressed = input_joy_cmd.buttons.at(6) == 1 && input_joy_cmd.buttons.at(7) == 1;
  // bool axes_are_pressed = input_joy_cmd.axes.at(6) != 0 || input_joy_cmd.axes.at(7) != 0;
  // bool stick_is_pressed = input_joy_cmd.buttons.at(11) == 1 || input_joy_cmd.buttons.at(12) == 1;
  // bool left_joy_stick_used = input_joy_cmd.axes.at(0) != 0.0 || input_joy_cmd.axes.at(1) != 0.0;
  bool left_arrow_pressed = input_joy_cmd.axes.at(6) ==  1;
  bool right_arrow_pressed = input_joy_cmd.axes.at(6) == -1;
  bool up_arrow_pressed = input_joy_cmd.axes.at(7) == 1;
  // bool bottom_arrow_pressed = input_joy_cmd.axes.at(6) == -1;
//  if (option_button_pressed) {
//     std_msgs::msg::String msg;
//     msg.data = "initialize";
//     command_to_LLC_pub_->publish(msg);
//     std::cout << "Input command is<" << "\033[32m initialize\033[m" << ">." << std::endl;
//   } else if (share_button_pressed) {
//     std_msgs::msg::String msg;
//     msg.data = "grieel_mode_transform";
//     command_to_HLC_pub_->publish(msg);
//     std::cout << "Input command is<" << "\033[32m grieel mode transform\033[m" << ">." << std::endl;
//   }
  if(left_arrow_pressed){
    motor_angle = M_PI_2;
    sensor_msgs::msg::JointState joint_command_msg;
    joint_command_msg.position.resize(1);
    joint_command_msg.velocity.resize(1);
    joint_command_msg.name.resize(1);
    joint_command_msg.position.at(0) = M_PI_2;
    joint_command_pub_->publish(joint_command_msg);
  } else if(right_arrow_pressed){
    motor_angle = -M_PI_2;
    sensor_msgs::msg::JointState joint_command_msg;
    joint_command_msg.position.resize(1);
    joint_command_msg.velocity.resize(1);
    joint_command_msg.name.resize(1);
    joint_command_msg.position.at(0) = -M_PI_2;
    joint_command_pub_->publish(joint_command_msg);
  } else if(up_arrow_pressed){
    motor_angle = 0;
    sensor_msgs::msg::JointState joint_command_msg;
    joint_command_msg.position.resize(1);
    joint_command_msg.velocity.resize(1);
    joint_command_msg.name.resize(1);
    joint_command_msg.position.at(0) = 0;
    joint_command_pub_->publish(joint_command_msg);
  } else if(L2_pressed){
    motor_angle = M_PI_4;
    sensor_msgs::msg::JointState joint_command_msg;
    joint_command_msg.position.resize(1);
    joint_command_msg.velocity.resize(1);
    joint_command_msg.name.resize(1);
    joint_command_msg.position.at(0) = M_PI_4;
    joint_command_pub_->publish(joint_command_msg);
  } else if(R2_pressed){
    motor_angle = -M_PI_4; 
    sensor_msgs::msg::JointState joint_command_msg;
    joint_command_msg.position.resize(1);
    joint_command_msg.velocity.resize(1);
    joint_command_msg.name.resize(1);
    joint_command_msg.position.at(0) = -M_PI_4;
    joint_command_pub_->publish(joint_command_msg);
  } else if(L1_pressed){
    motor_angle = motor_angle + M_PI/18;
    sensor_msgs::msg::JointState joint_command_msg;
    joint_command_msg.position.resize(1);
    joint_command_msg.velocity.resize(1);
    joint_command_msg.name.resize(1);
    joint_command_msg.position.at(0) = motor_angle + M_PI/18;
    joint_command_pub_->publish(joint_command_msg);
  } else if(R1_pressed){
    motor_angle = motor_angle - M_PI/18;
    sensor_msgs::msg::JointState joint_command_msg;
    joint_command_msg.position.resize(1);
    joint_command_msg.velocity.resize(1);
    joint_command_msg.name.resize(1);
    joint_command_msg.position.at(0) = motor_angle - M_PI/18;
    joint_command_pub_->publish(joint_command_msg);
  }
}

void SystemCommandInterface::showCommandList()
{
  using std::cout;
  using std::endl;

  cout << endl << "------------------------ Command List ------------------------" << endl;
// List of commands from command line 
  cout << "COMMANDS TO JOINT MOTOR:" << endl;
  cout << "" << endl;
  cout << "Please open another terminal and send a command." << endl;
  cout << "\033[32m ros2 topic pub --once /string std_msgs/msg/String '{data: YOUR_COMMAND}'\033[m";
  cout << "" << endl; 
  cout << "" << endl;
  cout << "<initialize>: Set joint pose as initial pose (Vertical)" << endl;
  cout << "<right>: Set joint pose to - 90 deg (CW limit)" << endl;
  cout << "<left>: Set joint pose to + 90 deg (CCW limit)" << endl;
  cout << "<mid_right>: Set joint pose to - 45 deg (CW limit)" << endl;
  cout << "<mid_left>: Set joint pose to + 45 deg (CCW limit)" << endl;
  cout << "" << endl;
// list of joystick commands 
  cout << "COMMANDS FROM JOYSTICK:" << endl;
  cout << "" << endl;
  cout << "R1:          + 10 deg CW  (relative to current angle) " << endl;
  cout << "L1:          + 10 deg CCw (relative to current angle) " << endl;
  cout << "right arrow: + 90 deg CCw (absolute) " << endl;
  cout << "left arrow:  - 90 deg Cw  (absolute) " << endl;
  cout << "up arrow:       0 deg UP  (absolute) " << endl;
  cout << "R2:          + 45 deg CW  (relative to current angle) " << endl;
  cout << "L2:          - 45 deg CCw (relative to current angle) " << endl;
  cout << endl << "<help>: Show command list" << endl;

  cout << "------------------------ End ------------------------" << endl;
  cout << "" << endl;
  cout << endl << "Waiting command..." << endl << endl;
}

void SystemCommandInterface::execInputCommand(const std::string command)
{
  std::cout << "Input command is <" << "\033[32m" << command << "\033[m" << ">." << std::endl;
  sensor_msgs::msg::JointState joint_command_msg;
  joint_command_msg.position.resize(1);
  joint_command_msg.velocity.resize(1);
  joint_command_msg.name.resize(1);

  joint_command_msg.name.at(0) = "motor_joint";
  joint_command_msg.velocity.at(0) = 0; 

  if (command == "initialize") {
    joint_command_msg.position.at(0) = 0;
    joint_command_pub_->publish(joint_command_msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;

  } else if (command == "right") {
    joint_command_msg.position.at(0) = - M_PI_2;
    joint_command_pub_->publish(joint_command_msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;

  } else if (command == "left") {
    joint_command_msg.position.at(0) = M_PI_2;
    joint_command_pub_->publish(joint_command_msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  } else if (command == "mid_right") {
    joint_command_msg.position.at(0) = -M_PI_4;
    joint_command_pub_->publish(joint_command_msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  }  else if (command == "mid_left") {
    joint_command_msg.position.at(0) = M_PI_4;
    joint_command_pub_->publish(joint_command_msg);
    std::cout << "\033[32m INPUT ACCEPTED \033[m" << std::endl;
  } 

  std::cout << std::endl << "Waiting command... " << std::endl;
}


void SystemCommandInterface::joyCallback(const sensor_msgs::msg::Joy & joy_cmd)
{
#if DEBUG_ENABLED
  std::cout << "joyCallback" << std::endl;
#endif  // DEBUG_ENABLED
  input_joy_cmd = joy_cmd;  // To subscribe topic "/joy" in a certain rate.
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemCommandInterface>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
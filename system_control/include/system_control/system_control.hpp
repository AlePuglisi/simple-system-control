#ifndef SYSTEM_CONTROL__SYSTEM_CONTROL_HPP_
#define SYSTEM_CONTROL__SYSTEM_CONTROL_HPP_

#include <chrono>
#include <cstdio>
#include <string>
#include <math.h>
#include <iostream>
#include <fstream>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <system_control_parameter.hpp>


class SystemControl : public rclcpp::Node
{
public:
  SystemControl();

  std::fstream file_out; 

private:

  /**
   * @brief This function runs periodically when called by the control_loop_timer_.
   *        It publish the torque control action to the effort_controller.
   */
  void controlLoop();

  /**
   * @brief This callback function runs when IJC subscribes current joint state from Gazebo.
   *
   * @param joint_pose_msg
   */
  void currentJointStateCallback(
    const sensor_msgs::msg::JointState & joint_state);
  /**
   * @brief This callback function runs when IJC subscribes target joint state from LC.
   *
   * @param EE_pose_msg
   */
  void referenceJointStateCallback(
    const sensor_msgs::msg::JointState & joint_state);

  // Timer
  rclcpp::TimerBase::SharedPtr control_loop_timer_; // sets digital controller sampling time Ts

  //  Publisher
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_control_pub_; // computed control output torque

  // Subscriber
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_reference_sub_; // received LC set points
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr current_joint_state_sub_; // received joint state from Gazebo node

  // attributes
  int Ts; // controller period (fs = 1/Ts) [ms]

  float torque_limit; // Saturation limit of joint motors
  float velocity_limit; 

  float previous_reference_joint_position[JOINT_NUM]; // store previous reference for numerical derivation
  float previous_reference_joint_position_filtered[JOINT_NUM];
  float reference_joint_position_filtered[JOINT_NUM];
  float joint_velocity_feed_forward[JOINT_NUM];

  float anti_wind_up[JOINT_NUM];
  float integral[JOINT_NUM]; 

  double initial_time; 

  // messages updated by controller
  sensor_msgs::msg::JointState current_joint_state; // store current joint state from gazebo
  sensor_msgs::msg::JointState reference_joint_state; // store current joint reference from LC
  std_msgs::msg::Float64MultiArray torque_control_output; // store torque control values
};

#endif  // LBR_INDIPENDENT_JOINT_CONTROLLER__LBR_INDIPENDENT_JOINT_CONTROLLER_HPP_
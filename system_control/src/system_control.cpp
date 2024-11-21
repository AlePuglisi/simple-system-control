#include "system_control/system_control.hpp"

SystemControl::SystemControl()
: Node("system_control")
{
  std::cout << "SystemControl class is established." << std::endl;

  // initialize variables and resize
  current_joint_state.position.resize(JOINT_NUM);
  current_joint_state.velocity.resize(JOINT_NUM);
  reference_joint_state.position.resize(JOINT_NUM);
  reference_joint_state.velocity.resize(JOINT_NUM);

  torque_control_output.data.resize(JOINT_NUM);

  Ts = 2; //ms

  torque_limit = 8.0; // Nm limit of motor joint  
  velocity_limit = 3.0; // rad/s limit of motor joint 
  
  file_out.open("/home/ale/ros2_ws/src/SIMPLE_SYSTEM_CONTROL/system_control/data_record.csv",std::ofstream::out | std::ofstream::trunc);
  file_out << "time,reference_position,reference_position_filtered,current_position,position_error,reference_velocity,current_velocity,feed_forward_velocity,velocity_error,integral_action,proportional_action,control_torque,\n";

  // initialize reference joint state, only F2T as pi/2 initial desired position
  for (int i=0; i < JOINT_NUM; i++){
    previous_reference_joint_position[i] = 0.0;
    reference_joint_position_filtered[i] = 0.0;
    previous_reference_joint_position_filtered[i] = 0.0;
    reference_joint_state.position.at(i) = 0.0; 
    joint_velocity_feed_forward[i] = 0.0;

    integral[i] = 0.0; 
    anti_wind_up[i] = 0.0; // anti wind-up action
  }

  // initialize Publisher and Subscirbers
  current_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 1,
    std::bind(&SystemControl::currentJointStateCallback, this, std::placeholders::_1));

  joint_state_reference_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
     "/trajectory_generation/joint_state", 1,
    std::bind(&SystemControl::referenceJointStateCallback, this, std::placeholders::_1));

  torque_control_pub_ = this-> create_publisher<std_msgs::msg::Float64MultiArray>(
    "/effort_controller/commands", 10);

  // Set up control loop rate
  control_loop_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(Ts), std::bind(&SystemControl::controlLoop, this));
  initial_time = this->get_clock()->now().seconds();
}

void SystemControl::currentJointStateCallback(
    const sensor_msgs::msg::JointState & joint_state)
{
  // update current joint state with Gazebo feedback
  for (int i=0; i < JOINT_NUM; i++){
    current_joint_state.position.at(i) = joint_state.position.at(i);
    current_joint_state.velocity.at(i) = joint_state.velocity.at(i);
  }
}

void SystemControl::referenceJointStateCallback(
    const sensor_msgs::msg::JointState & desired_joint_state)
{
  // update reference joint state (set point) with LC control signals
  for (int i=0; i < JOINT_NUM; i++){
    reference_joint_state.position.at(i) = desired_joint_state.position.at(i);
    reference_joint_state.velocity.at(i) = desired_joint_state.velocity.at(i);
  }
}

void SystemControl::controlLoop()
{
  // compute control action
  // definition of support variables
  float joint_position_error[JOINT_NUM];
  float joint_velocity_error[JOINT_NUM];
  float joint_velocity_reference[JOINT_NUM];

  // computation of all control signal components
  for (int i=0; i < JOINT_NUM; i++){
    // dq_d(t) = kpp*(q_d(t) - q(t)) + dq_ff(t), Proportional position controller + feedforwaed velocity
    reference_joint_position_filtered[i] = reference_joint_position_filtered[i]*(Tf-(Ts*1e-3))/Tf + (Ts*1e-3)/Tf * previous_reference_joint_position[i];

    //joint_position_error[i] = (reference_joint_state.position.at(i) - current_joint_state.position.at(i));
    joint_position_error[i] = (reference_joint_position_filtered[i] - current_joint_state.position.at(i));
    
    //joint_velocity_feed_forward[i] = (reference_joint_position_filtered[i] - previous_reference_joint_position_filtered[i]) / (Ts*1e-3); // Ts is in [ms], convert into [s]
    joint_velocity_feed_forward[i] = joint_velocity_feed_forward[i]*(1-(Ts*1e-3)*(1/Tdf)) + (reference_joint_position_filtered[i] - previous_reference_joint_position_filtered[i])*(1/Tdf) ;

    if (joint_velocity_feed_forward[i] > velocity_limit){
      joint_velocity_feed_forward[i] = velocity_limit;
    } else if(joint_velocity_feed_forward[i] < -velocity_limit ){
      joint_velocity_feed_forward[i] = -velocity_limit;
    }
    
    joint_velocity_reference[i] = joint_position_error[i] * Kpp + joint_velocity_feed_forward[i];

    // tau(t) = kpv*(dq_d(t) - dq(t)) + Ts*(dq_d(t) - dq(t))*kpv/Tiv, Propodtional Integral controller + gravity compensation (to estimate)
    joint_velocity_error[i] = joint_velocity_reference[i] - current_joint_state.velocity.at(i);
    integral[i] += (Ts*1e-3)*(joint_velocity_error[i]*Kpv/Tiv + anti_wind_up[i]);

    torque_control_output.data.at(i) = Kpv * joint_velocity_error[i] + integral[i] - 1*9.81*0.5*sin(current_joint_state.position.at(i));

    // Anti Wind-Up action, with CONDITIONAL CLAMPING 
    // if((joint_velocity_error[i] > 0 && torque_control_output.data.at(i) > torque_limit)){
    //   anti_wind_up[i] = 0;
    //   torque_control_output.data.at(i) = torque_limit; // Saturate actuator command
    // }
    // if((joint_velocity_error[i] < 0 && torque_control_output.data.at(i) < -torque_limit)){
    //   anti_wind_up[i] = 0;
    //   torque_control_output.data.at(i) = -torque_limit; // Saturate actuator command
    // }
    // else{
    //   anti_wind_up[i] = 1;
    // }

    anti_wind_up[i] = (torque_limit-torque_control_output.data.at(i))*1/Tc;

    // SATURATION OF ACTUATOR
    if(torque_control_output.data.at(i) > torque_limit){
      torque_control_output.data.at(i) = torque_limit;
    } else if(torque_control_output.data.at(i)<-torque_limit){
      torque_control_output.data.at(i) = -torque_limit;
    }

    previous_reference_joint_position[i] = reference_joint_state.position.at(i); // store reference joint state for next iteration
    previous_reference_joint_position_filtered[i] = reference_joint_position_filtered[i];
  }
  
  torque_control_pub_->publish(torque_control_output);

  double current_time = this->get_clock()->now().seconds() - initial_time;

  file_out << current_time << ","
           << reference_joint_state.position.at(0) << ","
           << reference_joint_position_filtered[0] << ","
           << current_joint_state.position.at(0)   << ","
           << joint_position_error[0]              << ","
           << joint_velocity_reference[0]          << ","
           << current_joint_state.velocity.at(0)   << ","
           << joint_velocity_feed_forward[0]       << ","
           << joint_velocity_error[0]              << ","
           << integral[0]                          << ","
           << joint_velocity_error[0]*Kpv          << ","
           << torque_control_output.data.at(0)     << ",\n";
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  node->file_out.close();

  return 0;
}

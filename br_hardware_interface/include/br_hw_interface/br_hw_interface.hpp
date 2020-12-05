//
// Created by michael on 7/17/20.
//

#ifndef BR_HARDWARE_INTERFACE_BR_HW_INTERFACE_HPP
#define BR_HARDWARE_INTERFACE_BR_HW_INTERFACE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <numeric>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tss_msgs/msg/tss_state.hpp"
#include "br_hardware_interface_msgs/msg/hardware_interface_debug.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "usb_hw_interface/arduino_com.hpp"
#include "br_hw_interface/pid.hpp"

inline double to_distance(int pulse_us, double speed_of_sound)
{
  return static_cast<double>(pulse_us) / 2000000.0 * speed_of_sound;
}

class BRHardwareInterface : public rclcpp::Node
{
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point last_time_;    //! Time of last update

  void configure_parameters();

  void read(std::chrono::steady_clock::duration elapsed_time);
  void write(std::chrono::steady_clock::duration elapsed_time);
  void update();

  HardwareCom connection;
  PID left_pid_, right_pid_;

  /// Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr front_dist_pub_, front_left_dist_pub_,
    front_right_dist_pub_, front_left_bottom_dist_pub_, front_right_bottom_dist_pub_,
    left_dist_pub_,
    right_dist_pub_, rear_dist_pub_, rear_left_dist_pub_, rear_right_dist_pub_,
    rear_left_bottom_dist_pub_, rear_right_bottom_dist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr num_satellites_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gps_speed_pub_, gps_angle_pub_;
  rclcpp::Publisher<tss_msgs::msg::TSSState>::SharedPtr tss_pub_;
  rclcpp::Publisher<br_hardware_interface_msgs::msg::HardwareInterfaceDebug>::SharedPtr debug_pub_;

  /// Subscribers
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  void cmd_vel_cb(geometry_msgs::msg::Twist::SharedPtr msg);

  /// Dynamically reconfigurable parameters
  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

  /// Parameters
  int encoder_ticks_per_rot_ {};
  double speed_of_sound_ {};
  double wheel_diameter_ {}, wheel_separation_ {};
  std::string left_wheel_joint_name_, right_wheel_joint_name_;

  /// State Variables
  long initial_left_encoder_position_, initial_right_encoder_position_;
  double last_left_encoder_position_, last_right_encoder_position_;
  std::array<double, 10> left_encoder_history_, right_encoder_history_;
  double left_wheel_angular_velocity_ {}, right_wheel_angular_velocity_ {};
  double velocity_x_ {}, velocity_theta_ {}; //! Set points from cmd_vel
  double control_deadzone_;
  std::string port_; ///! Port name for debug purposes
  bool debug_;
  br_hardware_interface_msgs::msg::HardwareInterfaceDebug debug_msg_;

  float last_longitude_, last_latitude_, last_altitude_;

  double tss_thickness_;

public:
  explicit BRHardwareInterface(const std::string & port = "/dev/ttyACM0");
};

#endif //BR_HARDWARE_INTERFACE_BR_HW_INTERFACE_HPP

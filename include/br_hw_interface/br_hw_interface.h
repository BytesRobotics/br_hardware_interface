//
// Created by michael on 7/17/20.
//

#ifndef BR_HARDWARE_INTERFACE_BR_HW_INTERFACE_H
#define BR_HARDWARE_INTERFACE_BR_HW_INTERFACE_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "usb_hw_interface/arduino_com.h"
#include "br_hw_interface/pid.h"

class BRHardwareInterface : public rclcpp::Node{
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point last_time_;  //! Time of last update

    void configure_parameters();

    void read(std::chrono::steady_clock::duration elapsed_time, std::chrono::steady_clock::time_point now);
    void write(std::chrono::steady_clock::duration elapsed_time, std::chrono::steady_clock::time_point now);
    void update();

    HardwareCom connection;
    PID left_pid_, right_pid_;

    /// Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;

    /// Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    void cmd_vel_cb(geometry_msgs::msg::Twist::SharedPtr msg);


    /// Dynamically reconfigurable parameters
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;

    /// Parameters
    int encoder_ticks_per_rot_;
    double speed_of_sound_;
    double wheel_diameter_, wheel_separation_;
    std::string left_wheel_joint_name_, right_wheel_joint_name_;

    /// State Variables
    long initial_left_encoder_position_, initial_right_encoder_position_;
    double last_left_encoder_position_, last_right_encoder_position_;
    double left_wheel_angular_velocity_, right_wheel_angular_velocity_;
    double velocity_x_, velocity_theta_; //! Set points from cmd_vel
    double last_left_motor_cmd_, last_right_motor_cmd_;

public:
    BRHardwareInterface();
};

#endif //BR_HARDWARE_INTERFACE_BR_HW_INTERFACE_H

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

#include "usb_hw_interface/arduino_com.h"

class BRHardwareInterface : public rclcpp::Node{
    rclcpp::TimerBase::SharedPtr timer_;
    std::chrono::steady_clock::time_point last_time_;  //! Time of last update

    void read();
    void write(std::chrono::steady_clock::duration elapsed_time, std::chrono::steady_clock::time_point now);
    void update();

    HardwareCom connection;

public:
    BRHardwareInterface();
};

#endif //BR_HARDWARE_INTERFACE_BR_HW_INTERFACE_H

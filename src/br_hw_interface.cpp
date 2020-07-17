//
// Created by michael on 7/17/20.
//

#include "br_hw_interface/br_hw_interface.h"

using namespace std::chrono_literals;

BRHardwareInterface::BRHardwareInterface():
Node("hardware_interface"),
connection("/dev/ttyACM1", 115200)
{
    connection.set_controller(0, 0, 0);
    usleep(30000);
    while(!connection.read_controller()){
        RCLCPP_ERROR(this->get_logger(), "Could not read hardware controller from /dev/ttyACM0");
    }

    /// Start reading from and updating the hardware controller
    timer_ = this->create_wall_timer(20ms, std::bind(&BRHardwareInterface::update, this));
}

void BRHardwareInterface::update()
{
    read(); //! Read

    auto now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::duration elapsed_time = now - last_time_;

    write(elapsed_time, now); //! Write

    last_time_ = std::chrono::steady_clock::now();
}

void BRHardwareInterface::read() {

    while(!connection.read_controller()){
        RCLCPP_DEBUG(this->get_logger(), "Could not read hardware controller from /dev/ttyACM0");
    }
}

void BRHardwareInterface::write(std::chrono::steady_clock::duration elapsed_time,
        std::chrono::steady_clock::time_point now) {

}
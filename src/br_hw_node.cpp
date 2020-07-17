//
// Created by michael on 7/17/20.
//

#include "br_hw_interface/br_hw_interface.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BRHardwareInterface>());
    rclcpp::shutdown();
    return 0;
}
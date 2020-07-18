//
// Created by michael on 7/17/20.
//

#include "br_hw_interface/br_hw_interface.h"

int main(int argc, char * argv[])
{
    std::string port = "/dev/ttyACM0";
    if(argc > 1){
        port = argv[1];
    }
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BRHardwareInterface>(port));
    rclcpp::shutdown();
    return 0;
}
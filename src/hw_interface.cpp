#include "usb_hw_interface/arduino_com.h"
#include <unistd.h>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ros/console.h>

// Helper functions
template <class T>
T map(T input, T inMin, T inMax, T outMin, T outMax){
    T output = (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    return output;
}

template <class T>
T constrain(T value, T min, T max){
    if(value > max){
        value = max;
    } else if(value < min){
        value = min;
    }
    return value;
}

double rmotor_cmd, lmotor_cmd, rwheel_encoder, lwheel_encoder;

void update_lmotor_cmd(const std_msgs::Float64::ConstPtr& msg){
}

void update_rmotor_cmd(const std_msgs::Float64::ConstPtr& msg){
}


int main(int argc, char * argv[])
{
   ros::init(argc, argv, "hardware_interface");
   ros::NodeHandle nh;

   HardwareCom connection("/dev/ttyACM0", 115200);

   ros::Subscriber rmotor_sub = nh.subscribe("rmotor_cmd", 1, update_rmotor_cmd);
   ros::Subscriber lmotor_sub = nh.subscribe("lmotor_cmd", 1, update_lmotor_cmd);

   ros::Publisher rwheel_encoder_pub = nh.advertise<std_msgs::Float64>("rwheel_encoder", 1);;
   ros::Publisher lwheel_encoder_pub = nh.advertise<std_msgs::Float64>("lwheel_encoder", 1);;

   ros::Rate rate(50); //50Hz
   while(ros::ok()){
//      connection.setController(0.5,-0.5,25);
      connection.readController();
      std::cout << std::bitset<8>(connection.getCh1()) << "\n";
      ros::spinOnce();
      rate.sleep();
  }
}
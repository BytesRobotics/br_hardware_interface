#include "usb_hw_interface/arduino_com.h"
#include <unistd.h>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
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

//Distance = Speed * Time/2
// speed of sound (m/s) = 331.5 + 0.60 T(°C)
float speed_of_sound = 343.0; // will be recomputed based on temperature readings (should be in m/s)

inline float to_distance(int pulse_us) {
    return pulse_us / 2000000.0 * speed_of_sound;
}

double rmotor_cmd, lmotor_cmd, rwheel_encoder, lwheel_encoder;

void update_lmotor_cmd(const std_msgs::Float64::ConstPtr& msg){
    lmotor_cmd = msg->data;
}

void update_rmotor_cmd(const std_msgs::Float64::ConstPtr& msg){
    rmotor_cmd = msg->data;
}

int main(int argc, char * argv[])
{
   ros::init(argc, argv, "hardware_interface");
   ros::NodeHandle nh;

   HardwareCom connection("/dev/ttyACM0", 115200);

   ros::Subscriber rmotor_sub = nh.subscribe("rmotor_cmd", 1, update_rmotor_cmd);
   ros::Subscriber lmotor_sub = nh.subscribe("lmotor_cmd", 1, update_lmotor_cmd);

   ros::Publisher rwheel_encoder_pub = nh.advertise<std_msgs::Int32>("rwheel_encoder", 1);
   ros::Publisher lwheel_encoder_pub = nh.advertise<std_msgs::Int32>("lwheel_encoder", 1);

    //  front_dist_pin  CH1
    //  left_dist_pin   CH2
    //  right_dist_pin  CH3
    //  rear_dist_pin   CH4
    //  bottom_dist_pin CH5
    ros::Publisher front_distance_pub = nh.advertise<std_msgs::Float64>("distance/front", 1);
    ros::Publisher rear_distance_pub = nh.advertise<std_msgs::Float64>("distance/rear", 1);
    ros::Publisher right_distance_pub = nh.advertise<std_msgs::Float64>("distance/right", 1);
    ros::Publisher left_distance_pub = nh.advertise<std_msgs::Float64>("distance/left", 1);
    ros::Publisher bottom_distance_pub = nh.advertise<std_msgs::Float64>("distance/bottom", 1);

    ros::Publisher nav_sat_pub = nh.advertise<sensor_msgs::NavSatFix>("gps", 1);
    ros::Publisher nav_sat_speed_pub = nh.advertise<std_msgs::Float64>("gps/speed", 1);
    ros::Publisher nav_sat_angle_pub = nh.advertise<std_msgs::Float64>("gps/angle", 1);
    ros::Publisher nav_sat_satellites_pub = nh.advertise<std_msgs::Int32>("gps/satellites", 1);


   ros::Rate rate(50); //50Hz
   while(ros::ok()){
      connection.setController(rmotor_cmd,lmotor_cmd);
      if(connection.readController()){

          std_msgs::Int32 encoder;
          encoder.data = connection.getEncoderRight();
          rwheel_encoder_pub.publish(encoder);
          encoder.data = connection.getEncoderLeft();
          lwheel_encoder_pub.publish(encoder);

          std_msgs::Float64 dist;
          dist.data = to_distance(connection.getCh1());
          front_distance_pub.publish(dist);
          dist.data = to_distance(connection.getCh4());
          rear_distance_pub.publish(dist);
          dist.data = to_distance(connection.getCh3());
          right_distance_pub.publish(dist);
          dist.data = to_distance(connection.getCh2());
          left_distance_pub.publish(dist);
          dist.data = to_distance(connection.getCh5());
          bottom_distance_pub.publish(dist);

          sensor_msgs::NavSatFix navSat;
          navSat.header.stamp = ros::Time::now();;
          navSat.latitude = connection.getLatitude();
          navSat.longitude = connection.getLongitude();
          navSat.altitude = connection.getAltitude();
          navSat.status.status = connection.getFixQuality();
          navSat.status.service = 1;
          navSat.position_covariance[0] = 0.1;
          navSat.position_covariance[3] = 0.1;
          navSat.position_covariance[6] = 0.1;
          navSat.position_covariance_type = 2;
          nav_sat_pub.publish(navSat);

          std_msgs::Int32 num_satellites;
          num_satellites.data = connection.getNumSatellites();
          nav_sat_satellites_pub.publish(num_satellites);
          std_msgs::Float64 speed, angle;
          speed.data = connection.getSpeed();
          nav_sat_speed_pub.publish(speed);
          angle.data = connection.getAngle();
          nav_sat_angle_pub.publish(angle);

      }
      ros::spinOnce();
      rate.sleep();
  }
}
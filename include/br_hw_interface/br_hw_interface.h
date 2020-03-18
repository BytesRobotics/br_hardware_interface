#ifndef ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H
#define ROS_CONTROL__ROBOT_HARDWARE_INTERFACE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include "usb_hw_interface/arduino_com.h"
#include <unistd.h>
#include <cmath>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <br_hw_interface/br_hw.h>
#include <control_toolbox/pid.h>
#include <std_msgs/Bool.h>


using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

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

inline float to_distance(int pulse_us, float speed_of_sound) {
    return pulse_us / 2000000.0 * speed_of_sound;
}


namespace br_hardware_interface
{
//    static const double POSITION_STEP_FACTOR = 10;
//    static const double VELOCITY_STEP_FACTOR = 10;

    class BRHardwareInterface: public br_hardware_interface::BRHardware
    {
    public:
        BRHardwareInterface(ros::NodeHandle& nh);
        ~BRHardwareInterface();
        void init();
        void update(const ros::TimerEvent& e);
        void read();
        void write(ros::Duration elapsed_time);

    protected:
        ros::NodeHandle nh_;
        ros::Timer non_realtime_loop_;
        ros::Duration control_period_;
        ros::Duration elapsed_time_;
        double loop_hz_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
        double p_error_, v_error_, e_error_;

        // Custom
        HardwareCom connection;

        ros::Publisher front_distance_pub;
        ros::Publisher rear_distance_pub;
        ros::Publisher right_distance_pub;
        ros::Publisher left_distance_pub;
        ros::Publisher bottom_distance_pub;

        ros::Publisher nav_sat_pub;
        ros::Publisher nav_sat_speed_pub;
        ros::Publisher nav_sat_angle_pub;
        ros::Publisher nav_sat_satellites_pub;

        ros::Subscriber siren_sub;
        ros::Subscriber fan_sub;

        void siren_cb(const std_msgs::Bool::ConstPtr& msg);
        int siren_gpio;
        void fan_cb(const std_msgs::Bool::ConstPtr& msg);
        int fan_gpio;

        double speed_of_sound; // will be recomputed based on temperature readings (should be in m/s)
        double encoder_ticks_per_rot;

        // Values for the head
        double head_position; //Since the head does not have feedback we simply provide the command after being processed
        double head_max_pose; //maximum position value for the head motion
        double head_min_pose; //minimum position value for the head motion
        double head_zero_pose; //zero position for the head (ie when cmd is zero)

        // For calculating joint velocity
        double last_right_encoder_read;
        double last_right_encoder_position;
        double right_encoder_zero;
        double current_right_velocity; // The velocity of the joint
        double last_left_encoder_read;
        double last_left_encoder_position;
        double left_encoder_zero;
        double current_left_velocity; // The velocity of the joint

        // For left and right wheel PIDs
        bool is_first_pass; //first pass sets state variables to prevent oscillation at start
        ros::NodeHandle nhp_;
        ros::Time last_cmd_time_;
        control_toolbox::Pid right_wheel_pid_;
        control_toolbox::Pid left_wheel_pid_;

        // For checking change in GPS data
        float last_latitude;
        float last_longitude;
        float last_altitude;

    };

}

#endif
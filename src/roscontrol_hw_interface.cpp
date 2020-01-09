#include <sstream>
#include <gb_hw_interface/gb_hw_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include "usb_hw_interface/arduino_com.h"
#include <unistd.h>
#include <cmath>
#include <math.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsInterface;

//https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot

namespace gb_hardware_interface
{
    GBHardwareInterface::GBHardwareInterface(ros::NodeHandle& nh) : nh_(nh), connection("/dev/ttyACM0", 115200){

        // Setup sensor value publishers
        front_distance_pub = nh_.advertise<std_msgs::Float64>("distance/front", 1);
        rear_distance_pub = nh_.advertise<std_msgs::Float64>("distance/rear", 1);
        right_distance_pub = nh_.advertise<std_msgs::Float64>("distance/right", 1);
        left_distance_pub = nh_.advertise<std_msgs::Float64>("distance/left", 1);
        bottom_distance_pub = nh_.advertise<std_msgs::Float64>("distance/bottom", 1);

        nav_sat_pub = nh_.advertise<sensor_msgs::NavSatFix>("gps", 1);
        nav_sat_speed_pub = nh_.advertise<std_msgs::Float64>("gps/speed", 1);
        nav_sat_angle_pub = nh_.advertise<std_msgs::Float64>("gps/angle", 1);
        nav_sat_satellites_pub = nh_.advertise<std_msgs::Int32>("gps/satellites", 1);

        nh_.param("/gb/hardware_interface/speed_of_sound", speed_of_sound, 343.0);
        nh_.param("/gb/hardware_interface/encoder_ticks_per_rot", encoder_ticks_per_rot, 374.0);

        init();

        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("/gb/hardware_interface/loop_hz", loop_hz_, 0.1);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &GBHardwareInterface::update, this);
    }

    GBHardwareInterface::~GBHardwareInterface() {

    }

    void GBHardwareInterface::init() {
        // Get joint names
        nh_.getParam("/gb/hardware_interface/joints", joint_names_);
        num_joints_ = joint_names_.size();

        // Resize vectors
        joint_position_.resize(num_joints_);
        joint_velocity_.resize(num_joints_);
        joint_effort_.resize(num_joints_);
        joint_position_command_.resize(num_joints_);
        joint_velocity_command_.resize(num_joints_);
        joint_effort_command_.resize(num_joints_);

        connection.setController(0,0,0);
        while(!connection.readController()){ROS_ERROR("Could not read hardware controller from /dev/ttyACM0");}

        // Initialize Controller
        for (int i = 0; i < num_joints_; ++i) {

            // Create joint state interface
            JointStateHandle jointStateHandle(joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(jointStateHandle);

            // Create velocity joint interface
            JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
            JointLimits limits;
            SoftJointLimits softLimits;
            getJointLimits(joint_names_[i], nh_, limits);
            VelocityJointSoftLimitsHandle jointLimitsHandle(jointVelocityHandle, limits, softLimits);
            velocity_joint_limits_interface_.registerHandle(jointLimitsHandle);
            velocity_joint_interface_.registerHandle(jointVelocityHandle);

            // Create effort joint interface
            JointHandle jointEffortHandle(jointStateHandle, &joint_effort_command_[i]);
            effort_joint_interface_.registerHandle(jointEffortHandle);
        }

        registerInterface(&joint_state_interface_);
        registerInterface(&velocity_joint_interface_);
        registerInterface(&effort_joint_interface_);
        registerInterface(&velocity_joint_limits_interface_);
    }

    void GBHardwareInterface::update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void GBHardwareInterface::read() {

        while(!connection.readController()){ROS_ERROR("Could not read hardware controller from /dev/ttyACM0");}

        for (int i = 0; i < num_joints_; i++) {
            if(joint_names_[i] == "left_wheel_joint"){
                joint_position_[i] = -1*connection.getEncoderLeft()/encoder_ticks_per_rot*2*M_PI;
            } else if (joint_names_[i] == "right_wheel_joint") {
                joint_position_[i] = -1*connection.getEncoderRight()/encoder_ticks_per_rot*2*M_PI;
            }
        }

        std_msgs::Float64 dist;
        dist.data = to_distance(connection.getCh1(), speed_of_sound);
        front_distance_pub.publish(dist);
        dist.data = to_distance(connection.getCh4(), speed_of_sound);
        rear_distance_pub.publish(dist);
        dist.data = to_distance(connection.getCh3(), speed_of_sound);
        right_distance_pub.publish(dist);
        dist.data = to_distance(connection.getCh2(), speed_of_sound);
        left_distance_pub.publish(dist);
        dist.data = to_distance(connection.getCh5(), speed_of_sound);
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

    void GBHardwareInterface::write(ros::Duration elapsed_time) {
        velocity_joint_limits_interface_.enforceLimits(elapsed_time);
        double left_motor_cmd;
        double right_motor_cmd;
        for (int i = 0; i < num_joints_; i++) {
            if(joint_names_[i] == "left_wheel_joint"){
                left_motor_cmd = joint_velocity_command_[i];
            } else if (joint_names_[i] == "right_wheel_joint") {
                right_motor_cmd = joint_velocity_command_[i];
            }
        }
        ROS_DEBUG_STREAM("Raw right: " << right_motor_cmd << " Raw left: " << left_motor_cmd);
        left_motor_cmd = -1.0*constrain(static_cast<int>(map(left_motor_cmd, -1.0, 1.0, -1000.0, 1000.0)), -1000, 1000);
        right_motor_cmd = constrain(static_cast<int>(map(right_motor_cmd, -1.0, 1.0, -1000.0, 1000.0)), -1000, 1000);
        ROS_DEBUG_STREAM("Processed right: " << right_motor_cmd << " Processed left: " << left_motor_cmd);
        connection.setController(right_motor_cmd,left_motor_cmd,0); //head servo currently zero
    }
}
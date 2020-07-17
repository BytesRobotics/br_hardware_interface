#include <sstream>
#include <br_hw_interface/br_hw_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include "usb_hw_interface/arduino_com.h"
#include "gpio/gpio.h"
#include <unistd.h>
#include <cmath>
#include <math.h>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::VelocityJointSoftLimitsHandle;
using joint_limits_interface::VelocityJointSoftLimitsInterface;

//https://slaterobots.com/blog/5abd8a1ed4442a651de5cb5b/how-to-implement-ros_control-on-a-custom-robot

namespace br_hardware_interface
{
   BRHardwareInterface::BRHardwareInterface(ros::NodeHandle& nh) : nh_(nh), connection("/dev/ttyACM0", 115200){

        // Setup joint state velocity stuff
        is_first_pass = true; //after all state variables are set control loop begins
        last_right_encoder_read = ros::Time::now().toSec();
        last_right_encoder_position = 0;
        current_right_velocity = 0;
        last_left_encoder_read = ros::Time::now().toSec();
        last_left_encoder_position = 0;
        current_left_velocity = 0;

        // Setup left and right wheel PIDs
        if (!nh_.hasParam("left_wheel_pid/p"))
            nh_.setParam("left_wheel_pid/p", 0.15);
        if (!nh_.hasParam("left_wheel_pid/i"))
            nh_.setParam("left_wheel_pid/i", 4.5);
        if (!nh_.hasParam("left_wheel_pid/d"))
            nh_.setParam("left_wheel_pid/d", 0.0);
        if (!nh_.hasParam("left_wheel_pid/i_clamp_min"))
            nh_.setParam("left_wheel_pid/i_clamp_min", -5.0);
        if (!nh_.hasParam("left_wheel_pid/i_clamp_max"))
            nh_.setParam("left_wheel_pid/i_clamp_max", 5.0);
        if (!nh_.hasParam("left_wheel_pid/antiwindup"))
            nh_.setParam("left_wheel_pid/antiwindup", true);

        if (!nh_.hasParam("right_wheel_pid/p"))
            nh_.setParam("right_wheel_pid/p", 0.15);
        if (!nh_.hasParam("right_wheel_pid/i"))
            nh_.setParam("right_wheel_pid/i", 4.5);
        if (!nh_.hasParam("right_wheel_pid/d"))
            nh_.setParam("right_wheel_pid/d", 0.0);
        if (!nh_.hasParam("right_wheel_pid/i_clamp_min"))
            nh_.setParam("right_wheel_pid/i_clamp_min", -5.0);
        if (!nh_.hasParam("right_wheel_pid/i_clamp_max"))
            nh_.setParam("right_wheel_pid/i_clamp_max", 5.0);
        if (!nh_.hasParam("right_wheel_pid/antiwindup"))
            nh_.setParam("right_wheel_pid/antiwindup", true);

        nh_.setParam("left_wheel_pid/publish_state", true);
        nh_.setParam("right_wheel_pid/publish_state", true);

        left_wheel_pid_.init(ros::NodeHandle(nh_, "left_wheel_pid"), false);
        right_wheel_pid_.init(ros::NodeHandle(nh_, "right_wheel_pid"), false);

        // Set the max and min parameters for the head servo
        ros::param::param<double>("~head_max_pose", head_max_pose, 0.8);
        ros::param::param<double>("~head_min_pose", head_min_pose,-0.8);
        ros::param::param<double>("~head_zero_pose", head_zero_pose, 0.0);
        head_position = 0;

        //https://github.com/ros-controls/control_toolbox/blob/melodic-devel/src/pid.cpp
        //http://docs.ros.org/jade/api/control_toolbox/html/classcontrol__toolbox_1_1Pid.html
        // Pid (double p=0.0, double i=0.0, double d=0.0, double i_max=0.0, double i_min=-0.0)

        last_cmd_time_ = ros::Time::now();

        // Setup sensor value publishers
        front_distance_pub = nh_.advertise<sensor_msgs::Range>("distance/front", 1);
        rear_distance_pub = nh_.advertise<sensor_msgs::Range>("distance/rear", 1);
        right_distance_pub = nh_.advertise<sensor_msgs::Range>("distance/right", 1);
        left_distance_pub = nh_.advertise<sensor_msgs::Range>("distance/left", 1);
        bottom_distance_pub = nh_.advertise<sensor_msgs::Range>("distance/bottom", 1);

        nav_sat_pub = nh_.advertise<sensor_msgs::NavSatFix>("gps", 1, true); //GPS messages latch
        nav_sat_speed_pub = nh_.advertise<std_msgs::Float64>("gps/speed", 1);
        nav_sat_angle_pub = nh_.advertise<std_msgs::Float64>("gps/angle", 1);
        nav_sat_satellites_pub = nh_.advertise<std_msgs::Int32>("gps/satellites", 1);
        last_latitude = 0;
        last_longitude = 0;
        last_latitude = 0;

        // Setup GPIO controls for siren and fan
        siren_sub = nh_.subscribe("siren", 10, &BRHardwareInterface::siren_cb, this);
        fan_sub = nh_.subscribe("fan", 10, &BRHardwareInterface::fan_cb, this);

        // configure gpio
        // fan
        fan_gpio = 149;
        gpio_export(fan_gpio);
        // siren
        siren_gpio = 200;
        gpio_export(siren_gpio);

	ros::Duration(0.5).sleep(); //Give time for GPIO to be exported correctly
	gpio_set_dir(fan_gpio, 1);
	gpio_set_dir(siren_gpio, 1);

        nh_.param("/br/hardware_interface/speed_of_sound", speed_of_sound, 343.0);
        nh_.param("/br/hardware_interface/encoder_ticks_per_rot", encoder_ticks_per_rot, 374.0);

        init();

        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
        nh_.param("/br/hardware_interface/loop_hz", loop_hz_, 0.1);
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        non_realtime_loop_ = nh_.createTimer(update_freq, &BRHardwareInterface::update, this);

    }

    BRHardwareInterface::~BRHardwareInterface() {

    }

    void BRHardwareInterface::init() {
        // Get joint names
        nh_.getParam("/br/hardware_interface/joints", joint_names_);
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

            // Create position joint interface
            JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
            position_joint_interface_.registerHandle(jointPositionHandle);

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
        registerInterface(&position_joint_interface_);
        registerInterface(&velocity_joint_interface_);
        registerInterface(&effort_joint_interface_);
//        registerInterface(&velocity_joint_limits_interface_);
    }

    void BRHardwareInterface::update(const ros::TimerEvent& e) {
        elapsed_time_ = ros::Duration(e.current_real - e.last_real);
        read();
        controller_manager_->update(ros::Time::now(), elapsed_time_);
        write(elapsed_time_);
    }

    void BRHardwareInterface::read() {

        while(!connection.readController()){ROS_DEBUG("Could not read hardware controller from /dev/ttyACM0");}

        float encoder_position = 0;
        for (int i = 0; i < num_joints_; i++) {
            if(joint_names_[i] == "left_wheel_joint"){
                encoder_position = connection.getEncoderLeft()/encoder_ticks_per_rot*2*M_PI;
                ROS_DEBUG_STREAM("left encoder value: " << encoder_position);
                if(is_first_pass){left_encoder_zero = encoder_position;}
                //sanity check to eliminate rogue values
                if(abs(encoder_position - left_encoder_zero - last_left_encoder_position) < 10 || is_first_pass){
                    joint_position_[i] = encoder_position-left_encoder_zero;
                } else {
                    joint_position_[i] = last_left_encoder_position;
                }
                joint_velocity_[i] = (joint_position_[i] - last_left_encoder_position)/(ros::Time::now().toSec() - last_left_encoder_read);
                current_left_velocity = joint_velocity_[i];
                last_left_encoder_read = ros::Time::now().toSec();
                last_left_encoder_position = joint_position_[i];
            } else if (joint_names_[i] == "right_wheel_joint") {
                encoder_position = -connection.getEncoderRight()/encoder_ticks_per_rot*2*M_PI;
                ROS_DEBUG_STREAM("right encoder value: " << encoder_position);
                if(is_first_pass){right_encoder_zero = encoder_position;}
                //sanity check to eliminate rogue values
                if(abs(encoder_position - right_encoder_zero - last_right_encoder_position) < 10  || is_first_pass){
                    joint_position_[i] = encoder_position-right_encoder_zero;
                } else {
                    joint_position_[i] = last_right_encoder_position;
                }
                joint_velocity_[i] = (joint_position_[i] - last_right_encoder_position)/(ros::Time::now().toSec() - last_right_encoder_read);
                current_right_velocity = joint_velocity_[i];
                last_right_encoder_read = ros::Time::now().toSec();
                last_right_encoder_position = joint_position_[i];
            } else if (joint_names_[i] == "head_joint") {
                joint_position_[i] = head_position;
            }
        }

        sensor_msgs::Range dist;
        dist.radiation_type = dist.ULTRASOUND;
        dist.header.stamp = ros::Time::now();
        dist.field_of_view = 0.366519; //21 deg
        dist.min_range = 0.02;
        dist.max_range = 9.00;

        dist.range = to_distance(connection.getCh(0), speed_of_sound);
        dist.header.frame_id = "front_dist_sensor";
        front_distance_pub.publish(dist);
        dist.range = to_distance(connection.getCh(3), speed_of_sound);
        dist.header.frame_id = "rear_dist_sensor";
        rear_distance_pub.publish(dist);
        dist.range = to_distance(connection.getCh(2), speed_of_sound);
        dist.header.frame_id = "right_dist_sensor";
        right_distance_pub.publish(dist);
        dist.range = to_distance(connection.getCh(1), speed_of_sound);
        dist.header.frame_id = "left_dist_sensor";
        left_distance_pub.publish(dist);
        dist.range = to_distance(connection.getCh(4), speed_of_sound);
        dist.header.frame_id = "bottom_dist_sensor";
        bottom_distance_pub.publish(dist);

        sensor_msgs::NavSatFix navSat;
        navSat.header.stamp = ros::Time::now();
        navSat.header.frame_id = "gps_link";
        navSat.latitude = connection.getLatitude();
        navSat.longitude = connection.getLongitude();
        navSat.altitude = connection.getAltitude();
        navSat.status.status = connection.getFixQuality() - 1;
        navSat.status.service = 1;

        float hdop = connection.getHdop();
        ROS_DEBUG_STREAM("HDOP: " << hdop); // print the horizontal dilution of precision for debug

        // use DOP for variance calculations
        navSat.position_covariance[0] = 10.0*hdop;
        navSat.position_covariance[3] = 10.0*hdop;
        navSat.position_covariance[6] = 50.0;
        navSat.position_covariance_type = 2;

        //sanity check on data and limit pub rate first checking validity then change
        if(navSat.longitude != last_longitude || navSat.latitude != last_latitude || navSat.altitude != last_altitude){
            last_longitude = navSat.longitude;
            last_latitude = navSat.latitude;
            last_altitude = navSat.altitude;
            nav_sat_pub.publish(navSat);
        }

        std_msgs::Int32 num_satellites;
        num_satellites.data = connection.getNumSatellites();
        nav_sat_satellites_pub.publish(num_satellites);
        std_msgs::Float64 speed, angle;
        speed.data = connection.getSpeed();
        nav_sat_speed_pub.publish(speed);
        angle.data = connection.getAngle();
        nav_sat_angle_pub.publish(angle);
    }

    void BRHardwareInterface::write(ros::Duration elapsed_time) {

        velocity_joint_limits_interface_.enforceLimits(elapsed_time);

        double left_motor_cmd;
        double right_motor_cmd;
        double head_motor_cmd;
        for (int i = 0; i < num_joints_; i++) {
            if(joint_names_[i] == "left_wheel_joint"){
                left_motor_cmd = joint_velocity_command_[i];
            } else if (joint_names_[i] == "right_wheel_joint") {
                right_motor_cmd = joint_velocity_command_[i];
            } else if (joint_names_[i] == "head_joint") {
                head_motor_cmd = joint_position_command_[i];
            }
        }

        ROS_DEBUG_STREAM("Raw right: " << right_motor_cmd << " Raw left: " << left_motor_cmd);

        //Get value from PID controllers
        ros::Duration dt = ros::Time::now() - last_cmd_time_;
        // left motor command
        double error = left_motor_cmd - current_left_velocity;
        left_motor_cmd = left_wheel_pid_.computeCommand(error, dt);
        // right motor command
        error = right_motor_cmd - current_right_velocity;
        right_motor_cmd = right_wheel_pid_.computeCommand(error, dt);

        ROS_DEBUG_STREAM("PID right: " << right_motor_cmd << " PID left: " << left_motor_cmd << " Head cmd: " << head_motor_cmd);

        if(abs(right_motor_cmd)<0.05){
            right_motor_cmd = 0;
        }

        if(abs(left_motor_cmd)<0.05){
            left_motor_cmd = 0;
        }

        left_motor_cmd = -1.0*constrain(left_motor_cmd, -1.0, 1.0);
        right_motor_cmd = constrain(right_motor_cmd, -1.0, 1.0);

        head_position = constrain(head_motor_cmd, head_min_pose, head_max_pose); // kept in defined coordinate space + radians
        head_motor_cmd = constrain(head_motor_cmd + head_zero_pose, head_min_pose, head_max_pose); // constrain with zero position for servo
        head_motor_cmd = map(head_motor_cmd, -1.5708, 1.5708, -1.0, 1.0); //https://www.servocity.com/hs-646wp-servo bounds reflect +- 180 degrees

        ROS_DEBUG_STREAM("Processed right: " << right_motor_cmd << " Processed left: " << left_motor_cmd << " Processed head: " << head_motor_cmd);

        if(!is_first_pass) {
            connection.setController(right_motor_cmd, left_motor_cmd, head_motor_cmd); //head servo currently zero
        }
        is_first_pass = false;

        last_cmd_time_ = ros::Time::now();
    }

    void BRHardwareInterface::siren_cb(const std_msgs::Bool::ConstPtr& msg){
        ROS_DEBUG_STREAM("Toggle Siren");
        if(msg->data == true){gpio_set_value(siren_gpio, 1);}
        else {gpio_set_value(siren_gpio, 0);}
    }

    void BRHardwareInterface::fan_cb(const std_msgs::Bool::ConstPtr& msg){
        ROS_DEBUG_STREAM("Toggle Fan");
        if(msg->data == true){gpio_set_value(fan_gpio, 1);}
        else {gpio_set_value(fan_gpio, 0);}
    }
}

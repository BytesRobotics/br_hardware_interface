//
// Created by michael on 7/17/20.
//

#include "br_hw_interface/br_hw_interface.h"

using namespace std::chrono_literals;

BRHardwareInterface::BRHardwareInterface():
Node("hardware_interface"),
connection("/dev/ttyACM1", 115200)
{
    configure_parameters();

    joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
            rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable());

    cmd_vel_sub_ = this->create_subscription<std_msgs::msg::String>("cmd_vel", 10,
            std::bind(&BRHardwareInterface::cmd_vel_cb, this, std::placeholders::_1));

    connection.set_controller(0, 0, 0);
    usleep(30000);
    while(!connection.read_controller()){
        RCLCPP_ERROR(this->get_logger(), "Could not read hardware controller from /dev/ttyACM0");
    }
    last_left_encoder_position_ = static_cast<double>(connection.get_left_encoder())/encoder_ticks_per_rot_;
    last_right_encoder_position_ = static_cast<double>(connection.get_right_encoder())/encoder_ticks_per_rot_;
    initial_left_encoder_position_ = connection.get_left_encoder();
    initial_right_encoder_position_ = connection.get_right_encoder();

    /// Start reading from and updating the hardware controller
    timer_ = this->create_wall_timer(20ms, std::bind(&BRHardwareInterface::update, this));
}

void BRHardwareInterface::update()
{
    auto now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::duration elapsed_time = now - last_time_;

    read(elapsed_time, now);
    write(elapsed_time, now);

    last_time_ = now;
}

void BRHardwareInterface::read(std::chrono::steady_clock::duration elapsed_time, std::chrono::steady_clock::time_point now) {
    while(!connection.read_controller()){
        RCLCPP_DEBUG(this->get_logger(), "Could not read hardware controller from /dev/ttyACM0");
    }

    double left_encoder_position = static_cast<double>(connection.get_left_encoder() - initial_left_encoder_position_)/encoder_ticks_per_rot_;
    double right_encoder_position = static_cast<double>(connection.get_right_encoder() - initial_right_encoder_position_)/encoder_ticks_per_rot_;

    left_wheel_angular_velocity_ = (left_encoder_position - last_left_encoder_position_)/elapsed_time.count();
    right_wheel_angular_velocity_ = (right_encoder_position - last_right_encoder_position_)/elapsed_time.count();

    auto joint_states_msg = sensor_msgs::msg::JointState();
    joint_states_msg.header.stamp = this->now();
    joint_states_msg.name.push_back(left_wheel_joint_name_);
    joint_states_msg.position.push_back(left_encoder_position);
    joint_states_msg.velocity.push_back(left_wheel_angular_velocity_);
    joint_states_msg.name.push_back(right_wheel_joint_name_);
    joint_states_msg.position.push_back(right_encoder_position);
    joint_states_msg.velocity.push_back(right_wheel_angular_velocity_);
    joint_states_publisher_->publish(joint_states_msg);

    last_left_encoder_position_ = left_encoder_position;
    last_right_encoder_position_ = right_encoder_position;
}

void BRHardwareInterface::write(std::chrono::steady_clock::duration elapsed_time,
        std::chrono::steady_clock::time_point now) {
    /// Use PID and kinematics model to update motors
    double angular_velocity_left_setpoint;
    double angular_velocity_right_setpoint;

    /// Kinematics model
    if(velocity_x_ == 0){
        angular_velocity_right_setpoint = (velocity_theta_ * wheel_separation_/2)/(wheel_diameter_/2);
        angular_velocity_left_setpoint = -angular_velocity_right_setpoint;
    } else {
        angular_velocity_right_setpoint = (velocity_x_ + velocity_theta_/2)/(wheel_diameter_/2);
        angular_velocity_left_setpoint = (velocity_x_ - velocity_theta_/2)/(wheel_diameter_/2);
    }
    auto right_cmd = last_right_motor_cmd_ + right_pid_.update(angular_velocity_right_setpoint - right_wheel_angular_velocity_, elapsed_time);
    auto left_cmd = last_left_motor_cmd_ + left_pid_.update(angular_velocity_left_setpoint - left_wheel_angular_velocity_, elapsed_time);
    last_right_motor_cmd_ = right_cmd;
    last_left_motor_cmd_ = left_cmd;

    connection.set_controller(right_cmd, left_cmd,0);
}

void BRHardwareInterface::configure_parameters() {
    encoder_ticks_per_rot_ = this->declare_parameter("encoder_ticks_per_rot", 196);
    speed_of_sound_ = this->declare_parameter("speed_of_sound", 374.0);
    wheel_diameter_ = this->declare_parameter("wheel_diameter", 0.2032);
    wheel_separation_ = this->declare_parameter("wheel_separation", 0.4096);
    left_wheel_joint_name_ = this->declare_parameter("left_wheel_joint_name", "left_wheel_joint");
    right_wheel_joint_name_ = this->declare_parameter("right_wheel_joint_name", "right_wheel_joint");

    double p = this->declare_parameter("pid_p", 0.1);
    left_pid_.set_p(p);
    right_pid_.set_p(p);
    double i = this->declare_parameter("pid_i", 0.001);
    left_pid_.set_i(i);
    right_pid_.set_i(i);
    double d = this->declare_parameter("pid_d", 0.001);
    left_pid_.set_d(d);
    right_pid_.set_d(d);
    double max_i = this->declare_parameter("pid_i_max", 5.0);
    left_pid_.set_i_clamp_max(max_i);
    right_pid_.set_i_clamp_max(max_i);
    double min_i = this->declare_parameter("pid_i_min", -5.0);
    left_pid_.set_i_clamp_min(min_i);
    right_pid_.set_i_clamp_min(min_i);
    bool anti_windup = this->declare_parameter("pid_use_anti_windup", true);
    left_pid_.set_anti_windup(anti_windup);
    right_pid_.set_anti_windup(anti_windup);

    /// Update parameters dynamically
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
    auto on_parameter_event_callback = [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
        std::stringstream ss;
        ss << "\nParameter event:\n changed parameters:";
        for (auto & changed_parameter : event->changed_parameters) {
            ss << "\n  " << changed_parameter.name;
            if(changed_parameter.name == "encoder_ticks_per_rot"){
                encoder_ticks_per_rot_ = changed_parameter.value.integer_value;
            } else if(changed_parameter.name == "speed_of_sound"){
                speed_of_sound_ = changed_parameter.value.double_value;
            }  else if(changed_parameter.name == "wheel_diameter"){
                wheel_diameter_ = changed_parameter.value.double_value;
            } else if(changed_parameter.name == "wheel_separation"){
                wheel_separation_ = changed_parameter.value.double_value;
            } else if(changed_parameter.name == "left_wheel_joint_name"){
                left_wheel_joint_name_ = changed_parameter.value.string_value;
            } else if(changed_parameter.name == "right_wheel_joint_name"){
                right_wheel_joint_name_ = changed_parameter.value.string_value;
            } else if(changed_parameter.name == "pid_p"){
                left_pid_.set_p(changed_parameter.value.double_value);
                right_pid_.set_p(changed_parameter.value.double_value);
            } else if(changed_parameter.name == "pid_i"){
                left_pid_.set_i(changed_parameter.value.double_value);
                right_pid_.set_i(changed_parameter.value.double_value);
            } else if(changed_parameter.name == "pid_d"){
                left_pid_.set_d(changed_parameter.value.double_value);
                right_pid_.set_d(changed_parameter.value.double_value);
            } else if(changed_parameter.name == "pid_i_max"){
                left_pid_.set_i_clamp_max(changed_parameter.value.double_value);
                right_pid_.set_i_clamp_max(changed_parameter.value.double_value);
            } else if(changed_parameter.name == "pid_i_min"){
                left_pid_.set_i_clamp_min(changed_parameter.value.double_value);
                right_pid_.set_i_clamp_min(changed_parameter.value.double_value);
            } else if(changed_parameter.name == "pid_use_anti_windup"){
                left_pid_.set_anti_windup(changed_parameter.value.bool_value);
                right_pid_.set_anti_windup(changed_parameter.value.bool_value);
            }

        }
        ss << "\n";
        RCLCPP_DEBUG(this->get_logger(), ss.str().c_str());
    };

    /// Setup callback for changes to parameters.
    parameter_event_sub_ = parameters_client_->on_parameter_event(on_parameter_event_callback);
}

void BRHardwareInterface::cmd_vel_cb(const geometry_msgs::msg::Twist::SharedPtr msg) {
    velocity_x_ = msg->linear.x;
    velocity_theta_ = msg->angular.z;
}

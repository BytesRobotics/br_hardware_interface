//
// Created by michael on 7/17/20.
//

#include "br_hw_interface/br_hw_interface.h"

using namespace std::chrono_literals;

BRHardwareInterface::BRHardwareInterface(const std::string& port):
Node("hardware_interface"),
connection(port, 115200)
{
    configure_parameters();

    /// Publications
    joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",
            rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile().reliable());

    front_dist_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/front", rclcpp::SensorDataQoS());
    front_left_dist_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/front_left", rclcpp::SensorDataQoS());
    front_right_dist_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/front_right", rclcpp::SensorDataQoS());
    front_left_bottom_dist_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/front_left_bottom", rclcpp::SensorDataQoS());
    front_right_bottom_dist_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/front_right_bottom", rclcpp::SensorDataQoS());
    left_dist_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/left", rclcpp::SensorDataQoS());
    right_dist_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/right", rclcpp::SensorDataQoS());
    rear_dist_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/rear", rclcpp::SensorDataQoS());
    rear_left_dist_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/rear_left", rclcpp::SensorDataQoS());
    rear_right_dist_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/rear_right", rclcpp::SensorDataQoS());
    rear_left_bottom_dist_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/rear_left_bottom", rclcpp::SensorDataQoS());
    rear_right_bottom_dist_pub_ = this->create_publisher<sensor_msgs::msg::Range>("distance/rear_right_bottom", rclcpp::SensorDataQoS());

    nav_sat_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps", rclcpp::SystemDefaultsQoS().transient_local());
    num_satellites_pub_ = this->create_publisher<std_msgs::msg::Int32>("gps/satellites", rclcpp::SystemDefaultsQoS().transient_local());
    gps_angle_pub_ = this->create_publisher<std_msgs::msg::Float64>("gps/angle", rclcpp::SystemDefaultsQoS().transient_local());
    gps_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("gps/speed", rclcpp::SystemDefaultsQoS().transient_local());

    /// Subscriptions
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10,
            std::bind(&BRHardwareInterface::cmd_vel_cb, this, std::placeholders::_1));

    connection.set_controller(0, 0, 0);
    usleep(40000);
    while(!connection.read_controller()){
        RCLCPP_ERROR(this->get_logger(), "Could not read hardware controller from /dev/ttyACM0");
        usleep(20000);
    }
    last_left_encoder_position_ = static_cast<double>(connection.get_left_encoder())/encoder_ticks_per_rot_;
    last_right_encoder_position_ = static_cast<double>(connection.get_right_encoder())/encoder_ticks_per_rot_;
    initial_left_encoder_position_ = connection.get_left_encoder();
    initial_right_encoder_position_ = connection.get_right_encoder();

    /// Start reading from and updating the hardware controller
    // NOTE : Arduino loop must run at least 2x the frequency of listed here otherwise it will cap the speed!
    float frequency = this->declare_parameter("frequency", 50);
    int delay = static_cast<int>(1.0/frequency*1000);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(delay), std::bind(&BRHardwareInterface::update, this));
}

void BRHardwareInterface::update()
{
    auto now = std::chrono::steady_clock::now();
    std::chrono::steady_clock::duration elapsed_time = now - last_time_;

    read(elapsed_time);
    write(elapsed_time);

    last_time_ = now;
}

void BRHardwareInterface::read(std::chrono::steady_clock::duration elapsed_time) {
    auto time_stamp = this->now();

    while(!connection.read_controller()){
        RCLCPP_DEBUG(this->get_logger(), "Could not read hardware controller from /dev/ttyACM0");
    }

    double left_encoder_position = static_cast<double>(connection.get_left_encoder() - initial_left_encoder_position_)/encoder_ticks_per_rot_;
    double right_encoder_position = static_cast<double>(connection.get_right_encoder() - initial_right_encoder_position_)/encoder_ticks_per_rot_;

    left_wheel_angular_velocity_ = (left_encoder_position - last_left_encoder_position_)/(elapsed_time.count()/1000000000.0);
    right_wheel_angular_velocity_ = (right_encoder_position - last_right_encoder_position_)/(elapsed_time.count()/1000000000.0);

    auto joint_states_msg = sensor_msgs::msg::JointState();
    joint_states_msg.header.stamp = time_stamp;
    joint_states_msg.name.push_back(left_wheel_joint_name_);
    joint_states_msg.position.push_back(left_encoder_position);
    joint_states_msg.velocity.push_back(left_wheel_angular_velocity_);
    joint_states_msg.name.push_back(right_wheel_joint_name_);
    joint_states_msg.position.push_back(right_encoder_position);
    joint_states_msg.velocity.push_back(right_wheel_angular_velocity_);
    joint_states_publisher_->publish(joint_states_msg);

    last_left_encoder_position_ = left_encoder_position;
    last_right_encoder_position_ = right_encoder_position;

    /// Distance Sensors
    auto dist_msg = sensor_msgs::msg::Range();
    dist_msg.header.stamp = time_stamp;
    dist_msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    dist_msg.field_of_view = 0.366519;
    dist_msg.max_range = 9.0;
    dist_msg.min_range = 0.02;

    dist_msg.header.frame_id = "front_dist_sensor";
    dist_msg.range = to_distance(connection.get_dist_sensor_value(DistSensor::front), speed_of_sound_);
    front_dist_pub_->publish(dist_msg);

    dist_msg.header.frame_id = "front_left_dist_sensor";
    dist_msg.range = to_distance(connection.get_dist_sensor_value(DistSensor::front_left), speed_of_sound_);
    front_left_dist_pub_->publish(dist_msg);

    dist_msg.header.frame_id = "front_right_dist_sensor";
    dist_msg.range = to_distance(connection.get_dist_sensor_value(DistSensor::front_right), speed_of_sound_);
    front_right_dist_pub_->publish(dist_msg);

    dist_msg.header.frame_id = "front_left_bottom_dist_sensor";
    dist_msg.range = to_distance(connection.get_dist_sensor_value(DistSensor::front_left_bottom), speed_of_sound_);
    front_left_bottom_dist_pub_->publish(dist_msg);

    dist_msg.header.frame_id = "front_right_bottom_dist_sensor";
    dist_msg.range = to_distance(connection.get_dist_sensor_value(DistSensor::front_right_bottom), speed_of_sound_);
    front_right_bottom_dist_pub_->publish(dist_msg);

    dist_msg.header.frame_id = "left_dist_sensor";
    dist_msg.range = to_distance(connection.get_dist_sensor_value(DistSensor::left), speed_of_sound_);
    left_dist_pub_->publish(dist_msg);

    dist_msg.header.frame_id = "right_dist_sensor";
    dist_msg.range = to_distance(connection.get_dist_sensor_value(DistSensor::right), speed_of_sound_);
    right_dist_pub_->publish(dist_msg);

    dist_msg.header.frame_id = "rear_dist_sensor";
    dist_msg.range = to_distance(connection.get_dist_sensor_value(DistSensor::rear), speed_of_sound_);
    rear_dist_pub_->publish(dist_msg);

    dist_msg.header.frame_id = "rear_left_dist_sensor";
    dist_msg.range = to_distance(connection.get_dist_sensor_value(DistSensor::rear_left), speed_of_sound_);
    rear_left_dist_pub_->publish(dist_msg);

    dist_msg.header.frame_id = "rear_right_dist_sensor";
    dist_msg.range = to_distance(connection.get_dist_sensor_value(DistSensor::rear_right), speed_of_sound_);
    rear_right_dist_pub_->publish(dist_msg);

    dist_msg.header.frame_id = "rear_left_bottom_dist_sensor";
    dist_msg.range = to_distance(connection.get_dist_sensor_value(DistSensor::rear_left_bottom), speed_of_sound_);
    rear_left_bottom_dist_pub_->publish(dist_msg);

    dist_msg.header.frame_id = "rear_right_bottom_dist_sensor";
    dist_msg.range = to_distance(connection.get_dist_sensor_value(DistSensor::rear_right_bottom), speed_of_sound_);
    rear_right_bottom_dist_pub_->publish(dist_msg);

    /// GPS
    if(connection.get_longitude() != last_longitude_ || connection.get_latitude() != last_latitude_ ||
    connection.get_altitude() != last_altitude_){
        auto nav_sat_msg = sensor_msgs::msg::NavSatFix();
        nav_sat_msg.header.stamp = time_stamp;
        nav_sat_msg.header.frame_id = "gps_link";
        nav_sat_msg.latitude = connection.get_latitude();
        nav_sat_msg.longitude = connection.get_longitude();
        nav_sat_msg.altitude = connection.get_altitude();
        nav_sat_msg.status.status = connection.get_fix_quality() - 1;
        nav_sat_msg.status.service = 1;
        auto hdop = connection.get_hdop();
        nav_sat_msg.position_covariance[0] = 10.0*hdop;
        nav_sat_msg.position_covariance[3] = 10.0*hdop;
        nav_sat_msg.position_covariance[6] = 50.0;
        nav_sat_msg.position_covariance_type = 2;
        nav_sat_pub_->publish(nav_sat_msg);

        std_msgs::msg::Int32 num_satellites;
        num_satellites.data = connection.get_num_satellites();
        num_satellites_pub_->publish(num_satellites);
        std_msgs::msg::Float64 speed, angle;
        speed.data = connection.get_speed();
        gps_speed_pub_->publish(speed);
        angle.data = connection.get_angle();
        gps_angle_pub_->publish(angle);

        last_longitude_ = connection.get_longitude();
        last_latitude_ = connection.get_latitude();
        last_altitude_ = connection.get_altitude();
    }

    /// TSS
}

void BRHardwareInterface::write(std::chrono::steady_clock::duration elapsed_time) {

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

    /// Run PIDs
    auto right_cmd = std::min(std::max(last_right_motor_cmd_ + right_pid_.update(angular_velocity_right_setpoint - right_wheel_angular_velocity_, elapsed_time), -1.0), 1.0);
    auto left_cmd = std::min(std::max(last_left_motor_cmd_ + left_pid_.update(angular_velocity_left_setpoint - left_wheel_angular_velocity_, elapsed_time), -1.0), 1.0);
    last_right_motor_cmd_ = right_cmd;
    last_left_motor_cmd_ = left_cmd;

    connection.set_controller(right_cmd, left_cmd,0);
}

void BRHardwareInterface::configure_parameters() {
    encoder_ticks_per_rot_ = this->declare_parameter("encoder_ticks_per_rot", 196);
    speed_of_sound_ = this->declare_parameter("speed_of_sound", 374.0);
    wheel_diameter_ = this->declare_parameter("wheel_diameter", 0.2032);
    wheel_separation_ = this->declare_parameter("wheel_separation", 0.4118);
    left_wheel_joint_name_ = this->declare_parameter("left_wheel_joint_name", "left_wheel_joint");
    right_wheel_joint_name_ = this->declare_parameter("right_wheel_joint_name", "right_wheel_joint");

    double p = this->declare_parameter("pid_p", 0.1);
    left_pid_.set_p(p);
    right_pid_.set_p(p);
    double i = this->declare_parameter("pid_i", 0.0);
    left_pid_.set_i(i);
    right_pid_.set_i(i);
    double d = this->declare_parameter("pid_d", 0.0);
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

# This is a message that contains internal information about the hardware interface

std_msgs/Header header

# Left PID
float64 l_kp
float64 l_ki
float64 l_kd
float64 l_p
float64 l_i
float64 l_d
float64 l_error

# Left Wheel
float64 l_setpoint
float64 l_encoder_pose
float64 l_velocity
float64 l_cmd

# Right PID
float64 r_kp
float64 r_ki
float64 r_kd
float64 r_p
float64 r_i
float64 r_d
float64 r_error


# Right Wheel
float64 r_setpoint
float64 r_encoder_pose
float64 r_velocity
float64 r_cmd

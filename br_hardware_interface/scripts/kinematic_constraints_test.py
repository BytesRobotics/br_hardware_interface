#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def time_to_float(time):
    return time.sec + time.nanosec / 1.0e9


class KinematicsTester(Node):

    def __init__(self):
        super().__init__('KinematicsTester')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/t260/odom')
        self.declare_parameter('max_velocity', 0.15)
        self.declare_parameter('axis', 0)  # 0 = x, 1 = yaw

        (self.cmd_vel_topic, self.odom_topic, self.max_vel, self.axis) = self.get_parameters(
            ['cmd_vel_topic', 'odom_topic', 'max_velocity', 'axis'])

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic.get_parameter_value().string_value, 10)
        self.odometry_sub = self.create_subscription(Odometry, self.odom_topic.get_parameter_value().string_value,
                                                     self.odom_cb, 10)

        zero_msg = Twist()
        self.cmd_vel_pub.publish(zero_msg)

        self.acceleration_x_values = []
        self.current_vel_x = 0
        self.acceleration_yaw_values = []
        self.current_vel_yaw = 0

        self.stage = 0
        self.max_x_acc = 0
        self.start_time = 0

    def odom_cb(self, msg):
        # self.get_logger().info('Velocity X: %f, Velocity Y: %f, Velocity Yaw: %f' %
        #                        (msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z))
        cmd_msg = Twist()
        if self.stage == 0:
            if self.start_time == 0:
                self.start_time = time_to_float(msg.header.stamp)
                print(self.start_time)
            if self.axis.get_parameter_value().integer_value == 0:
                if msg.twist.twist.linear.x >= self.max_vel.get_parameter_value().double_value and (
                        time_to_float(msg.header.stamp) - self.start_time) != 0:
                    self.max_acc = msg.twist.twist.linear.x / (time_to_float(msg.header.stamp) - self.start_time)
                    self.stage += 1
                else:
                    cmd_msg.linear.x = self.max_vel.get_parameter_value().double_value
                    self.get_logger().info('Velocity X: %f, Velocity Y: %f, Velocity Yaw: %f' %
                                           (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                                            msg.twist.twist.angular.z))
            elif self.axis.get_parameter_value().integer_value == 1:
                if msg.twist.twist.angular.z >= self.max_vel.get_parameter_value().double_value and (
                        time_to_float(msg.header.stamp) - self.start_time) != 0:
                    self.max_acc = msg.twist.twist.angular.z / (time_to_float(msg.header.stamp) - self.start_time)
                    self.stage += 1
                else:
                    cmd_msg.angular.z = self.max_vel.get_parameter_value().double_value
                    self.get_logger().info('Velocity X: %f, Velocity Y: %f, Velocity Yaw: %f' %
                                           (msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                                            msg.twist.twist.angular.z))
        elif self.stage == 1:
            print('#################### Max Acceleration : ' + str(self.max_acc))
            exit(0)

        self.cmd_vel_pub.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    kinematics_tester = KinematicsTester()
    rclpy.spin(kinematics_tester)
    kinematics_tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

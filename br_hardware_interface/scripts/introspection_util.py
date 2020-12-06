#!/usr/bin/env python3

from dearpygui.core import *
from dearpygui.simple import *
import threading
from queue import Queue, Empty
import math
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from br_hardware_interface_msgs.msg import HardwareInterfaceDebug
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import tf2_ros
from rclpy.duration import Duration

def stamp_to_float(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

class IntrospectionGui:

    def __init__(self, debug_queue, cmd_vel_queue, odom_queue):
        self.debug_queue = debug_queue
        self.cmd_vel_queue = cmd_vel_queue
        self.odom_queue = odom_queue
        self.t260_rotation = R.from_quat([0.000, -0.455, 0.000, 0.891])
        self.t260_translation = [0.150, 0.000, 0.213]

        self.t0 = stamp_to_float(self.debug_queue.get().header.stamp)
        self.play = True

        self.last_cmd_vel = self.cmd_vel_queue.get()

        # construct the window
        with window("DebugIntrospector", width=950, height=600, x_pos=0, y_pos=0):

            add_button("Pause", callback=self.toggle_play)
            add_same_line(name="Toggle Time Span Spacing", spacing=10)
            add_input_text("Time Span", default_value="5.0", width=50)

            add_plot("Plot Right", x_axis_name='time', yaxis2=True, height=250)
            add_plot("Plot Left", x_axis_name='time', yaxis2=True, height=250)

            self.plot_r_velocity  = []
            self.plot_r_setpoint = []
            self.plot_r_error = []
            self.plot_r_p = []
            self.plot_r_i = []
            self.plot_r_d = []

            self.plot_l_velocity  = []
            self.plot_l_setpoint = []
            self.plot_l_error = []
            self.plot_l_p = []
            self.plot_l_i = []
            self.plot_l_d = []

            self.plot_t = []

            set_render_callback(self.plot_callback)

        self.length = 300
        with window("CmdIntrospector", width=self.length+30, height=self.length+30):
            add_drawing("VelPlot", width=self.length, height=self.length)
        self.draw_vel_plot()

        with window("VelIntrospector"):
            add_plot("Angular Vel", x_axis_name='time', yaxis2=True, height=250)
            add_plot("Linear Vel", x_axis_name='time', yaxis2=True, height=250)

            self.angular_velocities = []
            self.linear_velocities = []
            self.vel_plot_t = []
            self.vel_cmd = []


    def draw_vel_plot(self):
        l = self.length
        draw_circle("VelPlot", [l/2, l/2], l/2, [180, 180, 180, 100], tag="outerCirc")
        draw_circle("VelPlot", [l/2, l/2], l/3, [180, 180, 180, 100], tag="midCirc")
        draw_circle("VelPlot", [l/2, l/2], l/6, [180, 180, 180, 100], tag="innerCirc")

        draw_line("VelPlot", [0, l/2], [l, l/2], [180, 180, 180, 100], 1, tag="xLine")
        draw_line("VelPlot", [l/2, 0], [l/2, l], [180, 180, 180, 100], 1, tag="yLine")


    def toggle_play(self, sender, data):
        if self.play:
            self.play = False
            delete_item("Pause")
            add_button("Play", callback=self.toggle_play, before="Toggle Time Span Spacing")
        else:
            self.play = True
            delete_item("Play")
            add_button("Pause", callback=self.toggle_play, before="Toggle Time Span Spacing")

    def start(self):
        start_dearpygui()

    def plot_callback(self, sender, data):
        try:
            data = self.debug_queue.get(block=False)
            t = stamp_to_float(data.header.stamp) - self.t0

            if self.play:
                self.plot_t.append(t)

                self.plot_r_velocity.append(data.r_velocity)
                self.plot_r_setpoint.append(data.r_setpoint)
                self.plot_r_error.append(data.r_error)
                self.plot_r_p.append(data.r_p)
                self.plot_r_i.append(data.r_i)
                self.plot_r_d.append(data.r_d)

                self.plot_l_velocity.append(data.l_velocity)
                self.plot_l_setpoint.append(data.l_setpoint)
                self.plot_l_error.append(data.l_error)
                self.plot_l_p.append(data.l_p)
                self.plot_l_i.append(data.l_i)
                self.plot_l_d.append(data.l_d)

                time_span = 5.0
                try:
                    time_span = float(get_value("Time Span"))
                except:
                    log_warning("Invalid time span, using default of 5 seconds")

                while time_span < self.plot_t[-1] - self.plot_t[0]:
                    self.plot_t.pop(0)

                    self.plot_r_velocity.pop(0)
                    self.plot_r_setpoint.pop(0)
                    self.plot_r_error.pop(0)
                    self.plot_r_p.pop(0)
                    self.plot_r_i.pop(0)
                    self.plot_r_d.pop(0)

                    self.plot_l_velocity.pop(0)
                    self.plot_l_setpoint.pop(0)
                    self.plot_l_error.pop(0)
                    self.plot_l_p.pop(0)
                    self.plot_l_i.pop(0)
                    self.plot_l_d.pop(0)

                # plotting new data
                clear_plot("Plot Right")
                clear_plot("Plot Left")

                add_line_series("Plot Right", "Right Vel", self.plot_t, self.plot_r_velocity, weight=2)
                add_line_series("Plot Right", "Right Setpoint ", self.plot_t, self.plot_r_setpoint, weight=2)
                add_line_series("Plot Right", "Right Error", self.plot_t, self.plot_r_error, weight=2)
                add_line_series("Plot Right", "Right P", self.plot_t, self.plot_r_p, weight=2, axis=1)
                add_line_series("Plot Right", "Right I", self.plot_t, self.plot_r_i, weight=2, axis=1)
                add_line_series("Plot Right", "Right D", self.plot_t, self.plot_r_d, weight=2, axis=1)

                add_line_series("Plot Left", "Left Vel", self.plot_t, self.plot_l_velocity, weight=2)
                add_line_series("Plot Left", "Left Setpoint ", self.plot_t, self.plot_l_setpoint, weight=2)
                add_line_series("Plot Left", "Left Error", self.plot_t, self.plot_l_error, weight=2)
                add_line_series("Plot Left", "Left P", self.plot_t, self.plot_l_p, weight=2, axis=1)
                add_line_series("Plot Left", "Left I", self.plot_t, self.plot_l_i, weight=2, axis=1)
                add_line_series("Plot Left", "Left D", self.plot_t, self.plot_l_d, weight=2, axis=1)
        except Empty:
            pass

        try:
            try:
                cmd_data = self.cmd_vel_queue.get(block=False)
                self.last_cmd_vel = cmd_data
            except Empty:
                pass
            cmd_data = self.last_cmd_vel
            odom_data = self.odom_queue.get(block=False)
            # clear the queue
            while not self.odom_queue.empty():
                self.odom_queue.get(block=False)

            # Draw the current velocity arrow
            x = cmd_data.linear.x
            y = cmd_data.angular.z
            offset = self.length/2
            xscale = self.length
            yscale = math.pi
            r = (xscale*x + 1)
            theta = -yscale * y
            clear_drawing("VelPlot")
            self.draw_vel_plot()
            draw_arrow("VelPlot", [offset + r*math.cos(theta), offset + r*math.sin(theta)], [self.length/2, self.length/2], [0, 200, 255], 1, 10, tag="CmdVelArrow")

            # Draw the current odom arrow
            odom_twist = odom_data.twist.twist
            vel = [odom_twist.linear.x, odom_twist.linear.y, odom_twist.linear.z]
            ang_vel = [odom_twist.angular.x, odom_twist.angular.y, odom_twist.angular.z]

            vel = self.t260_rotation.apply(vel)
            ang_vel = self.t260_rotation.apply(ang_vel)
            x = vel[0]
            y = ang_vel[2]
            # print("linear", vel, "angular", ang_vel)
            r = (xscale*x + 1)
            theta = -yscale * y
            draw_arrow("VelPlot", [offset + r*math.cos(theta), offset + r*math.sin(theta)], [self.length/2, self.length/2], [200, 0, 255], 1, 10, tag="VelArrow")

            # Plot graph
            t = stamp_to_float(odom_data.header.stamp) - self.t0
            self.angular_velocities.append(ang_vel)
            self.linear_velocities.append(vel)
            self.vel_plot_t.append(t)
            self.vel_cmd.append([cmd_data.linear.x, cmd_data.angular.z])
            while 5 < self.vel_plot_t[-1] - self.vel_plot_t[0]:
                self.angular_velocities.pop(0)
                self.linear_velocities.pop(0)
                self.vel_cmd.pop(0)
                self.vel_plot_t.pop(0)
            add_line_series("Linear Vel", "vx", self.vel_plot_t, [y[0] for y in self.linear_velocities], weight=2, axis=1)
            add_line_series("Linear Vel", "vy", self.vel_plot_t, [y[1] for y in self.linear_velocities], weight=2, axis=1)
            add_line_series("Linear Vel", "vz", self.vel_plot_t, [y[2] for y in self.linear_velocities], weight=2, axis=1)

            add_line_series("Linear Vel", "cmd_vx", self.vel_plot_t, [y[0] for y in self.vel_cmd], weight=2, axis=1)

            add_line_series("Angular Vel", "vr", self.vel_plot_t, [y[0] for y in self.angular_velocities], weight=2, axis=1)
            add_line_series("Angular Vel", "vp", self.vel_plot_t, [y[1] for y in self.angular_velocities], weight=2, axis=1)
            add_line_series("Angular Vel", "vy", self.vel_plot_t, [y[2] for y in self.angular_velocities], weight=2, axis=1)

            add_line_series("Angular Vel", "cmd_vy", self.vel_plot_t, [y[1] for y in self.vel_cmd], weight=2, axis=1)

        except Empty:
            pass

class IntrospectionUtil(Node):

    def __init__(self, debug_queue, cmd_vel_queue, odom_queue):
        super().__init__('introspection_util')
        self.debug_queue = debug_queue
        self.cmd_vel_queue = cmd_vel_queue
        self.odom_queue = odom_queue
        self.debug_subscription = self.create_subscription(
            HardwareInterfaceDebug,
            '/hw_interface/debug',
            self.debug_callback,
            qos_profile=qos_profile_sensor_data)
        self.cmd_vel_subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback,
            qos_profile=qos_profile_sensor_data)
        self.odom_subscription = self.create_subscription(
            Odometry, '/t265/odom/sample', self.odom_callback,
            qos_profile=qos_profile_sensor_data)
        print("Node initialized")


        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # when = rclpy.time.Time()
        # self.transform = self.tf_buffer.lookup_transform("base_link", "base_link", when, timeout=Duration(seconds=5.0))

    def debug_callback(self, msg):
        self.debug_queue.put(msg)

    def cmd_vel_callback(self, msg):
        self.cmd_vel_queue.put(msg)

    def odom_callback(self, msg):
        self.odom_queue.put(msg)

def run_node(debug_queue, cmd_vel_queue, odom_queue):
    print("Running node")
    rclpy.init()
    introspection_util = IntrospectionUtil(debug_queue, cmd_vel_queue, odom_queue)
    rclpy.spin(introspection_util)
    introspection_util.destroy_node()
    rclpy.shutdown()

def run_gui(debug_queue, cmd_vel_queue, odom_queue):
    print("Running GUI")
    gui = IntrospectionGui(debug_queue, cmd_vel_queue, odom_queue)
    gui.start()

if __name__ == '__main__':
    debug_q = Queue()
    cmd_vel_q = Queue()
    odom_q = Queue()

    t1 = threading.Thread(target = run_node, args = (debug_q, cmd_vel_q, odom_q, ))
    t1.start()
    run_gui(debug_q, cmd_vel_q, odom_q)

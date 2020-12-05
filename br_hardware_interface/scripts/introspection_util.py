#!/usr/bin/env python3

from dearpygui.core import *
from dearpygui.simple import *
import threading
from queue import Queue, Empty

import rclpy
from rclpy.node import Node
from br_hardware_interface_msgs.msg import HardwareInterfaceDebug
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

def stamp_to_float(stamp):
    return stamp.sec + stamp.nanosec * 1e-9

class IntrospectionGui:

    def __init__(self, queue, plot_len=100):
        self.queue = queue
        self.plot_len = plot_len
        data = self.queue.get()
        self.t0 = stamp_to_float(data.header.stamp)

        show_logger()
        # construct the window

        with window("Introspector"):
            add_plot("Plot", x_axis_name='time', yaxis2=True, height=-1)

            zeros = [0 for i in range(self.plot_len + 1)]
            self.plot_r_velocity  = zeros.copy()
            self.plot_r_setpoint = zeros.copy()
            self.plot_r_error = zeros.copy()
            self.plot_r_p = zeros.copy()
            self.plot_r_i = zeros.copy()
            self.plot_r_d = zeros.copy()

            self.plot_t = [i for i in range(self.plot_len + 1)]

            set_render_callback(self.plot_callback)

    def start(self):
        start_dearpygui()

    def plot_callback(self, sender, data):
        data = self.queue.get()
        t = stamp_to_float(data.header.stamp) - self.t0

        # Right wheel debug information
        self.plot_r_velocity.pop(0)
        self.plot_r_setpoint.pop(0)
        self.plot_r_error.pop(0)
        self.plot_r_p.pop(0)
        self.plot_r_i.pop(0)
        self.plot_r_d.pop(0)

        self.plot_r_velocity.append(data.r_velocity)
        self.plot_r_setpoint.append(data.r_setpoint)
        self.plot_r_error.append(data.r_error)
        self.plot_r_p.append(data.r_p)
        self.plot_r_i.append(data.r_i)
        self.plot_r_d.append(data.r_d)

         # plotting new data
        clear_plot("Plot")
        # print(self.plot_r_velocity)
        add_line_series("Plot", "Right Vel", self.plot_t, self.plot_r_velocity, weight=2)
        add_line_series("Plot", "Right Setpoint ", self.plot_t, self.plot_r_setpoint, weight=2)
        add_line_series("Plot", "Right Error", self.plot_t, self.plot_r_error, weight=2)
        # add_line_series("Plot", "Right P", self.plot_t, self.plot_r_p, weight=2, axis=1)
        # add_line_series("Plot", "Right I", self.plot_t, self.plot_r_i, weight=2, axis=1)
        # add_line_series("Plot", "Right D", self.plot_t, self.plot_r_d, weight=2, axis=1)



class IntrospectionUtil(Node):

    def __init__(self, queue):
        super().__init__('introspection_util')
        self.queue = queue
        self.subscription = self.create_subscription(
            HardwareInterfaceDebug,
            '/hw_interface/debug',
            self.listener_callback,
            qos_profile=qos_profile_sensor_data)
        print("Node initialized")

    def listener_callback(self, msg):
        self.queue.put(msg)

def run_node(queue):
    print("Running node")
    rclpy.init()
    introspection_util = IntrospectionUtil(queue)
    rclpy.spin(introspection_util)
    introspection_util.destroy_node()
    rclpy.shutdown()

def run_gui(queue):
    print("Running GUI")
    gui = IntrospectionGui(queue)
    gui.start()

if __name__ == '__main__':
    q = Queue()
    t1 = threading.Thread(target = run_node, args = (q, ))
    t1.start()
    run_gui(q)

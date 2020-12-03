# Copyright 2020 Bytes Robotics

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')

    # Get the launch directory
    br_hardware_interface_dir = get_package_share_directory('br_hardware_interface')
    launch_files_dir = get_package_share_directory('launch_files')

    hardware_interface_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(br_hardware_interface_dir, 'launch'),
                                       '/hw_interface.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    control_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(launch_files_dir, 'launch'),
                                       '/control.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    ld = LaunchDescription()
    ld.add_action(hardware_interface_cmd)
    ld.add_action(control_cmd)

    return ld

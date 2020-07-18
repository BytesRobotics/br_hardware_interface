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

    hardware_interface_cmd = Node(
        package='br_hardware_interface',
        executable='br_hw_node',
        name='hardware_interface',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()
    ld.add_action(hardware_interface_cmd)

    return ld
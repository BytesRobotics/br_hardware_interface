import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # export models path
    if os.getenv('HARDWARE_INTERFACE_PORT') is not None:
        port = os.getenv('HARDWARE_INTERFACE_PORT')
    else:
        port = "/dev/ttyACM0"

    hw_interface_dir = get_package_share_directory('br_hardware_interface')
    params = os.path.join(hw_interface_dir, "params", "hw_interface_params.yaml")

    hardware_interface_cmd = Node(
        package='br_hardware_interface',
        executable='br_hw_node',
        name='hardware_interface',
        output="screen",
        arguments=[port],
        parameters=[params]
    )

    ld = LaunchDescription()
    ld.add_action(hardware_interface_cmd)

    return ld
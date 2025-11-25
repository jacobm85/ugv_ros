from ament_index_python.packages import get_package_share_path
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Allow overriding which device to use (eventX or js0)
    declare_device_arg = DeclareLaunchArgument(
        "device",
        default_value="/dev/input/event2",
        description="Joystick input device (e.g., /dev/input/event2 or /dev/input/js0)"
    )

    # Create a node to read joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[
            {"device": LaunchConfiguration("device")},
            {"deadzone": 0.1},
            {"autorepeat_rate": 0.0},
        ]
    )

    # Create a node to control the robot using joystick input
    joy_ctrl_node = Node(
        package='ugv_tools',
        executable='joy_ctrl',
    )

    # Return the launch description
    return LaunchDescription([
        declare_device_arg,
        joy_node,
        joy_ctrl_node
    ])

"""
Launch the omni_vel_controller as a standalone ros2_control controller.

Usage:
  ros2 launch omni_vel_controller omni_vel_controller.launch.py
  ros2 launch omni_vel_controller omni_vel_controller.launch.py robot:=omniquad
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('omni_vel_controller')

    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='omnicar',
        description='Robot variant: omnicar or omniquad',
    )

    robot = LaunchConfiguration('robot')

    # Load the YAML config matching the selected robot
    controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'omni_vel_controller',
            '--param-file', [pkg_dir, '/config/', robot, '.yaml'],
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_arg,
        controller_node,
    ])

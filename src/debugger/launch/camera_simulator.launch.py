#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='debugger',
            executable='camera_simulator_node',
            name='camera_simulator',
            output='screen'
        )
    ])

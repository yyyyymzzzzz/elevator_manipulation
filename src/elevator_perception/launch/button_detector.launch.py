#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取参数文件路径
    config_file = os.path.join(
        '/home/ymz/Workspace/elevator_manipulation/config',
        'button_detector_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='elevator_perception',
            executable='button_detector_node',
            name='button_detector',
            parameters=[config_file],
            output='screen'
        )
    ])

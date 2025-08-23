#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 机械臂控制器
        Node(
            package='elevator_arm_control',
            executable='arm_controller',
            name='arm_controller',
            output='screen'
        ),
        
        # 目标位姿发布器
        Node(
            package='debugger',
            executable='target_pose_publisher',
            name='target_pose_publisher',
            output='screen'
        ),
        
        # 可视化辅助节点
        Node(
            package='debugger',
            executable='visualization_helper',
            name='visualization_helper',
            output='screen'
        ),
    ])

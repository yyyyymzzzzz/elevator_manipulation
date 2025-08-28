#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 获取参数文件路径
    button_detector_config = os.path.join(
        '/home/ymz/Workspace/elevator_manipulation/config',
        'button_detector_params.yaml'
    )
    
    button_3d_config = os.path.join(
        '/home/ymz/Workspace/elevator_manipulation/config',
        'button_3d_visualizer_params.yaml'
    )
    
    return LaunchDescription([
        # 按钮检测节点
        Node(
            package='elevator_perception',
            executable='button_detector_node',
            name='button_detector',
            parameters=[button_detector_config],
            output='screen'
        ),
        
        # 按钮3D可视化节点
        Node(
            package='elevator_perception',
            executable='button_3d_visualizer',
            name='button_3d_visualizer',
            parameters=[button_3d_config],
            output='screen'
        )
    ])

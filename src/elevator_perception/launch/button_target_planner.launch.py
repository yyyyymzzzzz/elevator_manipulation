#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录
    package_dir = get_package_share_directory('elevator_perception')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(package_dir, 'config', 'button_target_planner_params.yaml'),
        description='配置文件路径'
    )
    
    # 按钮目标规划节点
    button_target_planner_node = Node(
        package='elevator_perception',
        executable='button_target_planner',
        name='button_target_planner',
        parameters=[LaunchConfiguration('config_file')],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        button_target_planner_node,
    ])

#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包目录
    pkg_share = get_package_share_directory('agv_controller')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('elevator_manipulation'),
            'config',
            'agv_target_controller_params.yaml'
        ]),
        description='AGV控制器配置文件路径'
    )
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.10.10',
        description='机器人IP地址'
    )
    
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='31001',
        description='机器人端口'
    )
    
    # AGV目标控制器节点
    agv_controller_node = Node(
        package='agv_controller',
        executable='agv_target_controller',
        name='agv_target_controller',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'robot_ip': LaunchConfiguration('robot_ip'),
                'robot_port': LaunchConfiguration('robot_port'),
            }
        ],
        remappings=[
            # 重映射话题名称（如果需要）
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        robot_ip_arg,
        robot_port_arg,
        agv_controller_node,
    ])

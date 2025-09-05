#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # 获取配置文件路径
    config_dir = os.path.join(get_package_share_directory('elevator_perception'), '..', '..', '..', '..', 'config')
    params_file = os.path.join(config_dir, 'realtime_button_target_planner_params.yaml')
    
    # 声明启动参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=params_file,
        description='实时按钮目标规划器配置文件路径'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )
    
    # 实时按钮目标规划器节点
    realtime_planner_node = Node(
        package='elevator_perception',
        executable='realtime_button_target_planner',
        name='realtime_button_target_planner',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # 输入话题
            ('/button_3d_markers', '/button_3d_markers'),
            # 输出话题
            ('/realtime_target_markers', '/realtime_target_markers'),
            ('/button_target_pose', '/button_target_pose'),
            ('/button_targets_json', '/button_targets_json'),
        ],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        realtime_planner_node,
    ])

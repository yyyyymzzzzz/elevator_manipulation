#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    
    # 获取配置文件路径
    config_dir = os.path.join(get_package_share_directory('elevator_perception'), '..', '..', '..', '..', 'config')
    
    # 声明启动参数
    use_realtime_planner_arg = DeclareLaunchArgument(
        'use_realtime_planner',
        default_value='true',
        description='使用实时目标规划器而不是原始规划器'
    )
    
    realtime_params_file_arg = DeclareLaunchArgument(
        'realtime_config_file',
        default_value=os.path.join(config_dir, 'realtime_button_target_planner_params.yaml'),
        description='实时目标规划器配置文件路径'
    )
    
    original_params_file_arg = DeclareLaunchArgument(
        'original_config_file', 
        default_value=os.path.join(config_dir, 'button_target_planner_params.yaml'),
        description='原始目标规划器配置文件路径'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )
    
    button_selection_method_arg = DeclareLaunchArgument(
        'button_selection_method',
        default_value='sequential',
        description='按钮选择方法: sequential, all, closest'
    )
    
    publish_frequency_arg = DeclareLaunchArgument(
        'publish_frequency',
        default_value='30.0',
        description='目标位置发布频率 (Hz)'
    )
    
    # 实时按钮目标规划器节点（默认使用）
    realtime_planner_group = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_realtime_planner')),
        actions=[
            Node(
                package='elevator_perception',
                executable='realtime_button_target_planner',
                name='realtime_button_target_planner',
                parameters=[
                    LaunchConfiguration('realtime_config_file'),
                    {
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'button_selection_method': LaunchConfiguration('button_selection_method'),
                        'publish_frequency': LaunchConfiguration('publish_frequency'),
                    }
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
        ]
    )
    
    # 原始按钮目标规划器节点（备用）
    original_planner_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('use_realtime_planner')),
        actions=[
            Node(
                package='elevator_perception',
                executable='button_target_planner',
                name='button_target_planner',
                parameters=[
                    LaunchConfiguration('original_config_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}
                ],
                remappings=[
                    ('/button_3d_markers', '/button_3d_markers'),
                    ('/target_markers', '/target_markers'),
                    ('/button_target_pose', '/button_target_pose'),
                ],
                output='screen',
                emulate_tty=True
            )
        ]
    )
    
    return LaunchDescription([
        use_realtime_planner_arg,
        realtime_params_file_arg,
        original_params_file_arg,
        use_sim_time_arg,
        button_selection_method_arg,
        publish_frequency_arg,
        realtime_planner_group,
        original_planner_group,
    ])

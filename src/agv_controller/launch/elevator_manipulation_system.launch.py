#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取包目录
    agv_pkg_share = get_package_share_directory('agv_controller')
    elevator_perception_pkg = get_package_share_directory('elevator_perception')
    elevator_arm_pkg = get_package_share_directory('elevator_arm_control')
    
    # 声明启动参数
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.10.10',
        description='AGV机器人IP地址'
    )
    
    arm_ip_arg = DeclareLaunchArgument(
        'arm_ip', 
        default_value='192.168.10.100',
        description='机械臂IP地址'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    # 1. 启动机械臂控制器
    # arm_controller_node = Node(
    #     package='elevator_arm_control',
    #     executable='arm_controller',
    #     name='arm_controller',
    #     output='screen',
    #     parameters=[
    #         {
    #             'use_sim_time': LaunchConfiguration('use_sim_time'),
    #             'arm_ip': LaunchConfiguration('arm_ip'),
    #         }
    #     ]
    # )
    
    # 2. 启动按钮检测节点
    # button_detector_node = Node(
    #     package='elevator_perception',
    #     executable='button_detector_node',
    #     name='button_detector_node',
    #     output='screen',
    #     parameters=[
    #         PathJoinSubstitution([
    #             get_package_share_directory('elevator_manipulation'),
    #             'config',
    #             'button_detector_params.yaml'
    #         ]),
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ]
    # )
    
    # 3. 启动3D可视化节点
    # button_3d_visualizer_node = Node(
    #     package='elevator_perception',
    #     executable='button_3d_visualizer',
    #     name='button_3d_visualizer',
    #     output='screen',
    #     parameters=[
    #         PathJoinSubstitution([
    #             get_package_share_directory('elevator_manipulation'),
    #             'config',
    #             'button_3d_visualizer_params.yaml'
    #         ]),
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ]
    # )
    
    # 4. 启动实时按钮目标规划器（支持按钮顺序切换）
    # realtime_button_planner_node = Node(
    #     package='elevator_perception',
    #     executable='realtime_button_target_planner',
    #     name='realtime_button_target_planner',
    #     output='screen',
    #     parameters=[
    #         PathJoinSubstitution([
    #             get_package_share_directory('elevator_manipulation'),
    #             'config',
    #             'realtime_button_target_planner_params.yaml'
    #         ]),
    #         {
    #             'use_sim_time': LaunchConfiguration('use_sim_time'),
    #             'button_selection_method': 'sequential',  # 使用顺序模式
    #             'sequential_duration': 20.0,  # 每个按钮20秒超时
    #         }
    #     ]
    # )
    
    # 5. 启动按钮跟随器（机械臂控制）
    # button_follower_node = Node(
    #     package='elevator_arm_control',
    #     executable='button_follower',
    #     name='button_follower',
    #     output='screen',
    #     parameters=[
    #         {
    #             'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         }
    #     ]
    # )
    
    # 6. 启动AGV目标控制器
    agv_controller_node = Node(
        package='agv_controller',
        executable='agv_target_controller',
        name='agv_target_controller',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory('agv_controller'),
                'config',
                'agv_target_controller_params.yaml'
            ]),
            {
                'robot_ip': LaunchConfiguration('robot_ip'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        arm_ip_arg,
        use_sim_time_arg,
        
        # 按顺序启动所有节点
        # arm_controller_node,
        # button_detector_node,
        # button_3d_visualizer_node,
        # realtime_button_planner_node,
        # button_follower_node,
        agv_controller_node,
    ])

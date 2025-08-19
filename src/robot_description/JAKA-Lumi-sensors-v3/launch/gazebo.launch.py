#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('jaka_lumi_sensors_v3')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # Define paths
    default_model_path = os.path.join(pkg_share, 'urdf', 'jaka_lumi_sensors_v3.urdf')
    world_path = os.path.join(gazebo_ros_dir, 'worlds', 'empty.world')
    
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
    )
    
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Path to world file'
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['cat ', LaunchConfiguration('model')]),
                value_type=str
            )
        }]
    )
    
    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'jaka_lumi_sensors_v3',
            '-file', LaunchConfiguration('model'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )
    
    # Static transform publisher for base_footprint
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )
    
    return LaunchDescription([
        model_arg,
        world_arg,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        static_tf_node
    ])

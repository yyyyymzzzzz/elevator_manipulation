#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package directory
    pkg_share = get_package_share_directory('jaka_lumi_sensors_v3')
    
    # Define paths
    default_model_path = os.path.join(pkg_share, 'urdf', 'jaka_lumi_sensors_v3.urdf')
    
    # Declare launch arguments
    model_arg = DeclareLaunchArgument(
        name='model', 
        default_value=default_model_path,
        description='Absolute path to robot urdf file'
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
    
    # Joint State Publisher (without GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )
    
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])

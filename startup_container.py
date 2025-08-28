import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Get the value of the use_sim_time argument
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Get package path
    robot_description_path = os.path.join(
        get_package_share_directory("robot_description"),
        "urdf",
        "jaka_lumi_sensors_v3.urdf"
    )
    
    # Use MoveItConfigsBuilder to load configurations
    moveit_config = (
        MoveItConfigsBuilder("jaka_lumi_sensors_v3", package_name="jaka_lumi_moveit_config")
        .robot_description(file_path=robot_description_path)
        .robot_description_semantic(file_path="config/JAKA-Lumi-sensors-v3.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    button_detector_config = os.path.join(
        '/home/ymz/Workspace/elevator_manipulation/config',
        'button_detector_params.yaml'
    )
    
    button_3d_config = os.path.join(
        '/home/ymz/Workspace/elevator_manipulation/config',
        'button_3d_visualizer_params.yaml'
    )

    # 1. Start move_group node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True},
            {"allow_trajectory_execution": True},
            {"fake_execution": False},
            {'use_sim_time': use_sim_time},
            {"trajectory_execution.allowed_start_tolerance": 0.1},
            {"trajectory_execution.allowed_goal_tolerance": 0.1},
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
        ],
    )

    # 2. Start RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("jaka_lumi_moveit_config"),
        "config",
        "moveit.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': use_sim_time},
        ],
    )

    # 3. Start robot_state_publisher 
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {'use_sim_time': use_sim_time},
            ],
    )

    # 4. Start trajectory controller
    trajectory_controller_node = Node(
        package='elevator_arm_control',
        executable='trajectory_controller',
        name='jaka_arm_controller',
        output='screen'
    )

    # 5. Start arm_controller
    arm_controller_node = Node(
        package='elevator_arm_control',
        executable='arm_controller',
        name='arm_controller',
        output='screen'
    )
    
    # 6. Start camera node
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('orbbec_camera'),
                'launch',
                'gemini2L.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 7. Add static transform publisher to link camera to robot
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_camera_tf_publisher',
        arguments=['0.1', '0', '0', '0', '1.5708', '1.5708', 'Camera3_Link', 'camera_link'],
    )

    # 按钮检测节点
    button_detector_node = Node(
        package='elevator_perception',
        executable='button_detector_node',
        name='button_detector',
        parameters=[button_detector_config],
        output='screen'
    )

    # 按钮3D可视化节点
    button_3d_visualizer_node = Node(
        package='elevator_perception',
        executable='button_3d_visualizer',
        name='button_3d_visualizer',
        parameters=[button_3d_config],
        output='screen'
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            rviz_node,
            robot_state_publisher,
            run_move_group_node,
            trajectory_controller_node,
            arm_controller_node,
            camera_launch,
            static_tf_node,
            button_detector_node,
            button_3d_visualizer_node
        ]
    )
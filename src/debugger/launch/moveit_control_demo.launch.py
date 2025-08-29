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
    # 自定义 OMPL 配置作为参数字典
    ompl_planning_yaml_params = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planning_request_adapters/AddTimeOptimalParameterization default_planning_request_adapters/FixWorkspaceBounds default_planning_request_adapters/FixStartStateBounds default_planning_request_adapters/FixStartStateCollision default_planning_request_adapters/FixStartStatePathConstraints",
            "default_planning_request_adapters": {
                "AddTimeOptimalParameterization": {
                    "type": "planning_request_adapter/AddTimeOptimalParameterization",
                    "path_tolerance": 0.1,
                    "resample_dt": 0.5,
                }
            },
        },
        "jaka_arm": {
            "planner_configs": ["RRTConnect"],
            "projection_evaluator": "joints(l_a1,l_a2)",
            "longest_valid_segment_fraction": 1.0,
        },
    }

    # 1. Start move_group node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            ompl_planning_yaml_params, 
            {"publish_robot_description_semantic": True},
            {"allow_trajectory_execution": True},
            {"fake_execution": False},
            {'use_sim_time': use_sim_time},
            {"trajectory_execution.allowed_start_tolerance": 0.1},
            {"trajectory_execution.allowed_goal_tolerance": 0.1},
            {"trajectory_execution.allowed_execution_duration_scaling": 25.0},  # 增加执行时间容忍度
            {"trajectory_execution.execution_duration_monitoring": True},
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

    # 4. Start trajectory controller with optimization parameters
    trajectory_optimization_params = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
        "config",
        "trajectory_optimization_params.yaml"
    )
    
    trajectory_controller_node = Node(
        package='elevator_arm_control',
        executable='trajectory_controller',
        name='jaka_arm_controller',
        output='screen',
        parameters=[trajectory_optimization_params] if os.path.exists(trajectory_optimization_params) else []
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


    return LaunchDescription(
        [
            use_sim_time_arg,
            rviz_node,
            robot_state_publisher,
            run_move_group_node,
            trajectory_controller_node,
            arm_controller_node,
            static_tf_node,
        ]
    )
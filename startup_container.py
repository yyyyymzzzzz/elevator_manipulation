import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Declare the use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    agv_pkg_share = get_package_share_directory('agv_controller')
    elevator_perception_pkg = get_package_share_directory('elevator_perception')
    elevator_arm_pkg = get_package_share_directory('elevator_arm_control')

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
        
        # 'ompl' 是顶层键
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            
            # 1. 声明要使用的适配器 (这部分你之前是正确的)
            "request_adapters": "default_planning_request_adapters/AddTimeOptimalParameterization default_planning_request_adapters/FixWorkspaceBounds default_planning_request_adapters/FixStartStateBounds default_planning_request_adapters/FixStartStateCollision default_planning_request_adapters/FixStartStatePathConstraints",
            
            # 'jaka_arm' 组的配置
            "jaka_arm": {
                "planner_configs": ["RRTConnect", "RRTstar"],
                "RRTConnect": {
                    "type": "geometric::RRTConnect",
                    "range": 0.0,
                },
                "RRTstar": {
                    "type": "geometric::RRTstar",
                    "range": 0.0,
                    "goal_bias": 0.05,
                },
                "projection_evaluator": "joints(l_a1,l_a2,l_a3,l_a4,l_a5,l_a6)", 
                "longest_valid_segment_fraction": 0.01, 
            },
        },
        
        # 2. 修正: 将适配器的 *定义* 移动到顶层，作为 "ompl" 的兄弟键
        "default_planning_request_adapters": {
            "AddTimeOptimalParameterization": {
                "type": "planning_request_adapter/AddTimeOptimalParameterization",
                "path_tolerance": 0.1,    # 路径重新采样的容忍度
                "resample_dt": 0.1,         # 建议：将 0.5 改为 0.1 以获得更平滑的轨迹
            }
            # ... 这里可以添加 FixWorkspaceBounds 等的定义, 但通常它们不需要额外参数
        },
    }


    button_detector_config = os.path.join(
        '/home/nvidia/Workspace/elevator_manipulation/config',
        'button_detector_params.yaml'
    )
    
    button_3d_config = os.path.join(
        '/home/nvidia/Workspace/elevator_manipulation/config',
        'button_3d_visualizer_params.yaml'
    )

    realtime_button_target_planner_config = os.path.join(
        '/home/nvidia/Workspace/elevator_manipulation/config',
        'realtime_button_target_planner_params.yaml'
    )

    agv_target_controller_config = os.path.join(
        '/home/nvidia/Workspace/elevator_manipulation/config',
        'agv_target_controller_params.yaml'
    )

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
            {"trajectory_execution.allowed_goal_tolerance": 0.05},
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
        arguments=['0.08', '0.005', '0', '0', '1.5708', '1.5708', 'Camera3_Link', 'camera_link'],
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

    # 实时按钮目标规划器节点
    realtime_planner_node = Node(
        package='elevator_perception',
        executable='realtime_button_target_planner',
        name='realtime_button_target_planner',
        parameters=[realtime_button_target_planner_config],
        output='screen',
        emulate_tty=True
    )

    button_follower_node = Node(
        package='elevator_arm_control',
        executable='button_follower',
        name='button_follower',
        output='screen'
    )

    agv_controller_node = Node(
        package='agv_controller',
        executable='agv_target_controller',
        name='agv_target_controller',
        output='screen',
        parameters=[
            agv_target_controller_config,
            {
                'robot_ip': LaunchConfiguration('robot_ip'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            robot_ip_arg,
            arm_ip_arg,
            rviz_node,
            robot_state_publisher,
            run_move_group_node,
            trajectory_controller_node,
            arm_controller_node,
            camera_launch,
            static_tf_node,
            button_detector_node,
            button_3d_visualizer_node,
            realtime_planner_node,
            button_follower_node,
            agv_controller_node
        ]
    )
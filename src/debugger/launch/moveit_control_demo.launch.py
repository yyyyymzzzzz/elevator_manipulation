import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 获取包路径
    robot_description_path = os.path.join(
        get_package_share_directory("robot_description"),
        "urdf",
        "jaka_lumi_sensors_v3.urdf"
    )
    
    # 使用MoveItConfigsBuilder加载配置
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

    # 1. 启动 move_group 节点
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True},
            {"allow_trajectory_execution": True},
            {"fake_execution": False},
            {"use_sim_time": False},
            # 增加轨迹执行的容差，解决起始状态偏差问题
            {"trajectory_execution.allowed_start_tolerance": 0.1},  # 增加到0.1弧度
            {"trajectory_execution.allowed_goal_tolerance": 0.1},   # 增加目标容差
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},  # 允许更长的执行时间
        ],
    )

    # 2. 启动 RViz
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
        ],
    )

    # 3. 启动 robot_state_publisher 
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 4. 启动轨迹控制器（桥接MoveIt和arm_controller）
    trajectory_controller_node = Node(
        package='elevator_arm_control',
        executable='trajectory_controller',
        name='jaka_arm_controller',
        output='screen'
    )

    # 5. 启动我们修改后的 arm_controller
    arm_controller_node = Node(
        package='elevator_arm_control',
        executable='arm_controller',
        name='arm_controller',
        output='screen'
    )

    return LaunchDescription(
        [
            rviz_node,
            robot_state_publisher,
            run_move_group_node,
            trajectory_controller_node,
            arm_controller_node,
        ]
    )
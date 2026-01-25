from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 配置文件路径
    button_detector_config = os.path.join(
        '/home/nvidia/Workspace/elevator_manipulation/config',
        'button_detector_params.yaml'
    )
    button_3d_config = os.path.join(
        '/home/nvidia/Workspace/elevator_manipulation/config',
        'button_3d_visualizer_params.yaml'
    )
    realtime_button_target_planner_config = os.path.join(
        '/home/cat/Workspace/elevator_manipulation/config',
        'realtime_button_target_planner_params.yaml'
    )
    
    # 相机节点
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('orbbec_camera'),
                'launch',
                'gemini2L.launch.py'
                # 'gemini345.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 相机到机器人的静态TF
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

    realtime_planner_node = Node(
        package='elevator_perception',
        executable='realtime_button_target_planner',
        name='realtime_button_target_planner',
        parameters=[realtime_button_target_planner_config],
        output='screen',
        emulate_tty=True
    )

    # 按钮3D可视化节点
    button_3d_visualizer_node = Node(
        package='elevator_perception',
        executable='button_3d_visualizer',
        name='button_3d_visualizer',
        parameters=[button_3d_config],
        output='screen'
    )
    
    # 面板检测节点
    look_panel_node = Node(
        package='elevator_perception',
        executable='panel_perception',
        name='panel_perception',
        output='screen'
    )

    # RViz可视化
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
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        use_sim_time_arg,
        camera_launch,
        static_tf_node,
        button_detector_node,
        realtime_planner_node,
        button_3d_visualizer_node,
        look_panel_node,
        rviz_node,
    ])
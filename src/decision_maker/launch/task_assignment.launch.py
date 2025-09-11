from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'task_config_file',
            default_value='tasks.yaml',
            description='任务配置文件名'
        ),
        
        DeclareLaunchArgument(
            'enable_continuous_loop',
            default_value='true',
            description='是否启用连续循环执行任务'
        ),
        
        # 启动任务分配节点
        Node(
            package='decision_maker',
            executable='task_assignment',
            name='task_assignment',
            parameters=[{
                'task_config_file': LaunchConfiguration('task_config_file'),
                'enable_continuous_loop': LaunchConfiguration('enable_continuous_loop'),
            }],
            output='screen'
        ),
    ])

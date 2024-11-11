import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node  # 从 launch_ros.actions 导入 Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=''),
        DeclareLaunchArgument('use_composition', default_value='false'),
        DeclareLaunchArgument('container_name', default_value=''),

        # 添加启动节点
        Node(
            package='your_package_name',  # 替换为您的包名
            executable='your_executable',  # 替换为您的可执行文件名
            name='your_node_name',  # 为节点指定一个名称
            output='screen',
            parameters=[{'use_sim_time': False}],  # 传递相关参数
            remappings=[('/cmd_vel', '/bot1/cmd_vel')]  # 如果需要进行话题重映射
        ),
    ])

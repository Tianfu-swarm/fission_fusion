import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition 

def generate_launch_description():
    # 设置环境变量
    set_qt_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    # 声明可传入的参数（默认值也可以保留）
    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('results_file_path', default_value='/home/tianfu/fission_fusion_ws/src/fission_fusion/data/output.csv'),
        DeclareLaunchArgument('isMinCommunication', default_value='true'),
        DeclareLaunchArgument('isConCommunication', default_value='true'),
        DeclareLaunchArgument('isModelworks', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('expected_subgroup_size', default_value='14'),
        DeclareLaunchArgument('subgroup_size_sigma', default_value='1'),
        DeclareLaunchArgument('groupsize_tolerance', default_value='0'),
    ]

    # 创建机器人节点
    nodes = []
    for i in range(42): 
        nodes.append(
            Node(
                package='fission_fusion',
                executable='fission_fusion',
                name='fission_fusion_controller',
                namespace=f'bot{i}',
                output='screen',
                parameters=[{
                    "use_sim_time": LaunchConfiguration('use_sim_time'),
                    "results_file_path": LaunchConfiguration('results_file_path'),
                    "isMinCommunication": LaunchConfiguration('isMinCommunication'),
                    "isConCommunication": LaunchConfiguration('isConCommunication'),
                    "isModelworks": LaunchConfiguration('isModelworks'),
                    "expected_subgroup_size":LaunchConfiguration('expected_subgroup_size'),
                    "subgroup_size_sigma":LaunchConfiguration('subgroup_size_sigma'),
                    "groupsize_tolerance":LaunchConfiguration('groupsize_tolerance'),
                    "controller_type": "sffm",  #  "SDRM" / "skybat" / "P" /"sffm"
                }]
            )
        )

    # 添加 RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        output='screen',
        arguments=['-d', "/home/tianfu/fission_fusion_ws/src/fission_fusion/launch/rviz/defaul.rviz"],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{"use_sim_time": LaunchConfiguration('use_sim_time')}]
    )
    nodes.append(rviz_node)

    return LaunchDescription(declare_args + [set_qt_platform] + nodes)

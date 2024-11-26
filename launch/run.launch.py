import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Set environment variable for QT_QPA_PLATFORM
    # This line ensures that rviz uses the correct platform
    set_qt_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    nodes = []

    for i in range(20): 
        nodes.append(
            Node(
                package='fission_fusion',  
                executable='controller',  
                name=f'controller',  
                namespace=f'bot{i}',  
                output='screen',  
            )
        )
 
    rviz_node = Node(
        package='rviz2',  
        executable='rviz2',  
        name='rviz',  
        output='screen',  
        arguments=['-d', "/home/tianfu/fission_fusion_ws/src/fission_fusion/launch/rviz/defaul.rviz"],  
    )
    

    nodes.append(rviz_node)

  
    return LaunchDescription([set_qt_platform] + nodes)


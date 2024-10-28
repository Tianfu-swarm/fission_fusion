import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='information_dissemination',  
            executable='information_dissemination',  
            name='information_dissemination',  
            output='screen',  
        ),
    ])

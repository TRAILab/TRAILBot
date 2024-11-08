import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        Node(
            package='trailbot_gazebo',
            executable='basic_navigator'
        )
    )


import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf_transform',  # Replace with your actual package name
            executable='tf_transform_node.py',
            name='tf_transform_node'
        ),
    ])


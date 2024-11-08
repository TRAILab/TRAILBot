import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # FSM Node
    fsm_node = Node(
        package='fsm',
        executable='trailbot_fsm',
        # name='fsm',
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(fsm_node)
    return ld
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch.conditions
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from datetime import date
from datetime import datetime


def generate_launch_description():

    # # Husky Driving needs launch
    driving_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','driving.launch.py')
    driving_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([driving_launch_path]))

    # # Logitech Camera Launch File
    camera_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','logitech_camera.launch.py')
    camera_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([camera_launch_path])) 

    # Ouster Lidar Launch File
    ouster_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','ouster.launch.py')
    ouster_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([ouster_launch_path])) 

    ld = LaunchDescription()

    # driving launch is in ouster.launch.py 
    #(this throws no error launching joint_state_publisher, TODO to be tested thoroughly)
    ld.add_action(driving_launch)
    ld.add_action(camera_launch)
    ld.add_action(ouster_launch)
   
    #place camera launch after ouster, or else launch crashes
 
    return ld
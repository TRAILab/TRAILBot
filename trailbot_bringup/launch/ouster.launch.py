# File Launches Everything Needed for SLAM (LiDAR, IMU (3D), Husky Driving, and Cartographer)

import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Husky Driving needs launch
    driving_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','driving.launch.py')
    driving_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([driving_launch_path]))

    
    # ouster lidar launch
    ouster_launch_path = os.path.join(get_package_share_directory('ouster_ros'),'launch','driver.launch.py')
    ouster_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([ouster_launch_path]))


    ld = LaunchDescription()
    
    ld.add_action(driving_launch)
    ld.add_action(ouster_launch)

    return ld
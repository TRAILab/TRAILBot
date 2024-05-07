# File Launches Everything Needed for SLAM (LiDAR, IMU (3D), Husky Driving, and Cartographer)

import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

#alias in ~/.bashrc: bringup

def generate_launch_description():   
    # Husky Driving needs launch
    driving_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','driving.launch.py')
    driving_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([driving_launch_path]))

    #velodyne launch
    # velo_launch_path1 = os.path.join(get_package_share_directory('velodyne_driver'),'launch','velodyne_driver_node-VLP16-launch.py')
    # velo_launch1 = IncludeLaunchDescription(PythonLaunchDescriptionSource([velo_launch_path1]))
    # velo_launch_path2 = os.path.join(get_package_share_directory('velodyne_pointcloud'),'launch','velodyne_convert_node-VLP16-launch.py')
    # velo_launch2 = IncludeLaunchDescription(PythonLaunchDescriptionSource([velo_launch_path2]))
      
    # ouster lidar launch
    ouster_launch_path = os.path.join(get_package_share_directory('ouster_ros'),'launch','driver.launch.py')
    ouster_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([ouster_launch_path]))

    # Logitech Camera Launch File
    camera_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','logitech_camera.launch.py')
    camera_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([camera_launch_path])) 

    # Create an event handler to launch the ouster after X seconds
    camera_after_timer = TimerAction(
        period=4.0,  # Time to wait before executing the action (in seconds)
        actions=[camera_launch]  # Action to execute after the delay
    )

    # Define the event handler to trigger the action when the IO message is received, check camera stdout
    # io_event_handler = RegisterEventHandler(
    #     event_handler=OnProcessIO(
    #         target_action=camera_launch,
    #         on_stdout=lambda event: LogInfo(msg='Timer triggering every 33 ms'), # Check for last message in camera launch
    #         actions=[launch_ouster()]  # Action to execute when the message is received
    #     )
    # )

    ld = LaunchDescription()
    
    ld.add_action(driving_launch)
    ld.add_action(ouster_launch)
    ld.add_action(camera_after_timer) #ld.add_action(ouster_launch)
    
    # run logitech_camera.launch.py separately (ld.add_action() throws error)
    # TODO create logitech_camera lifecylce node in this launch file

    return ld
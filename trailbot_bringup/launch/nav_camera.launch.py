#File Launches Everything for Nav2 stack and includes SLAM and Driving 

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():

    # Launching SLAM (also includes driving and all other needed nodes)
    slam_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','slam_3D.launch.py')
    slam_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([slam_launch_path]))

    #the nav configs 
    package_name = 'nav'

    # Nav node
    nav_launch_path = os.path.join(get_package_share_directory(package_name),'launch','navigation_launch.py')
    nav_params_path = os.path.join(get_package_share_directory(package_name),'config','nav2_params_points.yaml')
    nav_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([nav_launch_path]),
                                        launch_arguments={'namespace': '',
                                                        # 'use_sim_time': 'true',
                                                         'autostart': 'true',
                                                        'params_file': nav_params_path,
                                                        # 'use_lifecycle_mgr': 'false',
                                                        #'map_subscribe_transient_local': 'true'
                                                        }
                                                        .items())



    # #Ximea Camera Node (commented because no current implementation)
    # ld = LaunchDescription()
    # config = os.path.join(
    #     get_package_share_directory("ximea_driver"),
    #     "config",
    #     "params.yaml",
    # )
    # ximea_node = Node(
    #     package="ximea_driver",
    #     executable="ximea_driver_node",
    #     name="ximea_driver_node",
    #     parameters=[config]
    # )

    config = os.path.join(
        get_package_share_directory("trailbot_bringup"),
        "logitech_camera",
        "config",
        "params.yaml",
    )
    logitech_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_node",
        #ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /path/to/colcon_ws/src/usb_cam/config/params.yaml
        # arguments=["params-file", config]
        #https://github.com/ros-drivers/usb_cam/tree/ros2?tab=readme-ov-file
        #to see supported formats
        #ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="test"

        #to compress:
        #add package https://gitlab.com/boldhearts/ros2_v4l2_camera#usage-1
        #ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=image_raw/compressed --remap out:=image_raw/uncompressed
        #
        parameters=[config]
    )
    ld = LaunchDescription()
    ld.add_action(logitech_node)
    ld.add_action(slam_launch)
    ld.add_action(nav_node)

    return ld
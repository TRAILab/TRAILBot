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

    #node to create tf frame following robots position at map elevation
    robot_transform_publisher = Node(
        package='tf_transform',
        executable='tf_transform_node',
        name='tf_transform_node',
        output='screen',
        parameters=[{'use_sim_time': 'true'}],
    )             

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


    ld = LaunchDescription()
    ld.add_action(slam_launch)
    ld.add_action(robot_transform_publisher)
    ld.add_action(nav_node)

    return ld
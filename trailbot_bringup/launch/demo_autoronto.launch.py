import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from datetime import date
from datetime import datetime


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

    fsm_node = Node(
        package='fsm',
        # executable='trailbot_fsm',
        executable='trailbot_fsm',
        name='fsm',
        output='screen'
    )

    navigator_node = Node(
        package='fsm',
        # executable='navigator_node',
        executable='fsm_test',
        name='test_cmd_vel_node',
        output='screen'
    )

    #Ximea Camera Node (commented because no current implementation)
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("ximea_driver"),
        "config",
        "params.yaml",
    )
    ximea_node = Node(
        package="ximea_driver",
        executable="ximea_driver_node",
        name="ximea_driver_node",
        parameters=[config]
    )

    # Launch voice_assistant/voice_assistant.launch.py which is voice interaction.
    launch_voice_assistant = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("voice_assistant"), 'launch', 'voice_assistant.launch.py'])))

    # Launch human detection
    human_detection_node = Node(
        package='human_detection',
        executable='human_detection_node'
    )

    # Launch trail detection
    trail_detection_node = Node(
        package='trail_detection_node',
        executable='trail_detection'
    )

    # Launch file logging
    current_date = date.today()
    current_time = datetime.now().time()
    formatted_time = current_time.strftime("%H:%M")
    file_logging = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--include-hidden-topics', '-o', f'/home/autoronto_bags/{current_date}-{formatted_time}','/camera/compressed','/velodyne_points','/scan','imu/data'],
        output='screen'
    )

    ld = LaunchDescription()
    # ld.add_action(fsm_node)
    # ld.add_action(navigator_node)
    ld.add_action(ximea_node)
    ld.add_action(slam_launch)
    # ld.add_action(nav_node)
    # ld.add_action(human_detection_node)
    # ld.add_action(trail_detection_node)
    # ld.add_action(launch_voice_assistant)

    logging = False
    if logging:
        ld.add_action(file_logging)
  

    return ld
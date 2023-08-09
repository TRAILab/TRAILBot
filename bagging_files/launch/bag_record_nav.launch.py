#File Launches Everything for Nav2 stack and includes SLAM and Driving 

import os
from ament_index_python.packages import get_package_share_directory
from datetime import date, datetime


from launch import LaunchDescription,  LaunchIntrospector, LaunchService
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros import actions


from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    # Launching SLAM (also includes driving and all other needed nodes)
    slam_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','slam_3D.launch.py')
    slam_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([slam_launch_path]))

        # Husky Driving needs launch
    driving_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','driving.launch.py')
    driving_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([driving_launch_path]))


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
    # #GPS LAUNCH
    # gps_launch= os.path.join(get_package_share_directory('nmea_navsat_driver'),'launch','nmea_serial_driver.launch.py')
    # gps_launch_path = IncludeLaunchDescription(PythonLaunchDescriptionSource([gps_launch]))
    # #GPS NODE
    #"""Generate a launch description for a single serial driver."""
    #config_file = os.path.join(get_package_share_directory("nmea_navsat_driver"), "config", "nmea_serial_driver.yaml")
    gps_driver_node = actions.Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        output='screen',
        parameters=[{#config_file, 
            'port': '/dev/serial/by-id/usb-Emlid_ReachM2_82438ED0F4EC80DD-if02',
            'baud': 115200,
            'frame_id': "gps",
            'time_ref_source': "gps",
            'useRMC': False
        }]
    )

        #velodyne launch
    velo_launch_path1 = os.path.join(get_package_share_directory('velodyne_driver'),'launch','velodyne_driver_node-VLP16-launch.py')
    velo_launch1 = IncludeLaunchDescription(PythonLaunchDescriptionSource([velo_launch_path1]))
    velo_launch_path2 = os.path.join(get_package_share_directory('velodyne_pointcloud'),'launch','velodyne_convert_node-VLP16-launch.py')
    velo_launch2 = IncludeLaunchDescription(PythonLaunchDescriptionSource([velo_launch_path2]))


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
        
    #IMU
    IMU_node = Node(
        package="umx_driver",
        executable="um7_driver",
        name="um7_node",
        parameters=[{'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0', 'zero_gyros': True, 'set_mag_ref': True, 'reset_ekf': True, 'update_rate': 60, 'baud': 115200}]
    )
    
    # Launch file logging
    current_date = date.today()
    current_time = datetime.now().time()
    formatted_time = current_time.strftime("%H:%M")
    file_logging = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--include-hidden-topics', '-o', f'/home/trailbot/bags/{current_date}-{formatted_time}','-a'],
        output='screen'
    )



    ld = LaunchDescription()
    ld.add_action(slam_launch)
    ld.add_action(nav_node)
    ld.add_action(driving_launch)
    ld.add_action(ximea_node)
    #ld.add_action(gps_launch_path)
    ld.add_action(gps_driver_node)
    ld.add_action(velo_launch1)
    ld.add_action(velo_launch2)
    ld.add_action(IMU_node)

       #logging part
    logging = True
    if logging:
        ld.add_action(file_logging)


    return ld
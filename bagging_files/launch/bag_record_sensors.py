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

        # Husky Driving needs launch
    driving_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','driving.launch.py')
    driving_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([driving_launch_path]))


    # #GPS node
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
    ld.add_action(driving_launch)
    ld.add_action(gps_driver_node)
    ld.add_action(velo_launch1)
    ld.add_action(velo_launch2)
    ld.add_action(IMU_node)

       #logging part
    logging = True
    if logging:
        ld.add_action(file_logging)


    return ld
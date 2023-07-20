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


    #the cartographer configs 
    package_name = 'slam'

    #velodyne launch
    velo_launch_path1 = os.path.join(get_package_share_directory('velodyne_driver'),'launch','velodyne_driver_node-VLP16-launch.py')
    velo_launch1 = IncludeLaunchDescription(PythonLaunchDescriptionSource([velo_launch_path1]))
    velo_launch_path2 = os.path.join(get_package_share_directory('velodyne_pointcloud'),'launch','velodyne_convert_node-VLP16-launch.py')
    velo_launch2 = IncludeLaunchDescription(PythonLaunchDescriptionSource([velo_launch_path2]))
    
    #rviz launch
    rviz_config_path = os.path.join(get_package_share_directory(package_name),'config','rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')

    #IMU
    IMU_node = Node(
        package="umx_driver",
        executable="um7_driver",
        name="um7_node",
        parameters=[{'port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0', 'zero_gyros': True, 'set_mag_ref': True, 'reset_ekf': True, 'update_rate': 60, 'baud': 115200}]
    )


    # Cartographer node
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    trailbot_cartographer_prefix = get_package_share_directory(package_name)
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  trailbot_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='trailbot_lds_3d.lua') 
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename],
        remappings=[('/husky_velocity_controller/odom', '/odom'),
                    ('/points2', '/velodyne_points'),
                    ('/imu', 'imu/data')],
    )

    occupancy_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', 'occupancy_grid.launch.py')]),
        launch_arguments={'resolution':resolution,
                          'publish_period_sec': publish_period_sec}.items(),)

    


    ld = LaunchDescription()
    ld.add_action(driving_launch)
    ld.add_action(cartographer_node)
    ld.add_action(velo_launch1)
    ld.add_action(velo_launch2)
    ld.add_action(occupancy_grid)
    ld.add_action(rviz_node)
    ld.add_action(IMU_node) 

    return ld
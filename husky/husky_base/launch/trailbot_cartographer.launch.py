import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    trailbot_cartographer_prefix = get_package_share_directory('trailbot_gazebo')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  trailbot_cartographer_prefix, 'config'))
    # configuration_basename = LaunchConfiguration('configuration_basename',
    #                                              default='trailbot_lds_2d.lua')
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                  default='trailbot_lds_3d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(get_package_share_directory('trailbot_gazebo'),
                                   'config','rviz_config.rviz')

    return LaunchDescription([

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),
                       
        #pointcloud to laserscan conversion node
        Node(
            package='pointcloud_to_laserscan_converter', executable='pointcloud_to_laserscan_node',
            #remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/cloud']),
                        #('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
            parameters=[{
                # 'target_frame': 'base_link',
                'target_frame': 'laser_frame',
                'transform_tolerance': 0.01,
                #base link height max/min
                # 'min_height': 0.1, #4 inches
                # 'max_height': 1.0,
                #laser_frame height max/min
                'min_height': -0.9282, #4 inches above ground 
                'max_height': 0.1, 
                'angle_min': -3.14,  #was -M_PI/2 now -PI
                'angle_max': 3.14,  #was  M_PI/2 now PI
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
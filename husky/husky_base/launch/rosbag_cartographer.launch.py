import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, IncludeLaunchDescription, ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap
from launch.substitutions import LaunchConfiguration, FindPackageShare, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    bag_filename_arg = DeclareLaunchArgument('bag_filename')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    trailbot_cartographer_prefix = get_package_share_directory('husky_base')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  trailbot_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                  default='trailbot_lds_3d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    rviz_config_dir = os.path.join(get_package_share_directory('husky_base'),
                                   'config','rviz_config_no_nav.rviz')


    cartographer_node = Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            remappings=[('/points2', '/velodyne_points'),
                        ('/imu', 'imu/data')
                        ]
    )
     
    occupancy_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
    )
    
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
    )
                   
    ros2_bag_play_cmd = ExecuteProcess(
        cmd = ['ros2', 'bag', 'play', LaunchConfiguration('bag_filename'), '--clock'],
        name = 'rosbag_play',)
  


    return LaunchDescription([
        bag_filename_arg,
        cartographer_node,
        occupancy_launch,
	rviz_node,
        ros2_bag_play_cmd
    ])

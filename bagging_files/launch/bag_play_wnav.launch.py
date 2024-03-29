import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
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

    #rviz launch
    rviz_config_path = os.path.join(get_package_share_directory('slam'),'config','rviz_config_bagging.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')
    

    # Cartographer node
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    trailbot_cartographer_prefix = get_package_share_directory('slam')
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
        remappings=[('/points2', '/velodyne_points'),
                    ('/imu', 'imu/data'),
                    ('/odom', 'odometry/filtered')],
    )

    occupancy_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('slam'), 'launch', 'occupancy_grid.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time, 'resolution':resolution,
                          'publish_period_sec': publish_period_sec}.items(),)

    # Nav node
    nav_launch_path = os.path.join(get_package_share_directory('nav'),'launch','navigation_launch.py')
    nav_params_path = os.path.join(get_package_share_directory('nav'),'config','nav2_params_points.yaml')
    nav_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([nav_launch_path]),
                                        launch_arguments={'namespace': '',
                                                        'use_sim_time': 'true',
                                                         'autostart': 'true',
                                                        'params_file': nav_params_path,
                                                        'use_lifecycle_mgr': 'false',
                                                        'map_subscribe_transient_local': 'true'
                                                        }
                                                        .items())  

    #node to create tf frame following robots position at map elevation
    robot_transform_publisher = Node(
        package='tf_transform',
        executable='tf_transform_node',
        name='tf_transform_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )             

    # #velodyne filter
    # velodyne_filter = Node(
    #     package='pointcloud_filter',
    #     executable='pointcloud_filter_node',
    #     name='pointcloud_filter_node',
    #     parameters= [
    #     {'frame_id': 'velodyne'},
    #     {'topic_pointcloud_in': '/velodyne_points'},
    #     {'topic_pointcloud_out': '/pointcloud_filtered'}],
    # )
                           
    ros2_bag_play_cmd = ExecuteProcess(
        cmd = ['ros2', 'bag', 'play', LaunchConfiguration('bag_filename'), '--clock', '--rate', '1.5',],
        name = 'rosbag_play',)
    
    # #velodyne filter 
    # filter_launch_path = os.path.join(get_package_share_directory('pointcloud_filter'),'launch','launch.py')
    # velo_filter_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([filter_launch_path]))


    return LaunchDescription([
        bag_filename_arg,
        cartographer_node,
        occupancy_grid,
        nav_node,
        robot_transform_publisher,
	    rviz_node,
        ros2_bag_play_cmd,
        #velodyne_filter,
    ])
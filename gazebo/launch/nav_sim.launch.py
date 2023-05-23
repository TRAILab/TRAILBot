import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir


# from launch.actions import RegisterEventHandler, DeclareLaunchArgument
# from launch.event_handlers import OnProcessExit
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name='trailbot_gazebo'

    # Gazebo sim launch
    gazebo_launch_path = os.path.join(get_package_share_directory(package_name),'launch','gazebo_sim.launch.py')
    gazebo_builder = IncludeLaunchDescription(PythonLaunchDescriptionSource([gazebo_launch_path]))

    # Rviz node
    rviz_config_path = os.path.join(get_package_share_directory(package_name),'config','rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')

    # Slam node
    slam_launch_path = os.path.join(get_package_share_directory('slam_toolbox'),'launch','online_async_launch.py')
    slam_config_path = os.path.join(get_package_share_directory(package_name),'config','mapper_params_online_async.yaml')
    slam_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([slam_launch_path]),
                                        launch_arguments={'params_file': slam_config_path}.items())

    # Nav node
    nav_launch_path = os.path.join(get_package_share_directory(package_name),'launch','navigation_launch.py')
    nav_params_path = os.path.join(get_package_share_directory(package_name),'config','nav2_params.yaml')
    tree_params_path = os.path.join(get_package_share_directory(package_name),'config','navigate_w_replanning_and_recovery.xml')
    nav_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([nav_launch_path]),
                                        launch_arguments={'namespace': '',
                                                        'use_sim_time': 'true',
                                                        'autostart': 'true',
                                                        'params_file': nav_params_path,
                                                        'default_bt_xml_filename': tree_params_path,
                                                        'use_lifecycle_mgr': 'false',
                                                        'map_subscribe_transient_local': 'true'}.items())

    # Cartographer node
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    trailbot_cartographer_prefix = get_package_share_directory(package_name)
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  trailbot_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename', default='trailbot_lds_2d.lua') 
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename]) 

    occupancy_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(package_name), 'launch', 'occupancy_grid.launch.py')]),
        launch_arguments={'use_sim_time': use_sim_time, 'resolution':resolution,
                          'publish_period_sec': publish_period_sec}.items(),)
    
    delayed_spawn = TimerAction(period=15.0,
                    actions=[rviz_node, slam_node])

    delayed_spawn2 = TimerAction(period=20.0,
                    actions=[nav_node])

    # Generate launch description
    ld = LaunchDescription()
    ld.add_action(gazebo_builder)
    ld.add_action(rviz_node)
    # ld.add_action(slam_node)

    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid)

    # ld.add_action(delayed_spawn)
    # ld.add_action(delayed_spawn2)
    ld.add_action(nav_node)

    return ld


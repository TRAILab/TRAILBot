import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

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

    # FSM Node
    fsm_node = Node(
        package='fsm',
        executable='trailbot_fsm',
        name='fsm',
        output='screen'
    )
    
    # Navigator Node
    navigator_node = Node(
        package='fsm',
        executable='navigator_node',
        name='test_cmd_vel_node',
        output='screen'
    )


    namespace = ''
    use_namespace = 'false'
    slam = 'True'
    map_yaml_file = os.path.join(get_package_share_directory(package_name),'maps','turtlebot3_world.xml')
    use_sim_time = 'true'
    params_file = os.path.join(get_package_share_directory(package_name),'config','nav2_params.yaml')
    autostart = 'true'
    use_composition = 'True'
    use_respawn = 'False'
    # bringup_path = os.path.join(get_package_share_directory(package_name),'launch','bringup_launch.py')
    bringup_path = os.path.join(get_package_share_directory(package_name),'launch','bringup_cartographer_launch.py')
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_path),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())


    # delayed_spawn = TimerAction(period=15.0,
    #                 actions=[rviz_node, slam_node])

    # delayed_spawn2 = TimerAction(period=20.0,
    #                 actions=[nav_node])

    # Generate launch description
    ld = LaunchDescription()
    ld.add_action(gazebo_builder)
    ld.add_action(rviz_node)
    ld.add_action(fsm_node)
    ld.add_action(navigator_node)
    # ld.add_action(slam_node)
    # ld.add_action(delayed_spawn)
    # ld.add_action(delayed_spawn2)
    ld.add_action(bringup_cmd)

    return ld


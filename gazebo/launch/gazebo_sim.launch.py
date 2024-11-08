import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

# from launch.actions import RegisterEventHandler, DeclareLaunchArgument
# from launch.event_handlers import OnProcessExit
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    package_name='trailbot_gazebo'

    # Robot state publisher node
    rsp_launch_path = os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py')
    rsp_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([rsp_launch_path]),
                                        launch_arguments={'use_sim_time': 'true'}.items())

    world_file = os.path.join(get_package_share_directory(package_name),'worlds','clearpath_playpen.world')
    # world_file = os.path.join(get_package_share_directory(package_name),'worlds','obstacles.world')

    # Gazebo server
    gzserver = ExecuteProcess(
                cmd=['gzserver',
                    '-s', 'libgazebo_ros_init.so',
                    '-s', 'libgazebo_ros_factory.so',
                    '--verbose',
                    world_file],
                output='screen')

    # Gazebo client
    gzclient = ExecuteProcess(
                cmd=['gzclient'],
                output='screen',)


    # Spawn robot. Trivial for single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'husky'],
                        output='screen')

    # Joint broadcaster node                     
    joint_broad = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad",'-c', '/controller_manager'],
        output='screen',
    )
    
    # Differential control node
    diff_drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", '-c', '/controller_manager'],
        output='screen',
    )

     # Make sure spawn_husky_velocity_controller starts after spawn_joint_state_broadcaster
    diffdrive_controller_spawn_callback = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broad,
            on_exit=[diff_drive],
        )
    )

    # Delay spawning of joint_broad and diff_drive until controller_manager is available
    # TODO: fix this properly    
    delayed_spawn = TimerAction(period=10.0,
                    actions=[joint_broad, diff_drive])

    # Joystic node
    joy_launch_path = os.path.join(get_package_share_directory(package_name),'launch','joystick.launch.py')
    joy_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([joy_launch_path]), 
                                        launch_arguments={'use_sim_time': 'true'}.items())

    # twist_mux node
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(package="twist_mux",
                        executable="twist_mux",
                        parameters=[twist_mux_params, {'use_sim_time': True}],
                        remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')])

    # Generate launch description
    ld = LaunchDescription()
    ld.add_action(rsp_node)
    ld.add_action(joy_node)
    ld.add_action(twist_mux)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_entity)
    ld.add_action(joint_broad)
    ld.add_action(diffdrive_controller_spawn_callback)
    # ld.add_action(diff_drive)
    
    # ld.add_action(delayed_spawn)

    return ld


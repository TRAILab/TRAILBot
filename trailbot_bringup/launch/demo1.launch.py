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
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"),
                 "urdf", "husky.urdf.xacro"]
            ),
            " ",
            "name:=husky",
            " ",
            "prefix:=''",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # the cartographer and nav configs

    package_name = 'husky_base'

    # Nav node
    nav_launch_path = os.path.join(get_package_share_directory(
        package_name), 'launch', 'navigation_launch.py')
    nav_params_path = os.path.join(get_package_share_directory(
        package_name), 'config', 'nav2_params_points.yaml')
    nav_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([nav_launch_path]),
                                        launch_arguments={'namespace': '',
                                                          'use_sim_time': 'true',
                                                          'autostart': 'true',
                                                          'params_file': nav_params_path,
                                                          'use_lifecycle_mgr': 'false',
                                                          'map_subscribe_transient_local': 'true'}.items())

    # velodyne launch
    velo_launch_path1 = os.path.join(get_package_share_directory(
        'velodyne_driver'), 'launch', 'velodyne_driver_node-VLP16-launch.py')
    velo_launch1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([velo_launch_path1]))
    velo_launch_path2 = os.path.join(get_package_share_directory(
        'velodyne_pointcloud'), 'launch', 'velodyne_convert_node-VLP16-launch.py')
    velo_launch2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([velo_launch_path2]))

    # rviz launch
    rviz_config_path = os.path.join(get_package_share_directory(
        package_name), 'config', 'rviz_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')

    # Ximea Camera Node (commented because no current implementation)
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

    # Cartographer node
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    trailbot_cartographer_prefix = get_package_share_directory(package_name)
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  trailbot_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration(
        'configuration_basename', default='trailbot_lds_2d.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration(
        'publish_period_sec', default='1.0')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', configuration_basename],
        remappings=[('/husky_velocity_controller/odom', '/odom'),
                    ('/points2', '/velodyne_points')],
    )

    occupancy_grid = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(
            package_name), 'launch', 'occupancy_grid.launch.py')]),
        launch_arguments={'resolution': resolution,
                          'publish_period_sec': publish_period_sec}.items(),)

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"),
         "config",
         "control.yaml"],
        # remappings=['/husky_velocity_controller/odom', '/odom']

    )

    fsm_node = Node(
        package='fsm',
        executable='trailbot_fsm',
        name='fsm',
        output='screen'
    )

    navigator_node = Node(
        package='fsm',
        executable='navigator_node',
        name='test_cmd_vel_node',
        output='screen'
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        remappings=[('/husky_velocity_controller/odom', '/odom')],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, config_husky_velocity_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        remappings=[('/husky_velocity_controller/odom', '/odom')],
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
        remappings=[('/husky_velocity_controller/odom', '/odom')],
    )

    spawn_husky_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["husky_velocity_controller"],
        output="screen",
        remappings=[('/husky_velocity_controller/odom', '/odom')],

    )

    # Launch husky_control/control.launch.py which is just robot_localization.
    launch_husky_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("husky_control"), 'launch', 'control.launch.py'])))

    # Launch husky_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_husky_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("husky_control"), 'launch', 'teleop_base.launch.py'])))

    # Launch husky_control/teleop_joy.launch.py which is tele-operation using a physical joystick.
    launch_husky_teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("husky_control"), 'launch', 'teleop_joy.launch.py'])))

    # Launch husky_bringup/accessories.launch.py which is the sensors commonly used on the Husky.
    launch_husky_accessories = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("husky_bringup"), 'launch', 'accessories.launch.py'])))

    # Launch voice_assistant/voice_assistant.launch.py which is voice interaction.
    launch_voice_assistant = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("voice_assistant"), 'launch', 'voice_assistant.launch.py'])))

    # Launch human detection
    human_detection_node = Node(
        package='human_detection',
        executable='human_detection_node'
    )

    # Launch file logging
    current_date = date.today()
    current_time = datetime.now().time()
    formatted_time = current_time.strftime("%H:%M")
    file_logging = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--include-hidden-topics', '-o', f'/home/trailbot/bags/{current_date}-{formatted_time}','/camera/compressed','/velodyne_points','/circle_marker','/circle_marker_array','/global_costmap/costmap','/global_costmap/costmap_updates','/parameter_events','/scan','trailbot_state','target_location','imu/data'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_controller_manager)
    ld.add_action(spawn_controller)
    ld.add_action(spawn_husky_velocity_controller)
    ld.add_action(launch_husky_control)
    ld.add_action(launch_husky_teleop_base)
    ld.add_action(launch_husky_teleop_joy)
    ld.add_action(launch_husky_accessories)
    ld.add_action(fsm_node)
    ld.add_action(navigator_node)
    ld.add_action(cartographer_node)
    ld.add_action(nav_node)
    ld.add_action(velo_launch1)
    ld.add_action(velo_launch2)
    ld.add_action(occupancy_grid)
    ld.add_action(rviz_node)
    ld.add_action(ximea_node)
    ld.add_action(launch_voice_assistant)
    ld.add_action(human_detection_node)

    logging = False
    if logging:
        ld.add_action(file_logging)
  

    return ld

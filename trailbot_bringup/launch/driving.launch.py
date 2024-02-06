#THIS FILE IS MEANT ONLY FOR REMOTE DRIVING OF HUSKY
import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir

def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
            " ",
            "name:=husky",
            " ",
            "prefix:=''",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}


    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"),
        "config",
        "control.yaml"]
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
    
    
    
    config = os.path.join(
        get_package_share_directory("trailbot_bringup"),
        "logitech_camera",
        "config",
        "params.yaml",
    )
    logitech_node = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam_node",
        #ros2 run usb_cam usb_cam_node_exe --ros-args --params-file /path/to/colcon_ws/src/usb_cam/config/params.yaml
        # arguments=["params-file", config]
        #https://github.com/ros-drivers/usb_cam/tree/ros2?tab=readme-ov-file
        #to see supported formats
        #ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:="test"

        #to compress:
        #add package https://gitlab.com/boldhearts/ros2_v4l2_camera#usage-1
        #ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=image_raw/compressed --remap out:=image_raw/uncompressed
        #
        parameters=[config]
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

    ld.add_action(logitech_node)


    return ld

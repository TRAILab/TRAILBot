import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

# from launch.substitutions import EnvironmentVariable
# from launch_ros.substitutions import FindPackageShar
# import xacro

def generate_launch_description():

    is_sim = LaunchConfiguration('is_sim')
    prefix = LaunchConfiguration('prefix')

    # Process the robot URDF files
    xacro_file = os.path.join(get_package_share_directory('trailbot_gazebo'),'urdf','husky.urdf.xacro')
    config_husky_velocity_controller = os.path.join(get_package_share_directory('trailbot_gazebo'),'config','control.yaml')

    robot_description_urdf = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file,
            " ",
            "name:=husky",
            " ",
            "prefix:=",
            prefix,
            " ",
            "is_sim:=",
            is_sim,
            " ",
            "gazebo_controllers:=",
            config_husky_velocity_controller,
        ]
    )


    # Create the robot_state_publisher node
    params = {'robot_description': robot_description_urdf, 'use_sim_time': is_sim}
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Return launch file
    return LaunchDescription([
        DeclareLaunchArgument(
            'is_sim',
            default_value='true',
            description='Use sim time if true'),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Entity name prefix'),
        rsp_node
    ])

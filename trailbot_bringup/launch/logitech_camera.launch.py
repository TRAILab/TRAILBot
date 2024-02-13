import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Define the path you want to pass as an argument
    config = os.path.join(
        get_package_share_directory("trailbot_bringup"),
        "logitech_camera",
        "config",
        "params.yaml",
    )
    # Launch usb_cam node with the provided argument
    params_file_path = "/home/trailbot/trail_ws/src/TRAILBot/trailbot_bringup/logitech_camera/config/params.yaml"
    
    # Retrieve the package directory path
    package_dir = os.path.dirname(os.path.dirname(__file__))

    # Construct the path to the YAML parameter file
    params_file_path = os.path.join(package_dir, 'logitech_camera', 'config', 'params.yaml')
    

    # Ensure the parameter file exists
    if not os.path.exists(params_file_path):
        print(f"Parameter file {params_file_path} does not exist.")
        return

    # Declare a launch argument for the parameter file path
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_path,
        description='Path to YAML parameter file'
    )

    # Launch usb_cam node with parameters loaded from the YAML file
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        remappings=[
                ('/image_raw', '/camera')
            ],
        output='screen',
        arguments=['--ros-args', '--params-file', LaunchConfiguration('params_file')]
    )

    # Create the launch description and add the nodes
    ld = LaunchDescription()
    ld.add_action(params_file_arg)
    ld.add_action(usb_cam_node)

    return ld

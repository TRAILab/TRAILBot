import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('voice_assistant'),
        'config',
        'params.yaml'
        )

    voice_assistant_node = Node(
        package="voice_assistant",
        executable="voice_assistant_node",
    )
    voice_arduino_bridge_node = Node(
        package="voice_assistant",
        executable="voice_assistant_node"
    )
    ld.add_action(voice_assistant_node)
    ld.add_action(voice_arduino_bridge_node)
    return ld
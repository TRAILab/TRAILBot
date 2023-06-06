import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LaunchService

import rclpy

from std_msgs.msg import String

class FSM:
    def __init__(self):
        self.node = rclpy.create_node('fsm_node')
        self.subscription = self.node.create_subscription(
            String,
            'robot_state',
            self.callback,
            10
        )

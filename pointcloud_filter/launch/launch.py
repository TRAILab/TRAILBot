#  Copyright (c) 2022 Jonas Mahler

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_filter',
            executable='pointcloud_filter_node',
            name='pointcloud_filter_node',
            parameters= [
            {'frame_id': 'velodyne'},
            {'topic_pointcloud_in': '/velodyne_points'},
            {'topic_pointcloud_out': '/pointcloud_filtered'}],
        )
    ])

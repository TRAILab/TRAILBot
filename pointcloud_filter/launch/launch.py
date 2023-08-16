#  Copyright (c) 2022 Jonas Mahler

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_filer',
            executable='pointcloud_filer_node',
            name='pointcloud_filer_node',
            parameters= [
            {'frame_id': 'lidar'},
            {'topic_pointcloud_in': 'bf_lidar/point_cloud_out'},
            {'topic_pointcloud_out': 'bf_lidar/point_cloud_pcl_example'}],
        )
    ])

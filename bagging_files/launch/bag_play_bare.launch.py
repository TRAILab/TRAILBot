import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    bag_filename_arg = DeclareLaunchArgument('bag_filename')

    #rviz launch
    rviz_config_path = os.path.join(get_package_share_directory('slam'),'config','rviz_config_bagging.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen')
                   
    ros2_bag_play_cmd = ExecuteProcess(
        cmd = ['ros2', 'bag', 'play', LaunchConfiguration('bag_filename'), '--clock'],
        name = 'rosbag_play',)
  


    return LaunchDescription([
        bag_filename_arg,
	    rviz_node,
        ros2_bag_play_cmd
    ])
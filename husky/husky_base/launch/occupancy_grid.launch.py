from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    return LaunchDescription([
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]),
        
        #pointcloud to laserscan conversion node
        Node(
            package='pointcloud_to_laserscan_converter', executable='pointcloud_to_laserscan_node',
            remappings=[('points2', '/velodyne_points'),],
            parameters=[{
                # 'target_frame': 'base_link',
                'target_frame': 'laser_frame',
                'transform_tolerance': 0.01,
                #base link height max/min
                # 'min_height': 0.1, #4 inches
                # 'max_height': 1.0,
                #laser_frame height max/min
                'min_height': -0.9282, #4 inches above ground 
                'max_height': 0.1, 
                'angle_min': -3.14,  #was -M_PI/2 now -PI
                'angle_max': 3.14,  #was  M_PI/2 now PI
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),

    ])
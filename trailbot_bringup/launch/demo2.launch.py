import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

from launch_ros.substitutions import FindPackageShare
from datetime import date
from datetime import datetime

#alias in ~/.bashrc: bot

def generate_launch_description():

    # # Launching SLAM (also includes driving and all other needed nodes)
    # slam_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','slam_3D.launch.py')
    # slam_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([slam_launch_path]))          

    # #the nav configs 
    # package_name = 'nav'

    # # Nav node
    # nav_launch_path = os.path.join(get_package_share_directory(package_name),'launch','navigation_launch.py')
    # nav_params_path = os.path.join(get_package_share_directory(package_name),'config','nav2_params_points.yaml')
    # nav_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([nav_launch_path]),
    #                                     launch_arguments={'namespace': '',
    #                                                     # 'use_sim_time': 'true',
    #                                                      'autostart': 'true',
    #                                                     'params_file': nav_params_path,
    #                                                     # 'use_lifecycle_mgr': 'false',
    #                                                     #'map_subscribe_transient_local': 'true'
    #                                                     }
    #                                                     .items())


   
    # rviz_config = os.path.join(pkg_share, 'rviz2', 'husky_sim_rviz.rviz')
    #Camera, ouster, driving Launch File
    bringup_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','trailbot_bringup.launch.py')
    bringup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([bringup_launch_path])) 

    #################################From Simulation Launch Code###############################
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch'
    )
    pkg_share = get_package_share_directory('nav')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False') #TODO
    map_dir = LaunchConfiguration('map',
        default=os.path.join(pkg_share, 'maps', 'my_map.yaml'))
    params_file = LaunchConfiguration('params_file',
        default=os.path.join(pkg_share, 'config', 'nav2_params_points.yaml'))
    autostart    = LaunchConfiguration('autostart', default='true')  # ADD

    # Declare arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(   # ADD
        'use_sim_time',
        default_value='false',
        description='Use simulated clock if true')

    declare_map_cmd = DeclareLaunchArgument(            # ADD
        'map',
        default_value=os.path.join(pkg_share, 'maps', 'my_map.yaml'),
        description='Full path to map yaml file for Nav2 bringup')

    declare_params_file_cmd = DeclareLaunchArgument(    # ADD
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params_points.yaml'),
        description='Full path to Nav2 parameter file')

    declare_autostart_cmd = DeclareLaunchArgument(      # ADD
        'autostart',
        default_value='true',
        description='Automatically startup the Nav2 stack')
    
    ###########################################################################################
    

    #FSM Launch File
    fsm_launch_path = os.path.join(get_package_share_directory('fsm'),'launch','fsm_launch.launch.py')
    fsm_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([fsm_launch_path]))
    
    # Nav3D and SLAM3D Launch File
    nav_2D_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','nav_2D.launch.py')
    nav_3D_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','nav_3D.launch.py')
    # nav_3D_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','nav_2D.launch.py')
    nav_2D_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([nav_2D_launch_path])) 
    nav_3D_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([nav_3D_launch_path])) 



    ####################### From simulation launch #######################################################
    # Nav2 bringup (used in Mode A; we’re just wiring args here)
    # nav2_bringup_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')
    #     ),
    #     launch_arguments={
    #         'map': map_dir,                 # uses declared arg
    #         'use_sim_time': use_sim_time,   # uses declared arg
    #         'params_file': params_file,     # uses declared arg
    #         'autostart': autostart          # uses declared arg
    #     }.items(),
    # )
    ######################################################################################################


    package_name = 'nav'
    nav_launch_path = os.path.join(get_package_share_directory(package_name),'launch','navigation_launch.py')
    nav_params_path = os.path.join(get_package_share_directory(package_name),'config','nav2_params_points.yaml')
    nav_node = IncludeLaunchDescription(PythonLaunchDescriptionSource([nav_launch_path]),
                                        launch_arguments={'namespace': '',
                                                        # 'use_sim_time': 'true',
                                                         'autostart': 'true',
                                                        'params_file': nav_params_path,
                                                        # 'use_lifecycle_mgr': 'false',
                                                        #'map_subscribe_transient_local': 'true'
                                                        }
                                                        .items())

    # fsm_node = Node(
    #     package='fsm',
    #     # executable='trailbot_fsm',
    #     executable='trailbot_fsm',
    #     name='fsm',
    #     output='screen'
    # )

    # fsm_nav_node = Node(
    #     package='fsm',
    #     # executable='navigator_node',
    #     executable='fsm_test',
    #     name='test_cmd_vel_node',
    #     output='screen'
    # )

    # #Logitech Camera Launch File
    package_dir = os.path.dirname(os.path.dirname(__file__))

    # Construct the path to the YAML parameter file
    params_file_path = os.path.join(package_dir, 'logitech_camera', 'config', 'params.yaml')
    
    # # Ensure the parameter file exists
    # if not os.path.exists(params_file_path):
    #     print(f"Parameter file {params_file_path} does not exist.")
    #     return

    # # Declare a launch argument for the parameter file path
    # params_file_arg = DeclareLaunchArgument(
    #     'params_file',
    #     default_value=params_file_path,
    #     description='Path to YAML parameter file'
    # )

    # Launch gscam_node for osmo action 4 camera
    osmo_cam = Node(
    package='gscam',
    executable='gscam_node',
    name='osmo_cam',
    output='screen',
    parameters=[
        {'gscam_config': 'v4l2src device=/dev/video0 ! image/jpeg,framerate=30/1,width=1920,height=1080  ! jpegdec ! videoconvert'},
        {'frame_id': 'camera'}
    ],
    remappings=[
                ('/camera/image_raw', '/camera'),
                ('/image_raw/compressed', '/camera/compressed'),
                ('/image_raw/compressedDepth', '/camera/compressedDepth'),
                ('/image_raw/theora', '/camera/theora')
            ]
)

    # Launch usb_cam node with parameters loaded from the YAML file
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        remappings=[
                ('/image_raw', '/camera'),
                ('/image_raw/compressed', '/camera/compressed'),
                ('/image_raw/compressedDepth', '/camera/compressedDepth'),
                ('/image_raw/theora', '/camera/theora')
            ],
        output='screen',
        parameters=[params_file_path]
        # arguments=['--ros-args', '--params-file', LaunchConfiguration('params_file')]
    )
    




    # camera_launch_path = os.path.join(get_package_share_directory('trailbot_bringup'),'launch','logitech_camera.launch.py')
    # camera_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([camera_launch_path]))





    # Launch voice_assistant/voice_assistant.launch.py which is voice interaction.
    launch_voice_assistant = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("voice_assistant"), 'launch', 'voice_assistant.launch.py'])))

    # Launch human detection
    human_detection_node = Node(
        package='human_detection',
        executable='human_detection_node'
    )
    nav2_path_publisher = Node(
        package='human_detection',
        executable='nav2_path_publisher'
    )

    pose_publisher = Node(
        package='human_detection',
        executable='pose_publisher'
    )

    # Launch trail detection
    trail_detection_node = Node(
        package='trail_detection_node',
        executable='trail_detection'
    )


    # launch the camera_shift node
    camera_shift_node = Node(
        package='camera_timestamp_shift',
        executable='camera_timestamp_shift',
        name='camera_timestamp_shift',
        output='screen'
        )



    # List of topics for bag that can run trail/human detection and navigation # '/camera',
    record_topics = ['/camera/shifted',
                    '/camera/compressed',
                    '/camera_info',
                    '/diagnostics',
                    '/dynamic_joint_states',
                    '/events/read_split',
                    '/husky_velocity_controller/cmd_vel_unstamped',
                    '/husky_velocity_controller/transition_event',
                    '/joint_state_broadcaster/transition_event',
                    '/joint_states',
                    '/joy_teleop/cmd_vel',
                    '/joy_teleop/joy',
                    '/odom',
                    '/ouster/imu',
                    '/ouster/metadata',
                    '/ouster/os_driver/transition_event',
                    '/ouster/points',
                    '/ouster/scan',
                    '/ouster/signal_image',
                    '/parameter_events',
                    'params_file',
                    '/robot_description',
                    '/rosout',
                    '/tf',
                    '/tracked_pose',
                    '/tf_static']

    # Launch file logging
    current_date = date.today()
    current_time = datetime.now().time()
    formatted_time = current_time.strftime("%H:%M")
    file_logging = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--include-hidden-topics', '-o', f'/home/trailbot/bags/{current_date}-{formatted_time}'] + record_topics
        # cmd=['ros2', 'bag', 'record', '-a', '--include-hidden-topics', '-o' f'/home/trailbot/bags/{current_date}-{formatted_time}'])
    )

    # ld.add_action(fsm_node)
    # ld.add_action(fsm_nav_node)
    # ld.add_action(camera_launch)
    # ld.add_action(slam_launch)
    # ld.add_action(nav_node)
    

    ld = LaunchDescription()

    # for snackbot demo
    # ld.add_action(bringup_launch)
    # ld.add_action(fsm_launch)
    # ld.add_action(nav_2D_launch) # nav_2D_launch
    # ld.add_action(usb_cam_node)
    # ld.add_action(human_detection_node)
    # ld.add_action(trail_detection_node)
    # ld.add_action(launch_voice_assistant)
    # ld.add_action(camera_shift_node)

    # for OpenNav demo
    #### Declared params ###### Do Not understand what this part is doing. need comments on this part or better name for the node.
    # ld.add_action(declare_use_sim_time_cmd)   # ADD
    # ld.add_action(declare_map_cmd)            # ADD
    # ld.add_action(declare_params_file_cmd)    # ADD
    # ld.add_action(declare_autostart_cmd)      # ADD
    #################################

    ld.add_action(bringup_launch)
    ld.add_action(nav_2D_launch) # use 2d slam
    # ld.add_action(nav_3D_launch) # use 3d slam
    #ld.add_action(nav2_bringup_launch)
    #ld.add_action(nav_node)
    # ld.add_action(usb_cam_node)
    ld.add_action(osmo_cam)
    # ld.add_action(nav2_path_publisher)
    ld.add_action(pose_publisher)
    ld.add_action(camera_shift_node)
    # ld.add_action(launch_voice_assistant)

    
    # ld.add_action(camera_launch)
    # ld.add_action(params_file_arg)
    # ld.add_action(camera_launch)

    logging = False
    if logging:
        ld.add_action(file_logging)
  
    return ld



# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# import os
# from ament_index_python.packages import get_package_share_directory
# from datetime import date, datetime

# def generate_launch_description():
#     pkg_share_nav = get_package_share_directory('nav')
#     nav2_bringup_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
#     trailbot_bringup_path = os.path.join(get_package_share_directory('trailbot_bringup'), 'launch', 'trailbot_bringup.launch.py')

#     # ---- Launch args (declare all you plan to use) ----
#     use_sim_time = LaunchConfiguration('use_sim_time')
#     map_yaml     = LaunchConfiguration('map')
#     params_file  = LaunchConfiguration('params_file')
#     autostart    = LaunchConfiguration('autostart')

#     declare_use_sim_time = DeclareLaunchArgument(
#         'use_sim_time', default_value='false',
#         description='Use simulated time (set true if bag playback with /clock).'
#     )
#     declare_map = DeclareLaunchArgument(
#         'map',
#         default_value=os.path.join(pkg_share_nav, 'maps', 'my_map.yaml'),
#         description='Full path to map.yaml'
#     )
#     declare_params = DeclareLaunchArgument(
#         'params_file',
#         default_value=os.path.join(pkg_share_nav, 'config', 'nav2_params_points.yaml'),
#         description='Full path to Nav2 params YAML'
#     )
#     declare_autostart = DeclareLaunchArgument(
#         'autostart', default_value='true',
#         description='Automatically transition Nav2 lifecycle nodes to active.'
#     )

#     # ---- Bring up robot hardware/sensors (your existing bringup) ----
#     bringup_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource([trailbot_bringup_path])
#     )

#     # ---- Nav2 bringup (map server + AMCL + planners/controllers) ----
#     # NOTE: Do NOT launch SLAM here; we’re loading a prebuilt map.
#     nav2_bringup = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(os.path.join(nav2_bringup_launch_dir, 'bringup_launch.py')),
#         launch_arguments={
#             'map': map_yaml,
#             'use_sim_time': use_sim_time,
#             'params_file': params_file,
#             'autostart': autostart
#         }.items()
#     )

#     # ---- USB cam (your config file path) ----
#     package_dir = os.path.dirname(os.path.dirname(__file__))
#     usb_params_file = os.path.join(package_dir, 'logitech_camera', 'config', 'params.yaml')
#     usb_cam_node = Node(
#         package='usb_cam',
#         executable='usb_cam_node_exe',
#         name='usb_cam_node',
#         remappings=[
#             ('/image_raw', '/camera'),
#             ('/image_raw/compressed', '/camera/compressed'),
#             ('/image_raw/compressedDepth', '/camera/compressedDepth'),
#             ('/image_raw/theora', '/camera/theora')
#         ],
#         output='screen',
#         parameters=[usb_params_file]
#     )

#     camera_shift_node = Node(
#         package='camera_timestamp_shift',
#         executable='camera_timestamp_shift',
#         name='camera_timestamp_shift',
#         output='screen'
#     )

#     pose_publisher = Node(
#         package='human_detection',
#         executable='pose_publisher',
#         name='pose_publisher',
#         output='screen'
#     )

#     # ---- Optional: bag recording (remove the stray "params_file") ----
#     record_topics = [
#         '/camera',
#         '/camera/shifted',
#         '/camera/compressed',
#         '/camera_info',
#         '/diagnostics',
#         '/dynamic_joint_states',
#         '/events/read_split',
#         '/husky_velocity_controller/cmd_vel_unstamped',
#         '/husky_velocity_controller/transition_event',
#         '/joint_state_broadcaster/transition_event',
#         '/joint_states',
#         '/joy_teleop/cmd_vel',
#         '/joy_teleop/joy',
#         '/odom',
#         '/ouster/imu',
#         '/ouster/metadata',
#         '/ouster/os_driver/transition_event',
#         '/ouster/points',
#         '/ouster/scan',
#         '/ouster/signal_image',
#         '/parameter_events',
#         '/robot_description',
#         '/rosout',
#         '/tf',
#         '/tracked_pose',
#         '/tf_static'
#     ]
#     current_date = date.today()
#     formatted_time = datetime.now().strftime("%H:%M")
#     rosbag_proc = ExecuteProcess(
#         cmd=['ros2', 'bag', 'record', '--include-hidden-topics',
#              '-o', f'/home/trailbot/bags/{current_date}-{formatted_time}'] + record_topics,
#         output='screen'
#     )

#     ld = LaunchDescription()
#     ld.add_action(declare_use_sim_time)
#     ld.add_action(declare_map)
#     ld.add_action(declare_params)
#     ld.add_action(declare_autostart)

#     # For OpenNav demo with prebuilt map:
#     ld.add_action(bringup_launch)
#     # ld.add_action(nav2_bringup)
#     ld.add_action(usb_cam_node)
#     ld.add_action(pose_publisher)
#     ld.add_action(camera_shift_node)

#     # Toggle bagging quickly here if you need it
#     enable_logging = False
#     if enable_logging:
#         ld.add_action(rosbag_proc)

#     return ld

# clone
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/uleroboticsgroup/simple_node.git
    $ git clone https://github.com/uleroboticsgroup/yasmin.git

# dependencies
    $ cd yasmin
    $ pip3 install -r requirements.txt

# colcon
    $ cd ~/ros2_ws
    $ colcon build

# add fsm node and navigator node to launch directory
From the marc/humble_carto branch add the fsm folder into the trailbot's launch directory

# modify launch file
Add the following to the launch file

    # FSM Node
    fsm_node = Node(
        package='fsm',
        executable='trailbot_fsm',
        name='fsm',
        output='screen'
    )
    
    # Navigator Node
    navigator_node = Node(
        package='fsm',
        executable='navigator_node',
        name='test_cmd_vel_node',
        output='screen'
    )

    ld.add_action(fsm_node)
    ld.add_action(navigator_node)

# modify the subscriber in the trailbot_fsm and navigator_node to subscribe to the perception and voice assistant node

# FYI
The fsm accepts PoseStamped msgs for its goal position i.e 

    ros2 topic pub --once /target_location geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 3.0, y: -3.0, z: 0.002472}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'







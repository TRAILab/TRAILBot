# Download dependencies
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/uleroboticsgroup/simple_node.git
    $ git clone https://github.com/uleroboticsgroup/yasmin.git

# Build dependencies
    $ cd yasmin
    $ pip3 install -r requirements.txt

# Build nodes 
    $ cd ~/ros2_ws
    $ colcon build


# To test in sim, run the nav_sim launch file

    

# To run on the Husky


## Add fsm node and navigator node to TRAILbot directory
    Starting from the marc/humble_carto branch, copy the fsm folder into the trailbot's base directory

## Modify Husky launch file
    Add the following to the Husky launch file

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

# Modify the subscriber in the trailbot_fsm and navigator_node to subscribe to the perception and voice assistant node
    
    TBD


# Send a goal location manually for testing purposes.

The fsm accepts PoseStamped msgs for its goal position. To test this manually, run: 

    ros2 topic pub --once /target_location geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {position: {x: 3.0, y: -3.0, z: 0.002472}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'








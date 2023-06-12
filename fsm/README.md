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

# add fsm node and navigator node to launch file
From the marc/humble_carto branch add the fsm folder into the trailbot's launch directory
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
    
# colcon and source

# modify the subscriber in the trailbot_fsm and navigator node to subscribe to the 
# perception and voice assistant node








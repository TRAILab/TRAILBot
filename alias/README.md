ALIASES
====
To create the Aliases for this program as laid out, follow these steps: 

1. Open a terminal.

2. Open your .bashrc file using a text editor. For example, you can use the following command to open it with the Nano editor:
```
nano ~/.bashrc
```

3. Go to the bottom of your editor or .bashrc file 

4. Add the following (make sure lines aren't duplicated from other installations/setups i.e. "source /opt/ros/humble/setup.bash may already be there from the ros2 setup)
```
source /opt/ros/humble/setup.bash
source ~/trail_ws/install/setup.bash
source ~/microros_ws/install/setup.bash
source ~/microros_ws/install/local_setup.bash

export ROS_LOCALHOST_ONLY=1
#export ROS_DOMAIN_ID=7 #microros currently only works with ros_domain_id 0

# Launch Arduino ide
alias arduino="~/Downloads/arduino-ide_2.1.0_Linux_64bit/arduino-ide"

# Run microros agent and operate servos
alias run_agent="ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/arduino -v6"
alias run_s1="ros2 service call /runservo trailbot_interfaces/srv/RunServo '{servo: 1}'"
alias run_s2="ros2 service call /runservo trailbot_interfaces/srv/RunServo '{servo: 2}'"
alias run_s3="ros2 service call /runservo trailbot_interfaces/srv/RunServo '{servo: 3}'"
alias run_s4="ros2 service call /runservo trailbot_interfaces/srv/RunServo '{servo: 4}'"

# Launch voice assistant node and change state to query
alias voice="ros2 launch voice_assistant voice_assistant.launch.py"
alias query="ros2 topic pub --once /trailbot_state std_msgs/msg/String '{data: 'QueryState'}'"

export ELEVENLABS_API_KEY="16e32e74342feed90fe7b638da80dc8d"
export OPENAI_API_KEY="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"

#sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null <<<0

save_map() {
        current_date=$(date +"$Y-%m-%d_%M")
        folder_path="$HOME/ros2_ws/saved_maps"

        ros2 run nav2_map_server map_saver_cli -f "$folder_path/$current_date"
}

# 3 Main Launch files
alias drive="ros2 launch trailbot_bringup driving.launch.py"
alias slam_2D="ros2 launch trailbot_bringup slam_2D.launch.py"
alias slam_3D="ros2 launch trailbot_bringup slam_3D.launch.py"
alias nav_2D="ros2 launch trailbot_bringup nav_2D.launch.py"
alias nav_3D="ros2 launch trailbot_bringup nav_3D.launch.py"
alias bot="ros2 launch trailbot_bringup demo1.launch.py"
```

5. Save and exit the file:
    - Press Ctrl+X.
    - Press Y to confirm saving changes.
    - Press Enter to exit Nano.

6. To make the changes take effect, run the following command in the terminal:
```
source ~/.bashrc
```
#


# These are the following Aliases and their uses 
| Command | Usage |
| ------- | --------------------------- |
| bot | Runs the full trailbot package (snack delivery) |
| slam_2D | Runs TRAILbot only using slam with 2D carto|
| slam_3D | Runs TRAILbot only using navigation with 3D carto|
| nav_2D | Runs TRAILbot only using navigation with 2D carto|
| nav_3D | Runs TRAILbot only using navigation with 3D carto|
| drive | Run when only wanting to drive/move TRAILbot |
| save_map | Use to save the map created by cartographer (requires directory ~/ros2_ws/saved_maps) |
| run_agent | Runs micro ros agent, use for running arduino|
| run_s* | Used to run different servo motors (* should be replaced with a number from 1-4) |
| voice | Runs voice assistant for snacks |
| query | Run to use querying with voice assistant |


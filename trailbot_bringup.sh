#! /bin/bash

ros2 launch trailbot_bringup logitech_camera.launch.py&
ros2 launch trailbot_bringup demo2.launch.py&
ros2 service call /ouster/set_config ouster_sensor_msgs/srv/SetConfig "{config_file: /home/trailbot/trail_ws/src/ouster-ros/ouster-ros/config/os-azimuth.json}"&
wait
echo "Trailbot Bringup Successful"

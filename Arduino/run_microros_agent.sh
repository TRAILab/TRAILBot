#!/bin/bash
set -e
set -x

cd ~/microros_ws 
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6


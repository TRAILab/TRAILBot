#!/bin/bash
set -e
set -x

# Add micro_ros_arduino precompiled library 
cd ~/Arduino/libraries
wget https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.5-humble.zip
unzip v2.0.5-humble.zip
rm v2.0.5-humble.zip

# Add patch
export ARDUINO_PATH=~/.arduino15/packages/arduino
cd $ARDUINO_PATH/hardware/sam/1.6.12/
curl https://raw.githubusercontent.com/micro-ROS/micro_ros_arduino/humble/extras/patching_boards/platform_arduinocore_sam.txt > platform.txt

scp -r ~/trail_ws/src/TRAILBot/trailbot_interfaces/ ~/microros_ws/src/

# Build trailbot_interfaces in microros_ws
cd ~/microros_ws
source /opt/ros/humble/setup.bash 
colcon build --packages-select trailbot_interfaces
source install/setup.bash

# Copy trailbot_interfaces in Arduino
scp -r ~/microros_ws/src/trailbot_interfaces/ ~/Arduino/libraries/micro_ros_arduino-2.0.5-humble/extras/library_generation/extra_packages/

# Build micro_ros_arduino library
cd ~/Arduino/libraries/micro_ros_arduino-2.0.5-humble
sudo docker pull microros/micro_ros_static_library_builder:humble
sudo docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:humble -p cortex_m3

# Symlink micro-ros_runservo_service in Arduino folder
ln -s ~/trail_ws/src/TRAILBot/Arduino/micro-ros_runservo_service/ ~/Arduino/

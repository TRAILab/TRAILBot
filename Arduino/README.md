# 1. Micro-ros run_servo_service node

## 1.1. Make trailbot_interfaces package available to micro-ros_runservo_service node:
1. Follow the instructions in [MicroRos_ArduinoDue_Setup_Guide](MicroRos_ArduinoDue_Setup_Guide.md) to install and setup microros with Arduino Due.
2. After building `trailbot_interfaces` package in ros2_ws, copy `trailbot_interfaces` package folder in `microros_ws/src/` folder. In a new terminal, build trailbot_interfaces in microros_ws again (not sure if this step is necessary. TODO: Verify and update README): 
   ```bash
   source /opt/ros/humble/setup.bash 
   cd microros_ws/
   colcon build --packages-select trailbot_interfaces

   # Check if ROS2 recognizes trailbot_interfaces
   source install/setup.bash
   ros2 interface show trailbot_interfaces/srv/RunServo 
   ```
3. After building `trailbot_interfaces` package in ros2_ws, copy `trailbot_interfaces` package folder in `/home/barza/Arduino/libraries/micro_ros_arduino-2.0.5-humble/extras/library_generation/extra_packages/` folder.
   
4. Build micro_ros_arduino library to include our custom trailbot_interfaces package:
   ```bash
   cd ~/Arduino/libraries/micro_ros_arduino-2.0.5-humble/

   sudo docker pull microros/micro_ros_static_library_builder:humble

   sudo docker run -it --rm -v $(pwd):/project --env MICROROS_LIBRARY_FOLDER=extras microros/micro_ros_static_library_builder:humble -p cortex_m3

   ```
5. If build was successful, you should be able to find `trailbot_interfaces/RunServo.srv` in ` ~/Arduino/libraries/micro_ros_arduino-2.0.5-humble/available_ros2_types` 

## 1.2. Upload to Arduino
Copy or Symlink `micro-ros_runservo_service` folder from `TRAILBot/Arduino/` to ` ~/Arduino/`. Open Arduino ide,  compile and upload `micro-ros_runservo_service.ino` in Arduino Due.

## 1.3. Run micro-ros agent and test run_servo_service node

1. Power Cycle/Reconnect Arduino Due to laptop via USB.

2. Open a new terminal and run microros agent with --dev (port connected to Arduino):

   ```bash
   cd ~/microros_ws

   # Source ros2
   source /opt/ros/humble/setup.bash 

   # Source local packages 
   source install/setup.bash

   # Run microros agent
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -v6
   ```

3. **Test node:** Open a new terminal and call `/runservo` service to see if you can operate servos:
   ```bash
   ros2 service call /runservo trailbot_interfaces/srv/RunServo {servo: 1}
   ros2 service call /runservo trailbot_interfaces/srv/RunServo {servo: 2}
   ros2 service call /runservo trailbot_interfaces/srv/RunServo {servo: 3}
   ros2 service call /runservo trailbot_interfaces/srv/RunServo {servo: 4}
   ```
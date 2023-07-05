Husky
=====

Common ROS packages for the Clearpath Husky, useable for both simulation and
real robot operation.

 - husky_control : Control configuration
 - husky_description : Robot description (URDF)
 - husky_msgs : Message definitions
 - husky_navigation : Navigation configurations and demos

For Husky instructions and tutorials, please see [Robots/Husky](http://wiki.ros.org/Robots/Husky).


Dependencies/Requirement to build:
=============
Install ROS2 Humble (follow guide below):
  - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Install NAV2 and Correlating Dependencies:
  - sudo apt install ros-humble-slam-toolbox
  - sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
  - sudo apt-get install ros-humble-turtlebot3*
  - sudo apt install ros-humble-twist-mux
  - sudo apt install ros-humble-gazebo-ros2-control

Install this dependency from the top folder (i.e. "ros2_ws") in your directory
```
cd ~/ros2_ws 
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```
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
```

IMU Setup For 3D Cartographer (not needed for 2D Cartographer):
=============
This setup is for the um7 Orientation Sensor which is placed in the bottom of the vending machine
- Link Here: https://www.pololu.com/product/2763

For Connection from UM7 to Laptop a USB to UART connector is needed
- We can only confirm that one with a CP2102 chip works


Callibration Process:
1. On a Windows Machine download the Redshift Serial Interface software under resources here:
  - https://www.pololu.com/product/2763/resources

2. Follow the Callibration Process here:
  https://redshiftlabs.com.au/support-services/um7-calibration-procedure/?v=3a1ed7090bfa

3. Recommendations and Tips
   - When callibrating follow the guide steps closely and use the videos as a reference
   - After following the steps make sure to zero the accelerometer and set the home position and flash again(this is important)


In order to use the um7 with ROS2 we need to use the following package:
```
cd ~/ros2_ws/src
mkdir IMU
cd IMU
mkdir src
cd src
git clone -b ros2 https://github.com/ros-drivers/um7.git
```

Follow the readme in the ros-drivers um7 package to see how to individually run the um7 with ROS2 and needed additional packages:
  - https://github.com/ros-drivers/um7/tree/ros2
Husky
=====

Common ROS packages for the Clearpath Husky, useable for both simulation and
real robot operation.

 - husky_control : Control configuration
 - husky_description : Robot description (URDF)
 - husky_msgs : Message definitions
 - husky_navigation : Navigation configurations and demos

For Husky instructions and tutorials, please see [Robots/Husky](http://wiki.ros.org/Robots/Husky).


Dependencies/Requirement to build
=============
Install ROS2 Humble (follow guide below):
  - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Install NAV2 and Correlating Dependencies:
  - sudo apt install ros-humble-slam-toolbox
  - sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
  - sudo apt-get install ros-humble-turtlebot3*
  - sudo apt install ros-humble-twist-mux
  - sudo apt install ros-humble-gazebo-ros2-control

Clone
```
cd ~/ros2_ws/src
git clone --recursive https://github.com/TRAILab/husky_trailbot.git
cd ..

```
build
```
colcon build --symlink-install

```
source
```
. install/setup.bash

```


Launch and Use Husky
=============

To run the needed launch file(s) to use the trailbot follow the steps below:

Making sure that the husky is turned input:
```
ros2 launch husky_base base.launch.py
```

This will now allow you to use the controller to move the husky as well as using navigation in RVIZ


Controller Information: 
=============
Teleop configuration for Logitech F710 Gamepad using the x-pad configuration.
- Left thumb-stick up/down for velocity, left/right for twist
- LB for enable
- RB for enable-turbo
```
        (LB)                                 (RB)
        (LT)                                 (RT)
      _=====_            D(  .)X            _=====_
     / _____ \                             / _____ \
   +.-'_____'-.---------------------------.-'_____'-.+
  /   |     |  '.                       .'  |      |   \
 / ___| /|\ |___ \ (back)(Lgtc)(strt)  / ___| (Y)  |___ \
/ |      |      | ;  __           __  ; |              | |
| | <---   ---> | | (__) .       (__) | | (X)       (B)| |
| |___   |   ___| ; MODE         VIBE ; |___       ____| /
|\    | \|/ |    /  _     ___      _   \    | (A) |    /|
| \   |_____|  .','" "', |___|  ,'" "', '.  |_____|  .' |
|  '-.______.-' /       \ANALOG/       \  '-._____.-'   |
|               |  LJ   |------|   RJ  |                |
|              /\       /      \       /\               |
|             /  '.___.'        '.___.'  \              |
|            /                            \             |
 \          /                              \           /
  \________/                                \_________/

BUTTON         Value
  LB             4
  RB             5
  A              0
  B              1
  X              2
  Y              3

   AXIS        Value
Left Horiz.      0
Left Vert.       1
Right Horiz.     3
Right Vert.      4
Left Trigger     2
Right Trigger    5
D-pad Horiz.     6
D-pad Vert.      7
```
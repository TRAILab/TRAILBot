# trailbot_gazebo

Husky Gazebo simulation package

## Installation

Install ros2-foxy as per the [website](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

Install slam_toolbox and nav2

```bash
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt-get install ros-humble-turtlebot3*
sudo apt install ros-humble-twist-mux
sudo apt install ros-humble-gazebo-ros2-control

```

## Building and sourcing
Assumes starting from a structure "... /{work directory}/src/TRAILbot/". From the TRAILBot directory, run in a terminal  
```bash
cd ../..
colcon build --symlink-install
. install/setup.bash
```

## Usage
For simulation only, run in a terminal window

```bash
ros2 launch trailbot_gazebo gazebo_sim.launch.py
```
For simulation with slam and navigation
```bash
ros2 launch trailbot_gazebo nav_sim.launch.py
```

## Keyboard control
To control the robot using the keyboard, in a separate terminal run. The terminal window must be the currently active window.
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
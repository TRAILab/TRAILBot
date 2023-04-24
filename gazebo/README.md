# trailbot_gazebo

Husky Gazebo simulation package

## Installation

Install ros2-foxy as per the [website](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

Install slam_toolbox and nav2

```bash
sudo apt install ros-foxy-slam-toolbox
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup
```

## Building and sourcing
Assumes a structure "... /{work directory}/src/TRAILbot/". From the TRAILBot directory, run in a terminal  
```bash
./build.sh
source ../../install/setup.bash
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

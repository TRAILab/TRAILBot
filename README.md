TRAILBOT README 
=====

SETUP GUIDE
=============
Sensor Drivers:

- Clone this repo and dependending repo's
```
cd ~/ros2_ws/src
git clone https://github.com/TRAILab/TRAILBot.git
git clone https://github.com/TRAILab/velodyne.git
git clone https://github.com/TRAILab/ximea-driver.git
cd ximea-driver
./tools/install_sdk.sh
cd ../..
```


Seperate Installations and Configurations:
============
Follow the husky setup readme for nav and control installations here:
- [Link to Husky and Nav README](husky/README.md)
Follow the build and package setup for the voice_assistant here:
- [Link to Voice Assistant README](voice_assistant/README.md)
Follow the build and testing setup for the human_detection here (build only if following example fully):
- [Link to Human Detection README](human_detection/readme.md)
Follow the download and build instructions for the FSM here:
- [Link to FSM README](fsm/README.md)
Follow the name setup for the arduino here:
- [Link to Arduino README](Arduino/README.md)



Build Everything Together After Dependencies are Installed (Everything is good as long as the build doesn't fail):
```
cd ~/ros2_ws
colcon build --symlink-install
```

Source before launching
```
. install/setup.bash
```


Different Launch Options for TRAILBOT
=============
There are 4 different launch modes you can choose to use:
- Make sure that the husky is turned on and everything is plugged in when doing so! 
1. Driving only mode:
```
ros2 launch trailbot_bringup driving.launch.py
```
2. SLAM/CARTO only mode:
```
ros2 launch trailbot_bringup slam_2D.launch.py
OR
ros2 launch trailbot_bringup slam_3D.launch.py
```
3. Navigation only mode:
```
ros2 launch trailbot_bringup nav_2D.launch.py
OR
ros2 launch trailbot_bringup nav_3D.launch.py
```
3. Full TRAILBOT (snack delivery) mode:
```
ros2 launch trailbot_bringup demo1.launch
```


Aliases that can be used 
====
- (follow setup here: [Alias Creation](alias/README.md))

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


CONTROLLER INSTRUCTIONS
================
After launching you can now use the controller to move the husky as well as using navigation in RVIZ
- Navigation in RVIZ can be used to move the robot using 2D Nav Goal
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
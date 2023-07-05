TRAILBOT
=====

SETUP GUIDE
=============
Sensor Drivers:

Clone this repo and dependending repo's
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
- [Link to Target README](husky/readme.md)

Follow the build and package setup for the voice_assistant here:
- [Link to Target README](voice_assistant/readme.md)

Follow the build and testing setup for the human_detection here (build only if following example fully):
- [Link to Target README](human_detection/readme.md)

Follow the download and build instructions for the fsm here:
- [Link to Target README](fsm/readme.md)

Follow the name setup for the arduino here:
- [Link to Target README](Arduino/readme.md)


Build Everything Together After Dependencies are Installed (Everything is good as long as the build doesn't fail):
```
colcon build --symlink-install
```

Source before launching
```
. install/setup.bash

```


Different Launch Options for TRAILBOT
=============
There are 3 different launch modes you can choose to use:
- Make sure that the husky is turned on and everything is plugged in when doing so! 

1. Driving only mode:
```
ros2 launch trailbot_bringup driving.launch.py
```

2. Navigation only mode:
```
ros2 launch trailbot_bringup nav.launch.py
```

3. Full TRAILBOT (snack delivery) mode:
```
ros2 launch trailbot_bringup demo1.launch
```


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

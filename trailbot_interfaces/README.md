# trailbot_interfaces
This package will have contain custom Trailbot services and msgs.

## Make include src 
```bash
cd ~/ros2_ws/src/TRAILBot/trailbot_interfaces
```

Add custom msg definitions in msg folder and edit package.xml and CMakeLists.txt accordingly
## Build package
```bash
cd ~/ros2_ws

# [Optional] Activate virtual env
source src/colcon_venv/venv/bin/activate

# Source ros2
source /opt/ros/humble/setup.bash 

# Build packages
colcon build --packages-select trailbot_interfaces

```
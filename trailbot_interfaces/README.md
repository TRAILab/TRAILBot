# trailbot_interfaces
This package will have contain custom Trailbot services and msgs.

## Build package
```bash
cd ~/trail_ws

# [Optional] Activate virtual env
source src/colcon_venv/venv/bin/activate

# Source ros2
source /opt/ros/humble/setup.bash 

# Build packages
colcon build --packages-select trailbot_interfaces

```
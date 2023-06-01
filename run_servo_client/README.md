# run_servo_client

## Build package
```bash
cd ~/trail_ws

# [Optional] Activate virtual env
source src/colcon_venv/venv/bin/activate

# Source ros2
source /opt/ros/humble/setup.bash 

# Build packages
colcon build --packages-select trailbot_interfaces run_servo_client

```

## Run package
```bash
cd ~/trail_ws

# [Optional] Activate virtual env
source src/colcon_venv/venv/bin/activate

# Source ros2
source /opt/ros/humble/setup.bash 

# Source ros2
source install/setup.bash 

# Run package to operate any of the servos e.g. servo 4
ros2 run run_servo_client client 4

```
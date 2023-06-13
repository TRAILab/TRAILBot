
## Arguments

### `-v` or `--verbose`

- Description: Enable print statements

### `-d` or `--download_model`

- Description: Download model from internet and save it into ./multipose_model

## Usage

To use the application, run the script and provide the desired arguments as command-line options. Here's an example:
```
ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera/compressed --remap out:=/camera
source install/setup.bash 
ros2 run human_detection human_detection
ros2 bag play testbag


cd ~/trail_ws
.install/setup.bash
ros2 launch husky_base base.launch.py
```

```bash
python LidarCameraSubscriber.py -v -k -i
```

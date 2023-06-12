
## Arguments

### `-v` or `--verbose`

- Description: Enable print statements

### `-d` or `--download_model`

- Description: Download model from internet and save it into ./multipose_model

## Usage

To use the application, run the script and provide the desired arguments as command-line options. Here's an example:
```
ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera/compressed --remap out:=/camera
ros2 run camera_listener camera_listener
ros2 bag play testbag
```

```bash
python LidarCameraSubscriber.py -v -k -i
```

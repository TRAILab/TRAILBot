## Configs (see `configs.yaml`)
- distance_where_lidar_stops_working: the minimum distance that the lidar can detect. This is around 0.4 for the current velodyne (2023 Aug)
- camera_transformation_k:
- rotation_matrix:
- translation_vector: 
- publishing_frequency: frequency to publish the detection location messages. If this is -1, the node will publish constantly (as camera FPS)


## Arguments

### `-v` or `--verbose`

- Description: Enable print statements

## Usage (new code using yolo_v7):
please see folder `yolov7_skeleton/`

## Usage (old code using movenet)

To use the application, run the script and provide the desired arguments as command-line options. Here's an example:
```
ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera/compressed --remap out:=/camera
source install/setup.bash 
colcon build --symlink-install
source install/setup.bash
ros2 run human_detection human_detection_node
ros2 bag play testbag


cd ~/trail_ws
.install/setup.bash
ros2 launch husky_base base.launch.py
```
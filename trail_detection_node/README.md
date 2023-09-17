SETUP GUIDE
=============
1. Create a new folder named ```model``` in trail_detection_node/trail_detection_node
2. Download ```psp_resnet50_pascal_voc_best_model.pth``` in NCRN Google Drive (Trail detection model folder) and put the file in the ```model``` folder
3. Download ```resnet50-25c4b509.pth``` in NCRN Google Drive (Trail detection model folder) and put the file in folder ```~/.torch/models```
4. Build the package

RUN the node
=============
```
ros2 run trail_detection_node trail_detection
```

Node Usage
=============
The node has three modes, which can be controlled by setting the ```only_camera_mode```(line 175) and ```visualize```(line 188) to different value: 
1. Trail centerline publisher(default, ```only_camera_mode=False```, ```visualize=False```): subscribes to camera and lidar and publishes the centerline point(posestamp type msg) in ```velodyne``` frame in ```trail_location``` topic
2. Trail centerline visualizer(```only_camera_mode=False```, ```visualize=True```): subscribes to camera and lidar and shows the chosen centerline point in **white**, the lidar points in **blue**, center points with no lidar points around in **yellow**, and center points with lidar points around in **red** in the image in the pop up window
3. Only Camera mode(```only_camera_mode=True```): subscribes only to camera, runs the segmentation model, and shows the segmented trail in image in the pop up window

Possible issues
=============
1. **Model doesn't load successfully:**
   Change the directory in the ```load model``` function in both ```trail_detection_node/trail_detection_node/v2_trailDetectionNode.py```(line 103 and 104) and ```trail_detection_node/trail_detection_node/visualizer_v2_trailDetectionNode.py```(line 107 and 110)
2. **Subfolder package relative import error:**
   Temporary fix: Move the script to the same folder and change the import code


    

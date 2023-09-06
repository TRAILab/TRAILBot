SETUP GUIDE
=============
1. Create a new folder named ```model``` in trail_detection_node/trail_detection_node
2. Download ```psp_resnet50_pascal_voc_best_model.pth``` in NCRN Google Drive (Trail detection model folder) and put the file in the ```model``` folder
3. Download ```resnet50-25c4b509.pth``` in NCRN Google Drive (Trail detection model folder) and put the file in folder ```~/.torch/models```
4. Build the package

RUN the node
=============
The package contains two nodes:
1. The node that takes the camera and lidar data and sends the Posestamp message
```
ros2 run trail_detection_node trail_detection
```
2. The node that takes the camera and lidar data and shows the visualization of the output
```
ros2 run trail_detection_node visualizer
```

Possible issues
=============
1. **Model doesn't load successfully:**
   Change the directory in the ```load model``` function in both ```trail_detection_node/trail_detection_node/v2_trailDetectionNode.py```(line 103 and 104) and ```trail_detection_node/trail_detection_node/visualizer_v2_trailDetectionNode.py```(line 107 and 110)
2. **Subfolder package relative import error:**
   Temporary fix: Move the script to the same folder and change the import code

Scripts explanation
=============
1. ```v1_trailDetectionNode.py``` and ```v2_trailDetectionNode.py```: two versions of ros2 node that sends the Posestamp message
2. ```visualizer_v1_trailDetectionNode.py``` and ```visualizer_v2_trailDetectionNode.py```: two versions of ros2 visualization node
3. ```GANav_visualizer.py```: ros2 visualization node for GANav model
4. ```jpu.py```, ```model_loader.py```, ```model_store.py```, ```segbase.py```, ```vgg.py```, and ```base_models``` folder: model loaders for FCN32S. FCN8s, and PSPNet. Copied from: https://github.com/Tramac/awesome-semantic-segmentation-pytorch
    

`yolov7.py ` is the yolov7 class for human tracking. It is taken from https://github.com/haroonshakeel/yolov7-object-tracking/blob/main/detect_or_track.py and modified

`models/`, `utils/`, `sort.py` are yolov7 helper files, downloaded from https://github.com/WongKinYiu/yolov7 (with no modification)

`example_usage.py` is an example that uses the yolov7 class. 

Usage:
1. download the model and test video here: https://utoronto-my.sharepoint.com/:f:/r/personal/nathan_hung_mail_utoronto_ca/Documents/yolov7?csf=1&web=1&e=pSSspq
2. run `sudo apt-get install ros-humble-vision-msgs`
3. run ```
python yolov7.py --weights-file yolov7.pt --nosave  --view-img --show-fps --show-track-lines --classes 0 --no-trace --source video.mp4
``` to run inference on __video.mp4__

to run inference on webcam, use ```
python yolov7.py --weights-file yolov7.pt --nosave  --view-img --show-fps --show-track-lines --classes 0 --no-trace --source webcam
```


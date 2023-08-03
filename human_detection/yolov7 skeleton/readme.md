`main.py ` is taken from https://github.com/haroonshakeel/yolov7-object-tracking/blob/main/detect_or_track.py and modified

`models/` `utils/` `sort.py` is downloaded from https://github.com/WongKinYiu/yolov7 (with no modification)

Usage:
1. download the model and test video here: https://utoronto-my.sharepoint.com/:f:/r/personal/nathan_hung_mail_utoronto_ca/Documents/yolov7?csf=1&web=1&e=pSSspq
2. run ```python main.py --weights-file yolov7.pt --nosave  --view-img --show-fps --show-track-lines --classes 0 --no-trace --source video.mp4 ``` to run inference on __video.mp4__

to run inference on webcam, use ```python main.py --weights-file yolov7.pt --nosave  --view-img --show-fps --show-track-lines --classes 0 --no-trace --source webcam```

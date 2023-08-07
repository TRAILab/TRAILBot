'''
NOTE:
change [ view_img=False ] to [ view_img=True ] to enable live preview 
'''

import yolov7
import torch
import cv2


with torch.no_grad():
    yolo_sort_tracker=yolov7.Yolo_sort_tracker() 
    mywebcam = cv2.VideoCapture(0)


while 1:
    with torch.no_grad():
        image_frame = mywebcam.read()[1]
        bounding_boxes, identities, confidences=yolo_sort_tracker.process_frame(image_frame,view_img=False)
        print("bounding_boxes:",bounding_boxes)
        print("identities:",identities)
        print("\n")

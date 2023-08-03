import argparse
import time
from pathlib import Path
import cv2
import torch
import numpy as np

from models.experimental import attempt_load
from utils.datasets import LoadImages
from utils.general import check_img_size, non_max_suppression, scale_coords, set_logging, increment_path
from utils.torch_utils import select_device, TracedModel
import sort

"""Function to draw bounding boxes"""
def draw_boxes(img, bbox, identities=None, categories=None, confidences=None, names=None, colors=None, thickness=2, hide_bounding_box=False,hide_labels=False):
    for i, box in enumerate(bbox):
        x1, y1, x2, y2 = [int(i) for i in box]
        tl = thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1  # line/font thickness

        cat = int(categories[i]) if categories is not None else 0
        id = int(identities[i]) if identities is not None else 0
        # conf = confidences[i] if confidences is not None else 0

        color = colors[cat]
        
        if not hide_bounding_box:
            cv2.rectangle(img, (x1, y1), (x2, y2), color, tl)

        if not hide_labels:
            label = str(id) + ":"+ names[cat] if identities is not None else  f'{names[cat]} {confidences[i]:.2f}'
            tf = max(tl - 1, 1)  # font thickness
            t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
            c2 = x1 + t_size[0], y1 - t_size[1] - 3
            cv2.rectangle(img, (x1, y1), c2, color, -1, cv2.LINE_AA)  # filled
            cv2.putText(img, label, (x1, y1 - 2), 0, tl / 3, [225, 255, 255], thickness=tf, lineType=cv2.LINE_AA)

    return img

"""Function to draw tracking lines"""
def draw_track_lines(im0, tracks, sort_tracker, thickness):
    for t, track in enumerate(tracks): # loop over tracks
        track_color = sort_tracker.color_list[t] # Get the color for the current track from the color_list of sort_tracker
        for i in range(len(track.centroidarr) - 1): # Iterate over the centroids in the track
            current_centroid = track.centroidarr[i]
            next_centroid = track.centroidarr[i + 1]
            current_point = (int(current_centroid[0]), int(current_centroid[1]))
            next_point = (int(next_centroid[0]), int(next_centroid[1]))
            cv2.line(im0, current_point, next_point, track_color, thickness=thickness)
            
class Yolo_sort_tracker:
    def __init__(self,
        device='cpu',
        weights_file='yolov7.pt',
        img_size=640,
        traced_model_already_exists=True,
        source='webcam',
        save_result=False,
        output_folder='/',
        output_file_name='yolov7results.mp4'
        ):
        # Initialize
        set_logging()
        self.device = select_device(device)
        self.use_half_precision = self.device.type != 'cpu'  # enable half precision if on GPU (only supported on CUDA)

        # Load model
        self.model = attempt_load(weights_file, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride, which is the step size or the number of units the sliding window moves when performing operations like convolution or pooling
        self.imgsize = check_img_size(img_size, s=self.stride)  # check img_size
        if not traced_model_already_exists:
            self.model = TracedModel(self.model, self.device, img_size)
        if self.use_half_precision:
            self.model.half()  # to FP16

        # Run inference once if on GPU. Not sure why or even if this is necessary. ### to be tested
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsize, self.imgsize).to(self.device).type_as(next(self.model.parameters())))

        # Set Dataloader
        self.vid_path, self.vid_writer = None, None
        torch.backends.cudnn.benchmark = True  # set True to speed up constant image size inference

        # defining option flags
        self.save_dir = Path(increment_path(Path(output_folder) / output_file_name))  # increment run
        self.save_result=save_result
        if save_result:
            self.save_dir.mkdir(parents=True)  # make dir

        # Names and colors of the detected objects' classes
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in self.names]

        # Initialize SORT tracker
        self.sort_tracker = sort.Sort(max_age=5, min_hits=2, iou_threshold=0.2) 

    def process_video_file(self,opt):
        self.dataset = LoadImages(opt.source, img_size=self.imgsize, stride=self.stride)
        for path, img, img_original, vid_cap in self.dataset:
            self.detect(img, self.imgsize, img_original, 
                path=path, 
                vid_cap=vid_cap,
                show_fps=opt.show_fps, 
                view_img=opt.view_img, 
                show_track_lines=opt.show_track_lines,
                line_thickness=opt.thickness,
                disable_tracking=opt.disable_tracking,
                hide_bounding_box=opt.nobbox,
                hide_labels=opt.nolabel,
                enable_augment=opt.augment,
                conf_thres=opt.conf_thres,
                iou_thres=opt.iou_thres,
                detection_object_classes=opt.classes,
                enable_agnostic_nms=opt.agnostic_nms)


    def process_frame(self, image_frame, view_img=False):
        img_original = np.array(image_frame)
        img = cv2.resize(image_frame, (576,640))        
        img = np.array(img).transpose(2, 0, 1)
        return self.detect(img, self.imgsize, img_original,view_img=view_img)


    def detect(self, 
        img, imgsize, im0, 
        path=None, # for saving result
        vid_cap=False, # for saving result
        show_fps=True, # for viewing result
        view_img=True, # for viewing result
        show_track_lines=True, # for viewin result
        line_thickness=2, # for viewing result
        disable_tracking=False, 
        hide_bounding_box=False, # for viewing result
        hide_labels=False, # for viewing result
        enable_augment=True,
        conf_thres=0.25,
        iou_thres=0.45,
        detection_object_classes=[0,],
        enable_agnostic_nms=True
        ):
        startTime = time.time()
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.use_half_precision else img.float()  # uint8 to FP16 or FP32
        img /= 255.0  # 0~255 to 0.0~1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Warmup. Not sure why or even if this is necessary. ### to be tested. ### maybe a flag should be added to avoid multiple warmups
        if self.device.type != 'cpu' and (img.shape[0]!=1 or imgsize != img.shape[2] or imgsize != img.shape[3]):
            for i in range(3):
                self.model(img, augment=enable_augment)[0]

        # Inference
        pred = self.model(img, augment=enable_augment)[0]

        # Apply NMS
        pred = non_max_suppression(pred, conf_thres, iou_thres, classes=detection_object_classes, agnostic=enable_agnostic_nms)

        #TESTING ###to be removed
        if len(pred)!=1:
            print("\n WARNING IN YOLOV7: if len(pred)!=1: ",len(pred))
            exit()

        # Process detections
        for i, det in enumerate(pred):  # detections per image
            output_string = ''

            if len(det)!=0:
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                dets_to_sort = np.empty((0,6))
                # NOTE: We send in detected object class too
                for x1,y1,x2,y2,conf,detclass in det.cpu().detach().numpy():
                    dets_to_sort = np.vstack((dets_to_sort, np.array([x1, y1, x2, y2, conf, detclass])))

                if not disable_tracking:
                    tracked_dets = self.sort_tracker.update(dets_to_sort, unique_color=True)
                    tracks =self.sort_tracker.getTrackers()
                    if len(tracked_dets)>0:
                        bbox_xyxy = tracked_dets[:,:4]
                        identities = tracked_dets[:, 8]
                        categories = tracked_dets[:, 4]
                        confidences = None
                        if show_track_lines:
                            draw_track_lines(im0, tracks, self.sort_tracker, line_thickness)
                    else:
                        ### not sure if this is possible
                        bbox_xyxy = dets_to_sort[:,:4]
                        identities = None
                        categories = dets_to_sort[:, 5]
                        confidences = dets_to_sort[:, 4]
                        print("if len(tracked_dets)>0 == FALSE!!!!")
                        # exit()
                else:
                    bbox_xyxy = dets_to_sort[:,:4]
                    identities = None
                    categories = dets_to_sort[:, 5]
                    confidences = dets_to_sort[:, 4]
                # draw bounding boxes for visualization
                im0 = draw_boxes(im0, bbox_xyxy, identities, categories, confidences, self.names, self.colors, line_thickness,hide_bounding_box,hide_labels)

                # prepare print results
                for c in det[:, -1].unique():
                    n = (det[:, -1] == c).sum()  # detections per class
                    output_string += f"{n} {self.names[int(c)]}, "  # add to string
            else:
                bbox_xyxy=None

            print(f'[INFO] {output_string}')

            # Show result on live cv2 window view: FPS
            if show_fps :
                currentTime = time.time()
                fps = 1/(currentTime - startTime)
                cv2.putText(im0, "FPS: " + str(round(fps, 4)), (20, 70), cv2.FONT_HERSHEY_PLAIN, 2, (0,255,0),2)
            # Show result on live cv2 window view: image
            if view_img:
                cv2.imshow("yolov7 preview", im0)
                cv2.waitKey(1)  # 1 millisecond


            # Save results (image with detections) to local file.
            if self.save_result and path!=None:
                path = Path(path)  # to Path
                save_path = str(self.save_dir / path.name)  # img.jpg
                # if self.dataset.mode == 'image':
                #     cv2.imwrite(save_path, im0)
                #     print(f" The image with the result is saved in: {save_path}")
                # else:  # 'video' or 'stream'
                if self.vid_path != save_path:  # new video
                    self.vid_path = save_path
                    if isinstance(self.vid_writer, cv2.VideoWriter):
                        self.vid_writer.release()  # release previous video writer
                    if vid_cap:  # video
                        fps = vid_cap.get(cv2.CAP_PROP_FPS)
                        w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    else:  # stream
                        fps, w, h = 30, im0.shape[1], im0.shape[0]
                        save_path += '.mp4'
                    self.vid_writer = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
                self.vid_writer.write(im0)
        return bbox_xyxy


if __name__ == '__main__':
    np.random.seed(0) # make outputs reproducible

    parser = argparse.ArgumentParser()
    # Files and devices:
    parser.add_argument('--weights-file', nargs='+', type=str, default='yolov7.pt', help='model.pt path(s)')
    parser.add_argument('--no-trace', action='store_true', help='don`t trace model (if traced_model.pt already exist this can save time)') # Model tracing determines all the operations that are executed when a model parses input data through its linear layers. Just like downloading a model, it only needs to run once. Once the traced_model.pt is generated, this operation is no longer needed
    parser.add_argument('--source', type=str, default='inference/images', help='video source to process')  # mp4 file/folder, 'webcam' for webcam
    parser.add_argument('--device', default='', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
    # Hyperparameters:
    parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
    parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
    parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS') # Non-maximum suppression is a post-processing step to remove duplicate and overlapping bounding boxes. Intersection over Union (IOU) is a metric used to measure the overlap between two bounding boxes. The IOU threshold controls how strictly the algorithm filters out overlapping bounding boxes. A higher IOU threshold will result in more aggressive suppression and fewer overlapping boxes being retained, while a lower threshold will allow more boxes to survive, even if they partially overlap with each other.
    # What to output
    parser.add_argument('--view-img', action='store_true', help='display results')
    parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
    parser.add_argument('--project', default='runs/detect', help='save results to project/name')
    parser.add_argument('--name', default='exp', help='save results to project/name')

    # yolov7 detection options 
    parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
    parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
    parser.add_argument('--augment', action='store_true', help='augmented inference')
    # SORT tracking options
    parser.add_argument('--disable-tracking', action='store_false', help='disable tracking')
    # Appearance option (what to display on screen or output video)
    parser.add_argument('--show-track-lines', action='store_true', help='show tracked path')
    parser.add_argument('--show-fps', action='store_true', help='show fps')
    parser.add_argument('--thickness', type=int, default=2, help='bounding box and font size thickness')
    parser.add_argument('--nobbox', action='store_true', help='don`t show bounding box')
    parser.add_argument('--nolabel', action='store_true', help='don`t show label')

    opt = parser.parse_args()


    with torch.no_grad(): #deactivate the autograd engine to save memory and speed up computations. On cpu, the speed is 15% faster with this
        yolo_sort_tracker=Yolo_sort_tracker(
            device=opt.device,
            weights_file=opt.weights_file,
            img_size=opt.img_size,
            traced_model_already_exists=opt.no_trace,
            source=opt.source,
            save_result=not opt.nosave,
            output_folder=opt.project,
            output_file_name=opt.name
            ) 

        if opt.source=='webcam':
            mywebcam = cv2.VideoCapture(0)
            while 1:
                _, image_frame  = mywebcam.read()
                bounding_boxes=yolo_sort_tracker.process_frame(image_frame,view_img=True)
                print(bounding_boxes)
        else:
            yolo_sort_tracker.process_video_file(opt) 

### fix up file specific stuffs
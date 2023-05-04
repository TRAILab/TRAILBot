# this subscriber receives img msg and process and save/imshow

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from detectron2.config import get_cfg
from detectron2.utils.logger import setup_logger
setup_logger()
import cv2, torch

# import some common detectron2 utilities
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog


class camSubscriber(Node):

    def __init__(self):
        super().__init__('cam_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

        #get cfg and predictor ready
        self.cfg = get_cfg()
        self.cfg.MODEL.DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
        print("GPU running: ", torch.cuda.is_available())
        # load the pre trained model from Detectron2 model zoo
        self.cfg.merge_from_file(model_zoo.get_config_file("COCO-Keypoints/keypoint_rcnn_R_50_FPN_3x.yaml"))
        # set confidence threshold for this model
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5  
        # load model weights
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-Keypoints/keypoint_rcnn_R_50_FPN_3x.yaml")
        # create the predictor for pose estimation using the config
        self.predictor = DefaultPredictor(self.cfg)
        self.frames = []

        self.i = 0

    def listener_callback(self, msg):
        self.get_logger().info('Image %d Received.' % self.i)
        self.i = self.i + 1

        #convert msg back to img
        image = self.bridge.imgmsg_to_cv2(msg)
        
        #process the img and show
        outputs = self.predictor(image)
        v = Visualizer(image[:, :, ::-1], MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0]), scale=1.2)
        v = v.draw_instance_predictions(outputs["instances"].to("cpu"))
        im = v.get_image()[:, :, ::-1]
        cv2.imshow('object_detection', im)

        self.frames.append(im)


def main(args=None):
    rclpy.init(args=args)

    im_subscriber = camSubscriber()

    try:
        rclpy.spin(im_subscriber)
    except KeyboardInterrupt:
        pass
    
    height, width, channels = im_subscriber.frames[0].shape
    out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'MJPG'), 5, (width, height))

    for frame in im_subscriber.frames:
        out.write(frame)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    im_subscriber.destroy_node()
    rclpy.shutdown()

    im_subscriber.out.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

import json
import os
from queue import Queue

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import gradio as gr
import numpy as np
import hydra
from omegaconf import DictConfig

def change_name_by_idx(path, uuid):
    base, ext = os.path.splitext(path)
    new_path = f"{base}_{uuid}{ext}"
    os.rename(path, new_path)

class CameraListener(Node):
    def __init__(self, cfg: DictConfig = None) -> None:
        super().__init__('camera_listener')
        self.cfg: DictConfig = cfg
        self.subscription = self.create_subscription(
            Image,
            '/front_color_camera/c_rgb/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.latest_frame = None  # store as numpy array

    def image_callback(self, msg: str) -> None:
        # Convert ROS Image → CV / numpy
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        # Store the latest
        print("Listenning...")
        with open(self.cfg.update_flag_path, "r") as f:
            data = json.load(f)

        print(data['trigger'])
        if data['trigger']:
            print("Saving data...")
            data['trigger'] = False
            self.latest_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            image_path = os.path.join(self.cfg.rgb_folder, f"captured_rgb_0.png")
            cv2.imwrite(image_path, self.latest_frame)
            with open(self.cfg.update_flag_path, "w") as f:
                json.dump(data, f)

@hydra.main(version_base=None, config_path="../configs", config_name="config")
def main(cfg: DictConfig) -> None:
    # Initialize ROS
    rclpy.init()
    listener: Node = CameraListener(cfg['inference'])
    rclpy.spin(listener)

    # Run ROS spinning in a thread
    # ros_thread = threading.Thread(target=lambda: rclpy.spin(listener), daemon=True)
    # ros_thread.start()

if __name__ == '__main__':
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os 

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.save_path = "/home/trailbot/action4/20260227/checkerboard1/outdoor_ultra/camera_data"
        if not os.path.exists(self.save_path):
            os.makedirs(self.save_path)
        self.subscription = self.create_subscription(
            Image,
            # 'camera',
            'camera/shifted',
            self.listener_callback,
            10)
        self.subscription
        self.bridge = CvBridge()
        self.i = 0

    def listener_callback(self, msg):
        self.i = self.i + 1
        timestamp = msg.header.stamp
        # Get the image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        print(f"cv_image shape: {cv_image.shape}, type: {type(cv_image)}")
        # cv2.imwrite(f"output_{self.i}.jpg", cv_image)
        img_name = os.path.join(self.save_path, f"frame_{self.i:04d}.jpg")
        cv2.imwrite(img_name, cv_image, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
        
        filename = os.path.join(self.save_path, f"output_{self.i}.txt")
        #get the timestamp
        # filename = f"output_{self.i}.txt"
        with open(filename, "w") as output_file:
           output_file.write(f"Timestamp: {timestamp}\n")
        print("image captured ", self.i)

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
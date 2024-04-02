import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'camera',
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


        # Convert YUV422_YUY2 encoding (logitech) to RGB 
        # cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_YUV2BGR_YUY2)
        rbg_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) #colours were originally flipped in RGB
        cv2.imwrite(f"output_{self.i}.jpg", rbg_image)
        
        #get the timestamp

        filename = f"output_{self.i}.txt"
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

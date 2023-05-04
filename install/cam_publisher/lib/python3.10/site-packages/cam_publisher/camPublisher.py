# this publisher capture the image from videocap and then convert image and publish

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2

class CamPublisher(Node):

    def __init__(self):
        super().__init__('cam_publisher')
        self.publisher_ = self.create_publisher(Image, 'topic', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        #use webcam
        self.cap = cv2.VideoCapture(0)
        
        #get ready to convert img to msg
        self.bridge = CvBridge()
        

    def timer_callback(self):
        ret, image = self.cap.read()
        
        # if ret, convert image and then publish
        if ret:
            self.image = self.bridge.cv2_to_imgmsg(image)
            self.publisher_.publish(self.image)
            self.get_logger().info('Publishing: No."%d" Image.' % self.i)
            self.i += 1


def main(args=None):
    rclpy.init(args=args)

    image_publisher = CamPublisher()

    rclpy.spin(image_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_publisher.destroy_node()
    rclpy.shutdown()


    # release resources and close the windows
    image_publisher.cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
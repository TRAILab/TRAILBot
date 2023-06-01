#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

class TFEchoNode(Node):
    def __init__(self):
        super().__init__('tf2_listener')
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(TransformStamped, 'tf2_listener', 10)
        self.create_timer(0.1, self.timer_callback)  # set the rate here

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            self.publisher.publish(transform)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transform available yet, waiting...')

def main():
    rclpy.init()
    node = TFEchoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




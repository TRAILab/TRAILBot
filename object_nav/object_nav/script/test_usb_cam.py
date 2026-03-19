#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import matplotlib.pyplot as plt
import threading

class CameraTimestampPlotter(Node):
    def __init__(self):
        super().__init__('camera_timestamp_plotter')
        self.timestamps = []
        self.timestamps_now = []
        self.msg_count = 0
        self.max_msgs = 20
        self.finished = False

        self.subscription = self.create_subscription(
            Image,
            '/camera',  # change to '/usb_cam/image_raw' if needed
            self.callback,
            10
        )

    def callback(self, msg):
        if self.finished:
            return

        ts_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ts_now = self.get_clock().now().seconds_nanoseconds()[0] + \
                 self.get_clock().now().seconds_nanoseconds()[1] * 1e-9

        self.timestamps.append(ts_msg)
        self.timestamps_now.append(ts_now)

        self.get_logger().info(f"msg: {ts_msg:.6f}, now: {ts_now:.6f}, diff: {ts_now - ts_msg:.6f}")

        self.msg_count += 1
        if self.msg_count >= self.max_msgs:
            self.finished = True
            self.get_logger().info(f"Reached {self.max_msgs} messages. Shutting down...")

def ros_spin(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = CameraTimestampPlotter()

    thread = threading.Thread(target=ros_spin, args=(node,), daemon=True)
    thread.start()

    # Wait for data collection to finish
    while not node.finished:
        pass

    # Plot after collecting all data
    plt.figure()
    plt.plot(node.timestamps, '-o', label='msg.header.stamp')
    plt.plot(node.timestamps_now, '-o', label='node.now()')
    plt.title("Camera Timestamps")
    plt.xlabel("Message Index")
    plt.ylabel("Timestamp (seconds)")
    plt.legend()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

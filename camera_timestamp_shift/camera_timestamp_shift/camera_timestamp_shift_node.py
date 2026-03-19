#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import numpy as np

class CameraTimestampShift(Node):
    def __init__(self):
        super().__init__('camera_timestamp_shift')

        self.max_samples = 300
        self.offset_list = []

        self.offset_sec = None
        self.offset_ready = False

        self.lidar_started = False
        self.camera_started = False

        self.lidar_sub = self.create_subscription(PointCloud2, '/ouster/points', self.lidar_callback, 10)
        self.camera_sub = self.create_subscription(Image, '/camera', self.camera_callback, 10)

        self.pub = self.create_publisher(Image, '/camera/shifted', 10)

        self.latest_lidar_time = None

    def lidar_callback(self, msg):
        self.lidar_started = True
        self.latest_lidar_time = self.to_sec(msg.header.stamp)

    def camera_callback(self, msg):
        cam_time = self.to_sec(msg.header.stamp)
        self.camera_started = True

        # Only calculate offset when both camera and lidar have started
        if self.lidar_started and self.camera_started and not self.offset_ready:
            if self.latest_lidar_time is None:
                return  # Skip until LiDAR gives a valid timestamp

            diff = self.latest_lidar_time - cam_time
            self.offset_list.append(diff)
            self.get_logger().info(f"[Collecting offset] Sample {len(self.offset_list)}/{self.max_samples}: {diff:.6f}s")

            if len(self.offset_list) >= self.max_samples:
                self.offset_sec = float(np.median(self.offset_list))
                self.offset_ready = True
                self.get_logger().info(f"✅ Offset stabilized at {self.offset_sec:.6f}s — starting timestamp shifting.")

        # Always publish something — optionally pass-through before ready
        if self.offset_ready:
            msg = self.shift_camera_timestamp(msg, self.offset_sec)
            self.pub.publish(msg)
            #print(f"[Publishing] Camera timestamp shifted by {self.offset_sec:.6f}s")

    def shift_camera_timestamp(self, msg, offset_sec):
        old_sec, old_nsec = msg.header.stamp.sec, msg.header.stamp.nanosec
        total_nsec = old_sec * 1e9 + old_nsec + int(offset_sec * 1e9)
        msg.header.stamp.sec = int(total_nsec // 1e9)
        msg.header.stamp.nanosec = int(total_nsec % 1e9)
        return msg

    @staticmethod
    def to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

def main(args=None):
    rclpy.init(args=args)
    node = CameraTimestampShift()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, Image  # or CompressedImage
from tf2_msgs.msg import TFMessage

import csv


class TimestampLogger(Node):
    def __init__(self):
        super().__init__('timestamp_logger')

        self.lidar_time = None
        self.camera_time = None
        self.tf_time = None

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.lidar_callback,
            10)

        self.camera_sub = self.create_subscription(
            Image,  # <- or CompressedImage
            '/camera',
            self.camera_callback,
            10)

        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)

        self.csv_file = open('timestamp_diffs_oct2.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'lidar_time', 'camera_time', 'tf_time',
            'lidar-camera', 'lidar-tf', 'camera-tf'
        ])

    def lidar_callback(self, msg):
        self.lidar_time = self.to_sec(msg.header.stamp)
        self.try_write()

    def camera_callback(self, msg):
        self.camera_time = self.to_sec(msg.header.stamp)
        self.try_write()

    def tf_callback(self, msg):
        if msg.transforms:
            self.tf_time = self.to_sec(msg.transforms[0].header.stamp)
            self.try_write()

    def try_write(self):
        if self.lidar_time and self.camera_time and self.tf_time:
            diff_lc = self.lidar_time - self.camera_time
            diff_lt = self.lidar_time - self.tf_time
            diff_ct = self.camera_time - self.tf_time

            self.csv_writer.writerow([
                self.lidar_time, self.camera_time, self.tf_time,
                diff_lc, diff_lt, diff_ct
            ])

    @staticmethod
    def to_sec(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TimestampLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

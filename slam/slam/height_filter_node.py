#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import yaml
from std_msgs.msg import Header

class HeightFilterNode(Node):
    def __init__(self, configs):
        super().__init__('height_filter_node')
        self.declare_parameter('input_topic', '/ouster/points')
        self.declare_parameter('output_topic', '/filtered_points')
        self.declare_parameter('min_height', -0.8)
        self.declare_parameter('max_height', 0.5)
        
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value
        
        self.configs = configs
        camera_transformation_k = configs['camera_transformation_k']
        self.camera_transformation_k = read_space_separated_matrix(camera_transformation_k)
        rotation_matrix = configs['rotation_matrix']

        self.radial_distortion = np.array([0.0, 0.0, 0, 0])

        self.T_CL = np.zeros((3,4))
        self.T_CL[:,:3] = read_space_separated_matrix(rotation_matrix)
        self.T_CL[:,3] = np.array(configs['translation_vector'])

        self.image_height = configs['image_height']
        self.image_width = configs['image_width']


        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.filter_callback,
            10
        )
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)

    def convert_to_camera_frame(self, point_cloud):
        """
        convert 3d lidar data into 2d coordinate of the camera frame + depth
        """
        point_cloud = np.hstack((point_cloud, np.ones((point_cloud.shape[0], 1))))
   
        N = point_cloud.shape[0]
        # print(point_cloud.shape)
        points3d_cam_frame = self.T_CL @ point_cloud.T # (3, N)

        uv_coordinate = np.zeros((3, N))

        uv_coordinate[0,:] = points3d_cam_frame[0,:] / points3d_cam_frame[2,:] #x / z
        uv_coordinate[1,:] = points3d_cam_frame[1,:] / points3d_cam_frame[2,:] #y / z
        # uv_coordinate[2,:] = points3d_cam_frame[2,:] # z
        uv_coordinate[2,:] = 1

        pixel_coords = self.camera_transformation_k @ uv_coordinate # 3 x N
        # pixel_coords[1,:] = 720 - pixel_coords[1,:]

        points2d = pixel_coords.T # N x 3
        dist = np.sqrt((point_cloud[:,:3]**2).sum(axis=1))
        ids = (points2d[:,0]>0) * (points2d[:,0]<1280) * (points2d[:,1]>0) * (points2d[:,1]<720) * (dist > 0.5)
        #filtered_points = points2d[ids,:]#[point_cloud[:,2] > 0,:]
        #self.filtered = points2d[ids,:]
        #self.filtered[:,2] = dist[ids]/10*255 # depth consideration
        # print(np.hstack((point_cloud[ids,:],np.array([dist[ids]]).T)))
        return ids
    
    def filter_callback(self, msg):
        # Extract "x", "y", and "z" fields using pc2.read_points generator
        point_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        # Convert point generator to a NumPy array
        points = np.array([point_gen['x'], point_gen['y'], point_gen['z']]).T
        
        # If the point cloud is empty, log a warning and return
        if points.shape[0] == 0:
            self.get_logger().warn("Received empty point cloud.")
            return

        # Filter out rows where all coordinates are zero (optional safety check)
        #mask = ~np.all(points[:, 1]<0.0, axis=1)
        

        # Apply height filtering (z-axis filtering)
        min_height = self.min_height
        max_height = self.max_height
        # ids_fov = self.convert_to_camera_frame(points)
        # points_fov = points[ids_fov,:]
        # height_mask = (points_fov[:, 2] >= min_height) & (points_fov[:, 2] <= max_height) & (points_fov[:, 1] > 0)
        # filtered_points = points_fov[height_mask]
        height_mask = (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
        filtered_points = points[height_mask]

        # Log debug information
        # self.get_logger().info(f"Total points received: {points.shape[0]}")
        # self.get_logger().info(f"Filtered points count: {filtered_points.shape[0]}")
        # self.get_logger().info(f"Height range of filtered points: Min = {np.min(filtered_points[:, 2])}, Max = {np.max(filtered_points[:, 2])}")
        # self.get_logger().info(f"fil: {filtered_points.shape}")
        # self.get_logger().info(f"point: {points.shape}")                   
        # self.get_logger().info(f"Sample filtered point (x, y, z): {filtered_points[0] if filtered_points.shape[0] > 0 else 'No points'}")

        # Rebuild the PointCloud2 message with the filtered points
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = msg.header.frame_id
         #'os_lidar'  # Set to the LiDAR frame

        filtered_msg = pc2.create_cloud_xyz32(header, filtered_points)
        self.publisher.publish(filtered_msg)

import subprocess, threading, os
def run_shell_command(command):
    with open(os.devnull, 'w') as nullfile:
        process = subprocess.Popen(command, shell=True, stdout=nullfile, stderr=subprocess.STDOUT)
        process.communicate()

def read_space_separated_matrix(string):
    """
    convert space separated matrix string to np matrix
    """
    lines = string.strip().split('\n')
    matrix = []
    for line in lines:
        values = line.split()  # Exclude the first element 'rotation_matrix'
        matrix.append([float(value) for value in values])
    numpy_matrix = np.array(matrix)
    return numpy_matrix

def main(args=None):
    with open('/home/trailbot/trail_ws/src/TRAILBot/human_detection/configs.yaml', 'r') as file:
        configs = yaml.safe_load(file)
    rclpy.init(args=args)
    node = HeightFilterNode(configs)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("\n\nDEBUG MODE ON\n\n")
    command = "ros2 bag play /home/trailbot/bags/Indoor2outdoor123/"

    thread = threading.Thread(target=run_shell_command, args=(command,))
    thread_main = threading.Thread(target=main)

    thread.start()
    thread_main.start()

    thread.join()
    thread_main.join()

     


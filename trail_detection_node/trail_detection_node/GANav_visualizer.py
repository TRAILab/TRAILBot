import torch
from .mmseg.apis import inference_segmentor, init_segmentor
from PIL import Image as ImagePIL
from torchvision import transforms
import cv2
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
from cv_bridge import CvBridge

'''
    The transformation matrix as well as the coordinate conversion and depth estimation functions are copied from human_detection_node
'''
camera_transformation_k = np.array([
    [628.5359544,0,676.9575694],
    [0,627.7249542,532.7206716],
    [0,0,1]])

rotation_matrix = np.array([
    [-0.007495781893,-0.0006277316155,0.9999717092],
    [-0.9999516401,-0.006361853422,-0.007499625104],
    [0.006366381192,-0.9999795662,-0.0005800141927]])

rotation_matrix = rotation_matrix.T

translation_vector = np.array([-0.06024059837, -0.08180891509, -0.3117851288])
image_width=1280
image_height=1024

def convert_to_lidar_frame(uv_coordinate):
    """
    convert 2d camera coordinate + depth into 3d lidar frame
    """
    point_cloud = np.empty( (3,) , dtype=float)
    point_cloud[2] = uv_coordinate[2]
    point_cloud[1] = ( image_height - uv_coordinate[1] )*point_cloud[2]
    point_cloud[0] = uv_coordinate[0]*point_cloud[2]

    inverse_camera_transformation_k = np.linalg.inv(camera_transformation_k)
    inverse_rotation_matrix = np.linalg.inv(rotation_matrix)
    point_cloud = inverse_camera_transformation_k @ point_cloud
    point_cloud = inverse_rotation_matrix @ (point_cloud-translation_vector) 
    return point_cloud

def convert_to_camera_frame(point_cloud):
    """
    convert 3d lidar data into 2d coordinate of the camera frame + depth
    """
    length = point_cloud.shape[0]
    translation = np.tile(translation_vector, (length, 1)).T
    
    point_cloud = point_cloud.T
    point_cloud = rotation_matrix@point_cloud + translation
    point_cloud = camera_transformation_k @ point_cloud

    uv_coordinate = np.empty_like(point_cloud)

    """
    uv = [x/z, y/z, z], and y is opposite so the minus imageheight
    """
    uv_coordinate[0] = point_cloud[0] / point_cloud[2]
    uv_coordinate[1] = image_height - point_cloud[1] / point_cloud[2]
    uv_coordinate[2] = point_cloud[2]

    uv_depth = uv_coordinate[2, :]
    filtered_uv_coordinate = uv_coordinate[:, uv_depth >= 0]
    return filtered_uv_coordinate

def estimate_depth(x, y, np_2d_array):
    """
    estimate the depth by finding points closest to x,y from thhe 2d array
    """
    # Calculate the distance between each point and the target coordinates (x, y)
    distances_sq = (np_2d_array[0,:] - x) ** 2 + (np_2d_array[1,:] - y) ** 2

    # Find the indices of the k nearest points
    k = 5     # Number of nearest neighbors we want
    closest_indices = np.argpartition(distances_sq, k)[:k]
    pixel_distance_threshold = 2000

    valid_indices = [idx for idx in closest_indices if distances_sq[idx]<=pixel_distance_threshold]
    if len(valid_indices) == 0:
        # lidar points disappears usually around 0.4m
        distance_where_lidar_stops_working = -1
        return distance_where_lidar_stops_working

    filtered_indices = np.array(valid_indices)
    # Get the depth value of the closest point
    closest_depths = np_2d_array[2,filtered_indices]

    return np.mean(closest_depths)

def load_model(device):

    config = "./trail_detection_node/trained_models/rugd_group6/ganav_rugd_6.py"
    checkpoint = "./trail_detection_node/trained_models/rugd_group6/ganav_rugd_6.pth"
    model = init_segmentor(config, checkpoint, device="cuda:0")
    print('Finished loading model!')

    return model

def find_route(model, device, cv_image):    
    result = inference_segmentor(model, cv_image)
    result_np = np.vstack(result).astype(np.uint8)
    pred = np.where(result_np == 0, 255, 0)

    # # prediction visualize
    # cv2.imshow('prediction',pred)
    # cv2.waitKey(25)

    route = np.zeros_like(pred)
    # calculate the center line by taking the average
    row_num = 0
    for row in pred:
        white_pixels = list(np.nonzero(row)[0])
        if white_pixels:
            average = (white_pixels[0] + white_pixels[-1]) / 2
            route[row_num][round(average)] = 255
        row_num = row_num + 1
    return route, pred

def equalize_hist_rgb(img):
    # Split the image into its color channels
    r, g, b = cv2.split(img)

    # Equalize the histograms for each channel
    r_eq = cv2.equalizeHist(r)
    g_eq = cv2.equalizeHist(g)
    b_eq = cv2.equalizeHist(b)

    # Merge the channels
    img_eq = cv2.merge((r_eq, g_eq, b_eq))
    # img_eq = cv2.fastNlMeansDenoisingColored(img_eq,None,10,20,7,42)
    # img_eq = cv2.GaussianBlur(img_eq, (25,25), 0)
    return img_eq

'''
    The traildetector node has two subscriptions(lidar and camera) and one publisher(trail position). After it receives msgs from both lidar and camera,
    it detects the trail in the image and sends the corresponding lidar position as the trail location.
    The msgs are synchornized before processing, using buffer and sync function.
    To find the path, the node will process the image, find a line to follow (by taking the average of left and right of the path), estimate the lidar points depth, 
    and choose to go to the closest point. 
    V1_traildetection assumes that the path is pointing frontward and has only one path in front.
'''
class trailDetector(Node):
    def __init__(self, model, device):
        super().__init__('trail_detector')
        # define trail subscription
        self.trail_publisher = self.create_publisher(
            PoseStamped,
            'trail_location',
            10)
        
        # define camera subscription
        self.camera_subscription = self.create_subscription(
            Image,
            'camera',
            self.camera_callback,
            10)
        self.camera_subscription
        
        # define lidar subscription
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            'velodyne_points',  
            self.lidar_callback,
            10)
        self.lidar_subscription

        self.bridge = CvBridge()

        # load model and device
        self.model = model
        self.device = device

        # buffers to hold msgs for sync
        self.buffer_size = 50   # 30 is a random choice, to be discussed 
        self.lidar_buffer = []
        self.camera_buffer = []
        self.only_camera_mode = True
    
    def camera_callback(self, msg):
        if len(self.camera_buffer) >= self.buffer_size:
            self.camera_buffer.pop(0)
        self.camera_buffer.append(msg)
        self.sync()

    def lidar_callback(self, msg):           
        if len(self.lidar_buffer) >= self.buffer_size:
            self.lidar_buffer.pop(0)
        self.lidar_buffer.append(msg)
        self.sync()

    def sync(self):
        # if one of the sensor has no msg, then return
        if self.only_camera_mode and self.camera_buffer:
            camera_msg = self.camera_buffer[0]
            self.camera_buffer.pop(0)
            self.only_camera_callback(camera_msg)
        elif not self.lidar_buffer or not self.camera_buffer:
            return
        
        while self.lidar_buffer and self.camera_buffer:
            lidar_msg = self.lidar_buffer[0]
            camera_msg = self.camera_buffer[0]

            time_tolerance = 20000000    # nanosec, random choice to be discussed
            time_difference = abs(lidar_msg.header.stamp.nanosec - camera_msg.header.stamp.nanosec)
            # print(time_difference)
            
            # compare the time difference, if it's within tolerance, then pass to the trail_callback, otherwise discard the older msg
            if time_difference <= time_tolerance:
                self.lidar_buffer.pop(0)
                self.camera_buffer.pop(0)
                self.get_logger().info("msgs received!")
                self.trail_callback(lidar_msg, camera_msg)
            elif lidar_msg.header.stamp.nanosec > camera_msg.header.stamp.nanosec:
                self.camera_buffer.pop(0)
            else:
                self.lidar_buffer.pop(0)

    def trail_callback(self, lidar_msg, camera_msg):
        # process lidar msg
        point_gen = pc2.read_points(
            lidar_msg, field_names=(
                "x", "y", "z"), skip_nans=True)
        points = [[x, y, z] for x, y, z in point_gen]
        points = np.array(points)
        points2d = convert_to_camera_frame(points)

        # process camera msg
        cv_image = self.bridge.imgmsg_to_cv2(camera_msg, desired_encoding='passthrough')
        
        # increase brightness
        M = np.ones(cv_image.shape, dtype = 'uint8') * 50 # increase brightness by 50 
        cv_image = cv2.add(cv_image, M)

        # # image visualizer
        # cv2.imshow('raw image',cv_image)
        # cv2.waitKey(25)

        route = find_route(self.model, self.device, cv_image)

        # # visualize route purely
        # cv2.imshow("white_route",route)
        # cv2.waitKey(25)

        route_indices = list(zip(*np.nonzero(route)))
        if not route_indices:
            print("No centerline found!")
            return
        
        # #filter points that have no lidar points near it
        # filtered_route_indices = []
        # for index in route_indices:
        #     point = []
        #     point.append(index[0])
        #     point.append(index[1])
        #     point.append(estimate_depth(index[0], index[1], points2d))
        #     if point[2] == -1:
        #         continue
        #     else:
        #         filtered_route_indices.append(point)

        # find the corresponding lidar points using the center line pixels
        filtered_3dPoints = []
        # for index in filtered_route_indices:
        for index in route_indices:
            point = []
            # point.append(index[0])
            # point.append(index[1])
            # point.append(index[2])

            point.append(index[0])
            point.append(index[1])
            point.append(estimate_depth(index[0], index[1], points2d))

            point_3d = convert_to_lidar_frame(point)
            filtered_3dPoints.append(point_3d)

        filtered_3dPoints = np.array(filtered_3dPoints)
        # find the nearest 3d point and set that as goal
        distances_sq = filtered_3dPoints[:,0]**2 + filtered_3dPoints[:,1]**2 + filtered_3dPoints[:,2]**2
        smallest_index = np.argmin(distances_sq)
        # print(math.sqrt(distances_sq[smallest_index]))
        # smallest_index = np.argmin(filtered_3dPoints[:,2])

        # visualize after-pocessed image
        visualize_cv_image = cv_image
        print(f"{visualize_cv_image.shape[0]}")
        circle_x = route_indices[smallest_index][0]
        circle_y = route_indices[smallest_index][1]
        
        # # visualize the lidar points in image
        # uv_x, uv_y, uv_z = points2d
        # for index_lidar in range(points2d.shape[1]):
        #     # print(f"{int(uv_x[index_lidar])},{int(uv_y[index_lidar])}")
        #     cv2.circle(visualize_cv_image, (int(uv_x[index_lidar]), int(image_height - uv_y[index_lidar])), radius=5, color=(255, 0, 0), thickness=-1)
        
        # visualize center line in image
        for index_circle in range(len(route_indices)):
            if index_circle == smallest_index:
                continue
            else:
                red_circle_x = route_indices[index_circle][0]
                red_circle_y = route_indices[index_circle][1]
                cv2.circle(visualize_cv_image, (red_circle_y, red_circle_x), radius=5, color=(0, 0, 255), thickness=-1)
        
        # visualize the chosen point in image
        cv2.circle(visualize_cv_image, (circle_y, circle_x), radius=7, color=(0, 255, 0), thickness=-1)
        cv2.circle(visualize_cv_image, (0, 0), radius=12, color=(0, 255, 0), thickness=-1)
        
        cv2.imshow('circled image',visualize_cv_image)
        cv2.waitKey(25)

        # publsih message
        trail_location_msg = PoseStamped()
        trail_location_msg.header.stamp = lidar_msg.header.stamp
        trail_location_msg.header.frame_id = "velodyne"
        #position
        trail_location_msg.pose.position.x = filtered_3dPoints[smallest_index][0]  
        trail_location_msg.pose.position.y = filtered_3dPoints[smallest_index][1]
        trail_location_msg.pose.position.z = filtered_3dPoints[smallest_index][2]
        #orientation
        yaw = math.atan2(filtered_3dPoints[smallest_index][1], filtered_3dPoints[smallest_index][0])
        trail_location_msg.pose.orientation.x = 0.0  
        trail_location_msg.pose.orientation.y = 0.0 
        trail_location_msg.pose.orientation.z = math.sin(yaw/2)
        trail_location_msg.pose.orientation.w = math.cos(yaw / 2)
        self.get_logger().info("location published!")
        self.trail_publisher.publish(trail_location_msg)
        
    def only_camera_callback(self, camera_msg):
        # process camera msg
        cv_image = self.bridge.imgmsg_to_cv2(camera_msg, desired_encoding='passthrough')

        # cv_image = cv2.fastNlMeansDenoisingColored(cv_image,None,5,10,7,14)
        
        # # histogram equalization method 1
        # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2YCrCb)
        # cv_image[:,:,0] = cv2.equalizeHist(cv_image[:,:,0])
        # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_YCrCb2BGR)

        # # histogram equalization method 2
        # cv_image = equalize_hist_rgb(cv_image)

        # # histogram equalization method 3 CLAHE
        # image_bw = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # clahe = cv2.createCLAHE(clipLimit=5)
        # cv_image = clahe.apply(image_bw) + 30

        # # histogram equalization method 4 CLAHE
        # cla = cv2.createCLAHE(clipLimit=4.0)
        # H, S, V = cv2.split(cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV))
        # eq_V = cla.apply(V)
        # cv_image = cv2.cvtColor(cv2.merge([H, S, eq_V]), cv2.COLOR_HSV2BGR)

        # # increase brightness
        # M = np.ones(cv_image.shape, dtype = 'uint8') * 50 # increase brightness by 50 
        # cv_image = cv2.add(cv_image, M)

        # # image visualizer
        # cv2.imshow('raw image',cv_image)
        # cv2.waitKey(25)

        route, pred = find_route(self.model, self.device, cv_image)

        # # visualize route purely
        # cv2.imshow("white_route",route)
        # cv2.waitKey(25)

        # increase the brightness of image where is predicted as road
        sign = cv2.cvtColor(pred, cv2.COLOR_GRAY2BGR)
        cv_image = cv2.add(cv_image, sign)

        route_indices = list(zip(*np.nonzero(route)))
        if not route_indices:
            print("No centerline found!")
            return
        
        for index_circle in range(len(route_indices)):
            red_circle_x = route_indices[index_circle][0]
            red_circle_y = route_indices[index_circle][1]
            cv2.circle(cv_image, (red_circle_y, red_circle_x), radius=5, color=(0, 0, 255), thickness=-1)

        cv2.imshow('circled image',cv_image)
        cv2.waitKey(25)
        



def main(args=None):
    print(torch.cuda.is_available())
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = load_model(device)
    
    rclpy.init(args=args)
    trailDetectorNode = trailDetector(model, device)
    rclpy.spin(trailDetectorNode)

    trailDetectorNode.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
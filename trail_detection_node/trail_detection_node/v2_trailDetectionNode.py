import torch
from .model_loader import LEDNet
from PIL import Image as ImagePIL
from torchvision import transforms
import cv2

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import math
from cv_bridge import CvBridge
import os

import message_filters

''' TODO:
    The transformation matrix as well as the coordinate conversion and depth estimation 
    functions to be transferred to human_detection_node
'''
ONLY_CAMERA_MODE = False # Only visualize path without publishing target pose
VISUALIZE = True # Enable the cv2 visuals of pipeline
CAM_INTRINSIC_K = np.array([
                                    [1104.0, 0     , 615.34],
                                    [0     , 1103.9, 310.33],
                                    [0     , 0     , 1     ]
                                ])

#Transformation of points from lidar to cam frame
T_CL = np.array([
                                [0.99983  , 0.012464 , 0.013538 , -0.023072],
                                [0.014029 , -0.040245, -0.99909 , -0.10742 ],
                                [-0.011908, 0.99911  , -0.040413, -0.14859 ],
                                [0        , 0        , 0        , 1        ]
                            ])

# RADIAL_DISTORTION = np.array([0.2290977399, 1.277538781, 0, 0])
RADIAL_DISTORTION = np.array([0.0, 0.0, 0, 0])

""" TODO:
Tunable parameters: 
    - init
    - k_size and brightness in pre_process_img fxn (pre-blur and brightness)
    - blur_kernel_size and min_contour_area in post_processing fxn
    - poly_degree in compute_best_path
    - min_area_threshold in remove_small_black_regions
    - dist_thresh_uv, min_depth and num_max_points_to_match in filter_usable_route_points

Description:
    The traildetector node has two subscriptions(lidar and camera) and one publisher(trail position). After it receives msgs from both lidar and camera,
    it detects the trail in the image and sends the corresponding lidar position as the trail location.
    The msgs are synchornized before processing, using buffer and sync function.
    To find the path, the node will process the image, fit a line to follow, estimate the lidar points depth, 
    and choose to go to a valid point along the centreline. 
    Assumes that the path is pointing frontward and has only one path in front.
"""

# TODO: Add proper documentation and optimize speed

class trailDetector(Node):
    def __init__(self, only_camera_mode, visualize,
                 pub_queue_size=10, sync_queue_size=30, cam_sub_queue_size=10, max_time_diff=0.5):
        
        super().__init__('trail_detector')
        
        self.only_camera_mode = only_camera_mode
        self.visualize = visualize

        self.image_width = None
        self.image_height = None
        self.c_x = CAM_INTRINSIC_K[0, 2]
        self.c_y = CAM_INTRINSIC_K[1, 2]
        self.f_x = CAM_INTRINSIC_K[0, 0]
        self.f_y = CAM_INTRINSIC_K[1, 1]

        #CvBridge
        self.bridge = CvBridge()

        # load model and device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self.load_model()

        # define trail publisher
        self.trail_publisher = self.create_publisher(
            PoseStamped,
            'trail_location',
            pub_queue_size)

        if self.only_camera_mode:
            # define camera subscription
            print("Mode: Only Camera")
            self.camera_subscription = self.create_subscription(
                Image,
                'camera',
                self.only_camera_callback,
                cam_sub_queue_size)
            self.camera_subscription
        else:
            # create subscribers and synchronizer
            print("Mode: Lidar and Camera")
            # create subscribers
            self.image_sub = message_filters.Subscriber(self, Image, 'camera')
            self.lidar_sub = message_filters.Subscriber(self, PointCloud2, 'ouster/points')

            # create callback
            ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub], sync_queue_size, max_time_diff)
            ts.registerCallback(self.trail_callback)

    #-----SEMANTIC SEGMENTATION MODEL-------------------------------------------------------
            
    def load_model(self):
        model = LEDNet(nclass=2, backbone='resnet50', pretrained_base=True)
        model_location = 'lednet_resnet50_trails_best_model.pth'
        full_path = os.path.expanduser(f'~/.torch/models/{model_location}')
        if os.path.isfile(full_path):
            try:
                model.load_state_dict(torch.load(full_path, map_location=self.device))
            except Exception as e:
                print(f"Error loading model: {e}")
                return None            
        else:
            print("Model file not found, ensure it exists as: ", full_path)
            return None
        model = model.to(self.device)
        model.eval()
        print('Finished loading model')

        return model

    #-----CAMERA/FIND ROUTE METHODS---------------------------------------------------------
    def convert_indices2uv(self, route_indices):
            uv_route = np.empty_like(route_indices).astype(float) #N, 2 
            uv_route[:, 0] = (route_indices[:, 0] - self.c_x) / self.f_x
            uv_route[:, 1] = (route_indices[:, 1] - self.c_y) / self.f_y
            return uv_route
    
    def compute_route_uv_and_img(self, camera_msg):
        def get_rgb_undistorted_img(camera_msg):
            # process camera msg
            cv_image = self.bridge.imgmsg_to_cv2(camera_msg, desired_encoding='passthrough')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Undistort image
            if not self.image_height:
                self.image_height, self.image_width = cv_image.shape[:2]
            new_K, _ = cv2.getOptimalNewCameraMatrix(CAM_INTRINSIC_K, RADIAL_DISTORTION, (self.image_width, self.image_height), 1, (self.image_width, self.image_height))  
            cv_image = cv2.undistort(cv_image, CAM_INTRINSIC_K, RADIAL_DISTORTION, None, new_K)
            return cv_image

        def pre_process_img(image, k_size=23, brightness=10):
            
            # Split the image into its color channels
            r, g, b = cv2.split(image)

            # Equalize the histograms for each channel
            r_eq = cv2.equalizeHist(r)
            g_eq = cv2.equalizeHist(g)
            b_eq = cv2.equalizeHist(b)

            # Merge the channels
            image = cv2.merge((r_eq, g_eq, b_eq))

            # Apply Gaussian Blur
            image = cv2.GaussianBlur(image, (k_size, k_size), 0)

            # # Increase Brightness
            M = np.ones(image.shape, dtype='uint8') * brightness  # Increase brightness by 50 
            image = cv2.add(image, M)

            return image
        
        def model_output(model, device, cv_image):
            PIL_image = ImagePIL.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            transform = transforms.Compose([
                transforms.ToTensor(),
                transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ])
            image = transform(PIL_image).unsqueeze(0).to(device)
            with torch.no_grad():
                output = model(image)
            # pred is prediction generated by the model, route is the variable for the center line
            model_pred = torch.argmax(output[0], 1).squeeze(0).cpu().data.numpy()
            model_pred[model_pred == 0] = 0
            model_pred[model_pred == 1] = 255
            model_pred = np.array(model_pred, dtype=np.uint8)
            return model_pred

        def remove_small_black_regions(mask, min_area_threshold=50000):
            inverted_mask = ~mask
    
            # Find contours of the inverted mask
            contours, _ = cv2.findContours(inverted_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Iterate through contours and add small black areas to the original mask
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area_threshold:
                    cv2.drawContours(mask, [contour], -1, (255), cv2.FILLED)

            return mask
        
        def post_processing(model_pred, blur_kernel_size=31, min_contour_area = 150000):
            orig_contours, _ = cv2.findContours(model_pred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            mask = np.zeros_like(model_pred)
            post_blur_mask = np.zeros_like(model_pred)
            cv2.drawContours(mask, orig_contours, -1, (255), cv2.FILLED)

            
            # Apply Gaussian blur to the mask
            mask = cv2.GaussianBlur(mask, (blur_kernel_size, blur_kernel_size), 0)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Don't include small white regions as trail
            for contour in contours:
                # Calculate the area of each contour
                area = cv2.contourArea(contour)
                
                # Draw contours above a certain size
                if area > min_contour_area:
                    cv2.drawContours(post_blur_mask, [contour], -1, (255), cv2.FILLED)

            # Show the contour image
            post_blur_mask = remove_small_black_regions(post_blur_mask)
            return post_blur_mask
        
        def compute_best_path(model_pred, poly_degree = 2):
            x_idx = []
            y_idx = []
            pixel_route = None
        
            # calculate the center line by taking the average
            for i, row in enumerate(model_pred):
                white_pixels = list(np.nonzero(row)[0])
                if white_pixels:
                    average = (white_pixels[0] + white_pixels[-1]) // 2
                    x_idx.append(average)
                    y_idx.append(i)
            if len(x_idx) > 0:
                x_idx = np.array(x_idx)
                y_idx = np.array(y_idx)

                coefficients = np.polyfit(y_idx, x_idx, deg=poly_degree)  # Third-degree polynomial (adjust degree as needed)
                x_new_idx = np.polyval(coefficients, y_idx).astype(int)
                x_new_idx = np.clip(x_new_idx, a_min=0, a_max=self.image_width-1)

                # route_image = np.zeros_like(model_pred)
                pixel_route = np.hstack((x_new_idx[:, np.newaxis], y_idx[:, np.newaxis]))
            return pixel_route

        def find_route(model, device, cv_image):
            model_pred = model_output(model, device, cv_image)
            model_pred = post_processing(model_pred)
            pixel_route = compute_best_path(model_pred)

            return model_pred, pixel_route  #array of size of the original undistorted image where the centreline is of value 255, rest is 0 in camera frame

        

        wait_time_max = 1
        cv_image = get_rgb_undistorted_img(camera_msg)
        
        undistorted_image = cv_image.copy()
        if self.visualize:
            cv2.imshow('Undistorted', cv_image)
            cv2.waitKey(wait_time_max)

        #Pre process image to prepare for segmentation model
        cv_image = pre_process_img(cv_image)
        # if self.visualize:
        #     cv2.imshow('pre-processed', cv_image)
        #     cv2.waitKey(wait_time_max)

        model_pred, pixel_route = find_route(self.model, self.device, cv_image)
        uv_route = None
        if self.visualize:
            #To visualize prediction
                        
            # cv2.imshow('segmentation_ouput',model_pred)
            # cv2.waitKey(wait_time_max)

            # # highlight red where is predicted as road
            sign = cv2.cvtColor(model_pred, cv2.COLOR_GRAY2RGB) /255 * 200
            sign = sign.astype(undistorted_image.dtype)  # Convert sign to the data type of undistorted_image
            sign[:, :, :2] = 0
            cv_image = cv2.add(undistorted_image, sign)
            # cv2.imshow('highlighted_route', cv_image)
            # cv2.waitKey(wait_time_max)
            if isinstance(pixel_route, np.ndarray):
                for centre_dot in pixel_route:
                    cv2.circle(cv_image, (centre_dot[0], centre_dot[1]), radius=5, color=(255, 0, 0), thickness=-1)
                # cv2.imshow('final_path', cv_image)
                # cv2.waitKey(wait_time_max)

        if isinstance(pixel_route, np.ndarray): uv_route = self.convert_indices2uv(pixel_route)
        return uv_route, cv_image
    
    #-----POINTCLOUD TRANSFORMATION AND FILTERING-------------------------------------------
    
    def lidar_pts_cam_frame(self, points3d):
        """
        convert 3d lidar data into 2d coordinate of the camera frame + depth
        points3d is 2d array of size (N, 4)   (x, y, z, 1)
        """
        N = points3d.shape[0]
        points3d = points3d.T #(4, N)
        points3d_cam_frame = T_CL @ points3d # (4, N)

        uv_coordinate = np.zeros((3, N)) # (3, N)

        uv_coordinate[0] = points3d_cam_frame[0] / points3d_cam_frame[2] #x / z
        uv_coordinate[1] = points3d_cam_frame[1] / points3d_cam_frame[2] #y / z
        uv_coordinate[2] = points3d_cam_frame[2] # z

        return uv_coordinate.T #(N, 3) u, v, depth
    
    def filter_usable_route_points(self, uv_route, points2d, min_depth=5, num_max_points_to_match=400, dist_thresh_uv=0.03):
        # points2d: (N, 3)
        # uv_route (M, 2)
        # original points are length N

        N = points2d.shape[0]
        row_indices = np.arange(N)[:, np.newaxis]
        points2d = np.hstack((points2d, row_indices)) #u, v, depth, idx
        filtered_points2d = points2d[points2d[:, 2] >= 0]

        if len(uv_route) < num_max_points_to_match: uv_route = uv_route[::-1]
        else: uv_route = uv_route[-num_max_points_to_match:][::-1]

        
        best_depth = float('inf')
        lidar_point_uv = np.empty(4)

        target_pcl_index = -1
        for i, uv_point in enumerate(uv_route):
            euclid_distances = np.linalg.norm(filtered_points2d[:, :2] - uv_point, axis=1)
            closest_filtered_idx = np.argmin(euclid_distances)
            dist = euclid_distances[closest_filtered_idx]

            if dist < dist_thresh_uv: #lidar point close to path
                lidar_point_uv = filtered_points2d[closest_filtered_idx]
                depth = lidar_point_uv[2]
                if min_depth <= depth < best_depth:
                    target_pcl_index = int(lidar_point_uv[3])
                    best_depth = depth
        return target_pcl_index, lidar_point_uv[:2]
    
    def uv2pixel(self, target_uv, image):
        if not self.visualize: return
        u, v = target_uv
        pix_x = u * self.f_x + self.c_x
        pix_y = v * self.f_y + self.c_y
        pix_x = np.clip(pix_x, 0, self.image_width - 1).astype(int)
        pix_y = np.clip(pix_y, 0, self.image_height - 1).astype(int)
        image = cv2.circle(image, (pix_x, pix_y), 10, (0, 255, 0), thickness=3)
        cv2.imshow("Target point", image)
        cv2.waitKey(1)

    #-----CREATE PUBLISH MESSAGE------------------------------------------------------------

    def publish_trail_target_point(self, lidar_msg, target_point):
        x, y, z = target_point

        # publish message
        trail_location_msg = PoseStamped()
        trail_location_msg.header.stamp = lidar_msg.header.stamp
        trail_location_msg.header.frame_id = "os_lidar"
        
        # position
        trail_location_msg.pose.position.x = x
        trail_location_msg.pose.position.y = y
        trail_location_msg.pose.position.z = z

        # orientation
        yaw = math.atan2(y, x)
        trail_location_msg.pose.orientation.x = 0.0  
        trail_location_msg.pose.orientation.y = 0.0 
        trail_location_msg.pose.orientation.z = math.sin(yaw/2)
        trail_location_msg.pose.orientation.w = math.cos(yaw / 2)
        self.trail_publisher.publish(trail_location_msg)
        
        # logging
        self.get_logger().info(f"Location published as Point: {x}, {y}, {z}")
        return
    
    #-----CALLBACK FUNCTIONS----------------------------------------------------------------

    def trail_callback(self, camera_msg, lidar_msg):
        print("Camera and Lidar Message received!")

        # process camera msg to retrieve path
        uv_route, cv_image = self.compute_route_uv_and_img(camera_msg)
        if not isinstance(uv_route, np.ndarray):
            print("No centerline found!")
            return

        # process lidar msg
        point_gen = pc2.read_points(
            lidar_msg, field_names=(
                "x", "y", "z"), skip_nans=True)
        points3d = np.array([[x, y, z, 1] for x, y, z in point_gen])
        # points3d = np.array(points3d)
        
        #Convert pointcloud to camera uv coordinates
        points2d = self.lidar_pts_cam_frame(points3d)

        target_pcl_index,  target_uv = self.filter_usable_route_points(uv_route, points2d)
        if target_pcl_index == -1:
            print("No usable centerline found!")
            return
        
        self.uv2pixel(target_uv, cv_image)
        target_point = points3d[target_pcl_index][:3] #array of [x, y, z]

        self.publish_trail_target_point(lidar_msg, target_point)
        return

    def only_camera_callback(self, camera_msg):
        print("Camera Message received!")
        uv_route, _ = self.compute_route_uv_and_img(camera_msg)
        if not isinstance(uv_route, np.ndarray):
            print("No centerline found!")
        return

#-----MAIN----------------------------------------------------------------------------------

def main(args=None):
    print("cuda is available: ", torch.cuda.is_available())
    
    rclpy.init(args=args)
    trailDetectorNode = trailDetector(ONLY_CAMERA_MODE, VISUALIZE)
    rclpy.spin(trailDetectorNode)

    trailDetectorNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

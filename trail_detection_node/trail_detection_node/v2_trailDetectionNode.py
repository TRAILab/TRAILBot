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

import tf2_ros

''' TODO:
    The transformation matrix as well as the coordinate conversion and depth estimation 
    functions to be transferred to human_detection_node
'''
ONLY_CAMERA_MODE = False # Only visualize path without publishing target pose
VISUALIZE = True # Enable the cv2 visuals of pipeline
TARGET_DISTANCE = 6.0 #Distance away along trail for target point to navigate towards on trail
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

"""
Description:
    The traildetector node has two subscriptions(lidar and camera) and one publisher(trail position). After it receives msgs from both lidar and camera,
    it detects the trail in the image and sends the corresponding lidar position as the trail location.
    The msgs are synchornized before processing, using buffer and sync function.
    To find the path, the node will process the image, fit a line to follow, estimate the lidar points depth, 
    and choose to go to a valid point along the centreline. 
    Assumes that the path is pointing frontward and has only one path in front.
"""

# TODO: optimize speed and performance, check double contour in post, tune params

class trailDetector(Node):
    def __init__(self, only_camera_mode: bool, visualize: bool,
                 pre_proc_blur_k_size: int = 11, brightness: int = 20,
                 post_proc_blur_k_size: int = 31, min_contour_area: int = 50000,
                 poly_degree: int = 2, min_black_area_threshold: int = 50000,
                 min_depth: float = TARGET_DISTANCE, num_max_points_to_match: int = 400, dist_thresh_uv: float = 0.03,
                 pub_queue_size: int = 10, sync_queue_size: int = 30, 
                 cam_sub_queue_size: int = 10, max_time_diff: float = 0.5) -> None:
        
        """
        Initializes the trail detector node.

        Args:
            only_camera_mode (bool): Flag indicating whether to operate in camera-only mode.
            visualize (bool): Flag indicating whether to enable visualization.
            pre_proc_blur_k_size (int): Kernel size for pre-processing Gaussian blur.
            brightness (int): Brightness adjustment value for pre-processing.
            post_proc_blur_k_size (int): Kernel size for post-processing Gaussian blur.
            min_contour_area (int): Minimum contour area threshold for post-processing.
            poly_degree (int): Degree of polynomial for fitting the path.
            min_black_area_threshold (int): Minimum area threshold for removing small black regions.
            min_depth (float): Minimum depth threshold for filtering usable route points.
            num_max_points_to_match (int): Maximum number of points to match for filtering usable route points.
            dist_thresh_uv (float): Distance threshold for UV route points.
            pub_queue_size (int): Queue size for the trail publisher.
            sync_queue_size (int): Queue size for message synchronization.
            cam_sub_queue_size (int): Queue size for camera subscription.
            max_time_diff (float): Maximum time difference for message synchronization.
        """

        super().__init__('trail_detector')
        
        # Set visualization mode
        self.only_camera_mode = only_camera_mode
        self.visualize = visualize

        # Tunable parameters
        self.pre_proc_blur_k_size = pre_proc_blur_k_size
        self.brightness = brightness
        self.post_proc_blur_k_size = post_proc_blur_k_size
        self.min_contour_area = min_contour_area
        self.poly_degree = poly_degree
        self.min_black_area_threshold = min_black_area_threshold
        self.min_depth = min_depth
        self.num_max_points_to_match = num_max_points_to_match
        self.dist_thresh_uv = dist_thresh_uv

        # Camera properties
        self.image_width = None
        self.image_height = None
        self.c_x = CAM_INTRINSIC_K[0, 2]
        self.c_y = CAM_INTRINSIC_K[1, 2]
        self.f_x = CAM_INTRINSIC_K[0, 0]
        self.f_y = CAM_INTRINSIC_K[1, 1]

        #CvBridge
        self.bridge = CvBridge()

        # Load model and device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = self.load_model()

        # Define trail publisher
        self.trail_publisher = self.create_publisher(
            PoseStamped,
            'trail_location',
            pub_queue_size)
        
        # self.ground_points_publisher = self.create_publisher(
        #     PointCloud2,
        #     'ground_points',
        #     10)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        if self.only_camera_mode:
            # Define camera subscription
            print("Mode: Only Camera")
            self.camera_subscription = self.create_subscription(
                Image,
                'camera',
                self.only_camera_callback,
                cam_sub_queue_size)
            self.camera_subscription
        else:
            # Create subscribers and synchronizer
            print("Mode: Lidar and Camera")
            # create subscribers
            self.image_sub = message_filters.Subscriber(self, Image, 'camera')
            self.lidar_sub = message_filters.Subscriber(self, PointCloud2, 'ouster/points')
            # self.lidar_sub = message_filters.Subscriber(self, PointCloud2, 'ground')

            # create callback
            ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub], sync_queue_size, max_time_diff)
            ts.registerCallback(self.trail_callback)

    #-----SEMANTIC SEGMENTATION MODEL-------------------------------------------------------
            
    def load_model(self) -> LEDNet:
        """
        Loads the LEDNet model for semantic segmentation.

        Returns:
            LEDNet: Loaded LEDNet model trained on trail segmentation data.
        """

        # Initialize semantic segmentation model and load dictionary
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
    def convert_pix2uv(self, route_indices: np.ndarray) -> np.ndarray:
        """
        Converts pixels (image indices) to UV coordinates.

        Args:
            route_indices (numpy.ndarray): Array of route pixels.
                Note: first col indicates pixel along width of image, and second col indicates along height.

        Returns:
            numpy.ndarray: UV coordinates of the route.
        """

        # Use camera properties to convert pixels to uv
        uv_route = np.empty_like(route_indices).astype(float) #N, 2 
        uv_route[:, 0] = (route_indices[:, 0] - self.c_x) / self.f_x
        uv_route[:, 1] = (route_indices[:, 1] - self.c_y) / self.f_y
        return uv_route
    
    def compute_route_uv_and_img(self, camera_msg: Image) -> tuple[np.ndarray, np.ndarray]:
        """
        Computes the planned route in uv coordinates and produces the image for visualization.

        Args:
            camera_msg (sensor_msgs.msg.Image): Camera message.

        Returns:
            tuple: Tuple containing route in uv coordinates(numpy.ndarray) and image with trail and centreline (numpy.ndarray).
        """

        def get_rgb_undistorted_img(camera_msg: Image) -> np.ndarray:
            """
            Processes the camera message to retrieve the undistorted RGB image.

            Args:
                camera_msg (sensor_msgs.msg.Image): Camera message.

            Returns:
                numpy.ndarray: Undistorted RGB image.
            """
            # process camera msg and set to RGB
            cv_image = self.bridge.imgmsg_to_cv2(camera_msg, desired_encoding='passthrough')
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Undistort image
            if not self.image_height:
                self.image_height, self.image_width = cv_image.shape[:2]
            new_K, _ = cv2.getOptimalNewCameraMatrix(CAM_INTRINSIC_K, RADIAL_DISTORTION, (self.image_width, self.image_height), 1, (self.image_width, self.image_height))  
            cv_image = cv2.undistort(cv_image, CAM_INTRINSIC_K, RADIAL_DISTORTION, None, new_K)
            return cv_image

        def pre_process_img(image: np.ndarray, pre_proc_blur_k_size: int, brightness: int) -> np.ndarray:
            """
            Pre-processes the image for segmentation model.
            Applies equalization, Gaussian Blur, and brightness increase.

            Args:
                image (numpy.ndarray): RGB undistorted input image.
                pre_proc_blur_k_size (int): Kernel size for Gaussian blur.
                brightness (int): Brightness adjustment value.

            Returns:
                numpy.ndarray: Pre-processed image.
            """
            # Does not seem like pre-processing is necessary in test runs to date
            equalize = False
            pre_blur = False
            increase_brightness = False

            if equalize:
                # Split the image into its color channels
                r, g, b = cv2.split(image)

                # Equalize the histograms for each channel
                r_eq = cv2.equalizeHist(r)
                g_eq = cv2.equalizeHist(g)
                b_eq = cv2.equalizeHist(b)

                # Merge the channels
                image = cv2.merge((r_eq, g_eq, b_eq))
            if pre_blur:
                # Apply Gaussian Blur
                image = cv2.GaussianBlur(image, (pre_proc_blur_k_size, pre_proc_blur_k_size), 0)

            if increase_brightness:
                # # Increase Brightness
                M = np.ones(image.shape, dtype='uint8') * brightness  # Increase brightness by 50 
                image = cv2.add(image, M)

            return image
        def show_model_logits_view(trail_layer, background_layer, cv_image):
            trail_layer = np.array(trail_layer, dtype=np.uint8)
            background_layer = np.array(background_layer, dtype=np.uint8)
            print("trail size: ", trail_layer.shape)
            trail_sign = cv2.cvtColor(trail_layer, cv2.COLOR_GRAY2RGB) /255 * 200
            trail_sign = trail_sign.astype(cv_image.dtype)  # Convert trail_sign to the data type of cv_image
            trail_sign[:, :, :2] = 0
            cv_image_trail = cv2.add(cv_image, trail_sign)
            # cv2.imshow('trail_heatmap', trail_layer)
            # cv2.waitKey(wait_time_max)

            background_sign = cv2.cvtColor(background_layer, cv2.COLOR_GRAY2RGB) /255 * 200
            background_sign = background_sign.astype(cv_image.dtype)  # Convert background_sign to the data type of cv_image
            background_sign[:, :, 1:] = 0
            cv_image_back = cv2.add(cv_image, background_sign)
            # cv2.imshow('background_heatmap', background_layer)
            # cv2.waitKey(wait_time_max)




            difference = abs(trail_layer - background_layer)
            # cv2.imshow('areas with similar class probabilites', difference)
            # cv2.waitKey(wait_time_max)
            return



        def model_output(model: LEDNet, device: torch.device, cv_image: np.ndarray) -> np.ndarray:
            """
            Obtains model output for the given image.

            Args:
                model (LEDNet): Semantic segmentation model.
                device (torch.device): Device to run the model.
                cv_image (numpy.ndarray): Input image.

            Returns:
                numpy.ndarray: Model output prediction mask.
            """

            # Prepare image to be model input format
            PIL_image = ImagePIL.fromarray(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
            transform = transforms.Compose([
                transforms.ToTensor(),
                transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225]),
            ])
            image = transform(PIL_image).unsqueeze(0).to(device)

            # Output results from model
            with torch.no_grad():
                output = model(image)
            print("Output of segmentation model", output)
            print("\n\ntype of output: ", type(output[0]))
            print("\n\nshape of output: ", output[0].shape)
            print("\n\noutput[0] min and max", output[0].min(), output[0].max())
            
            
            # Pred is prediction mask generated by the model, white == trail
            out_min = output[0].min()
            out_max = output[0].max()
            bias_perc = 0.15 # % to increase by
            background_bias = (out_max - out_min) * bias_perc #amount to increase background layer by
            output[0][0, 0, :, :] += background_bias
            print("background bias: ", background_bias)
            print("new output[0] min and max", output[0].min(), output[0].max())
            normalized_ouput = (((output[0].squeeze(0) - out_min) / (out_max - out_min)).cpu().data.numpy() * 255).astype(int)
            background_layer = normalized_ouput[0]
            trail_layer = normalized_ouput[1]

            model_pred = torch.argmax(output[0], 1).squeeze(0).cpu().data.numpy()
            model_pred[model_pred == 0] = 0
            model_pred[model_pred == 1] = 255
            model_pred = np.array(model_pred, dtype=np.uint8)
            # return model_pred
            print("model_pred size: ", model_pred.shape)

            show_model_logits_view(trail_layer, background_layer, cv_image)
            return model_pred
        

        def remove_small_black_regions(mask: np.ndarray, min_black_area_threshold: int) -> np.ndarray:
            """
            Removes small black regions from the mask.

            Args:
                mask (numpy.ndarray): Input mask.
                min_black_area_threshold (int): Minimum pixel area threshold for regions to be retained.

            Returns:
                numpy.ndarray: Mask with small black regions removed.
            """

            #Set non-trail area to be white
            inverted_mask = ~mask
    
            # Find contours of the inverted mask
            contours, _ = cv2.findContours(inverted_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Iterate through contours and add small non-trail areas to the original mask
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_black_area_threshold:
                    cv2.drawContours(mask, [contour], -1, (255), cv2.FILLED)

            return mask
        
        def post_processing(model_pred: np.ndarray, post_proc_blur_k_size: int, min_contour_area: int,
                            min_black_area_threshold: int) -> np.ndarray:
            """
            Performs post-processing on the model prediction.
            Find the contours of the image, blur the contours, and find the smoothened contours.
            TODO: Check if the first contour check is necessary.
            Only maintain contours above a threshold size.

            Args:
                model_pred (numpy.ndarray): Model prediction.
                post_proc_blur_k_size (int): Kernel size for Gaussian blur.
                min_contour_area (int): Minimum contour area threshold.

            Returns:
                numpy.ndarray: Processed model prediction.
            """
            apply_post_blur = False
            remove_small_white_noise = True
            remove_small_black_blobs = True

            contours, _ = cv2.findContours(model_pred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            mask = np.zeros_like(model_pred)

            if apply_post_blur:               
                pre_blur_mask = np.zeros_like(model_pred)
                cv2.drawContours(pre_blur_mask, contours, -1, (255), cv2.FILLED)
                # Apply Gaussian blur to the pre_blur_mask
                pre_blur_mask = cv2.GaussianBlur(pre_blur_mask, (post_proc_blur_k_size, post_proc_blur_k_size), 0)
                contours, _ = cv2.findContours(pre_blur_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if remove_small_white_noise:
                # Don't include small white regions as trail
                for contour in contours:
                    # Calculate the area of each contour
                    area = cv2.contourArea(contour)
                    
                    # Draw contours above a certain size
                    if area > min_contour_area:
                        cv2.drawContours(mask, [contour], -1, (255), cv2.FILLED)
            else:
                mask = model_pred

            if remove_small_black_blobs:
                # Show the contour image
                mask = remove_small_black_regions(mask, min_black_area_threshold)
                

            return mask
        
        def compute_centreline_path(model_pred: np.ndarray, poly_degree: int) -> np.ndarray:
            """
            Computes the polynomial fit centreline on the model prediction.

            Args:
                model_pred (numpy.ndarray): Model prediction.
                poly_degree (int): Degree of polynomial for fitting the path.

            Returns:
                numpy.ndarray: centreline path pixel.
                    Note: x_idx represents pixel along width of image
                          y_idx represents pixel along height of image
            """
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

        def find_route(model: LEDNet, device: torch.device, cv_image: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
            """
            Finds the pixel route in the image using the semantic segmentation model.

            Args:
                model (LEDNet): Semantic segmentation model.
                device (torch.device): Device to run the model.
                cv_image (numpy.ndarray): Input image after pre-processing.

            Returns:
                tuple: Tuple containing model prediction mask (numpy.ndarray) and centreline path pixels (numpy.ndarray).
            """

            model_pred = model_output(model, device, cv_image)
            # cv2.imshow('seg before post', model_pred)
            # cv2.waitKey(wait_time_max)
            model_pred = post_processing(model_pred, self.post_proc_blur_k_size, self.min_contour_area, self.min_black_area_threshold)
            pixel_route = compute_centreline_path(model_pred, self.poly_degree)

            return model_pred, pixel_route

        

        wait_time_max = 1
        cv_image = get_rgb_undistorted_img(camera_msg)
                
        undistorted_image = cv_image.copy()
        if self.visualize:
            cv2.imshow('Undistorted', cv_image)
            cv2.waitKey(wait_time_max)

        #Pre process image to prepare for segmentation model
        cv_image = pre_process_img(cv_image, self.pre_proc_blur_k_size, self.brightness)
        # if self.visualize:
            # cv2.imshow('pre-processed', cv_image)
            # cv2.waitKey(wait_time_max)

        model_pred, pixel_route = find_route(self.model, self.device, cv_image)
        self.model_pred = model_pred
        uv_route = None
        if self.visualize:
                        
            # cv2.imshow('segmentation_ouput',model_pred)
            # cv2.waitKey(wait_time_max)

            # Highlight red where is predicted as road
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

        if isinstance(pixel_route, np.ndarray): uv_route = self.convert_pix2uv(pixel_route)
        return uv_route, cv_image
    
    #-----POINTCLOUD TRANSFORMATION AND FILTERING-------------------------------------------
    
    def lidar_pts_in_cam_uv(self, points3d: np.ndarray) -> np.ndarray:
        """
        Transforms lidar points to camera frame uvcoordinates.

        Args:
            points3d (numpy.ndarray): Array of lidar points. size: (N, 4), (x, y, z, 1)

        Returns:
            numpy.ndarray: Transformed points in camera frame uv coordinates. (u, v, depth)
        """
        # Transform from lidar to cam frame
        N = points3d.shape[0]
        points3d = points3d.T #(4, N)
        points3d_cam_frame = T_CL @ points3d # (4, N)


        # Convert from cam cartesian to uv
        uv_coordinate = np.zeros((3, N))

        uv_coordinate[0] = points3d_cam_frame[0] / points3d_cam_frame[2] #x / z
        uv_coordinate[1] = points3d_cam_frame[1] / points3d_cam_frame[2] #y / z
        uv_coordinate[2] = points3d_cam_frame[2] # z

        points2d = uv_coordinate.T
        return points2d
    
    def get_target_point_location(self, uv_route: np.ndarray, points2d: np.ndarray) -> tuple[int, np.ndarray]:
        """
        Filters usable route points based on uv route and lidar points. target must be within
        dist_thresh_uv of a lidar point and over min_depth distance away.

        Args:
            uv_route (numpy.ndarray): centreline array in uv coordiantes. (u, v)
            points2d (numpy.ndarray): Lidar points in camera frame. (N, 3) u, v, depth
            self.min_depth (float): Minimum depth threshold.
            self.num_max_points_to_match (int): Maximum number of points to match.
            self.dist_thresh_uv (float): Distance threshold for UV route points.

        Returns:
            tuple: Index of target point in lidar point cloud and corresponding UV coordinates.
        """
        
        # Filter out the points with a negative depth, and add a col that has the corresponding index
        # in the unfiltered array
        N = points2d.shape[0]
        row_indices = np.arange(N)[:, np.newaxis]
        points2d = np.hstack((points2d, row_indices)) #u, v, depth, idx
        filtered_points2d = points2d[points2d[:, 2] >= 0]

        # Only consider the last num_max_points_to_match points or less (points starting from bottom)
        if len(uv_route) < self.num_max_points_to_match: uv_route = uv_route[::-1]
        else: uv_route = uv_route[-self.num_max_points_to_match:][::-1]

        
        best_depth = float('inf')
        lidar_point_uv = np.empty(4)
        final_lidar_point_uv = np.empty(4)
        target_pcl_index = -1

        # Find the closest target that is aligned with a lidar point and over min_depth away
        for i, uv_point in enumerate(uv_route):
            euclid_distances = np.linalg.norm(filtered_points2d[:, :2] - uv_point, axis=1)
            closest_filtered_idx = np.argmin(euclid_distances)
            dist = euclid_distances[closest_filtered_idx]               

            if dist < self.dist_thresh_uv: #lidar point close to path
                lidar_point_uv = filtered_points2d[closest_filtered_idx]
                depth = lidar_point_uv[2]
                if self.min_depth <= depth < best_depth:
                    target_pcl_index = int(lidar_point_uv[3])
                    final_lidar_point_uv = lidar_point_uv
                    best_depth = depth
                    # print("target_pcl_index", target_pcl_index)
                    # print("depth: ", depth)
        return target_pcl_index, final_lidar_point_uv[:2]
    
    def uv2pixel(self, target_uv: np.ndarray, image: np.ndarray) -> None:
        """
        Converts UV coordinates to pixel coordinates and visualizes them on the image.

        Args:
            target_uv (numpy.ndarray): UV coordinates of the target point.
            image (numpy.ndarray): Image for visualization.
        """

        if not self.visualize: return

        # Covert uv coordinates to pixels
        u, v = target_uv
        pix_x = u * self.f_x + self.c_x
        pix_y = v * self.f_y + self.c_y
        pix_x = np.clip(pix_x, 0, self.image_width - 1).astype(int)
        pix_y = np.clip(pix_y, 0, self.image_height - 1).astype(int)


        # convert points in uv to pixels
        # print(self.model_pred.shape)
        # print(self.points2d[:,0].min(),self.points2d[:,0].max(),self.points2d[:,1].min(),self.points2d[:,1].max())
        # self.ground_ids = [int(self.model_pred[x[1],x[0]]) for x in self.points2d[:,:2].astype(int)]
        # print(check)

        # Display the target pose image
        image = cv2.circle(image, (pix_x, pix_y), 10, (0, 255, 0), thickness=3)
        # for i in range(len(self.ground_ids)):
        #     cv2.circle(image, (self.points2d[i,0].astype(int), self.points2d[i,1].astype(int)), 5, (self.ground_ids[i], 0, 0), -1)
        cv2.imshow("Target point", image)
        cv2.waitKey(1)

    #-----CREATE PUBLISH MESSAGE------------------------------------------------------------

    def publish_trail_target_point(self, lidar_msg: PointCloud2, target_point: np.ndarray) -> None:
        """
        Publishes the trail target pose.

        Args:
            lidar_msg (sensor_msgs.msg.PointCloud2): Lidar message.
            target_point (numpy.ndarray): Target point coordinates.
        """
        x, y, z = target_point

        # publish message
        trail_location_msg = PoseStamped()
        trail_location_msg.header.stamp = lidar_msg.header.stamp
        # trail_location_msg.header.stamp = self.get_clock().now().to_msg()
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
        self.get_logger().info(f"Trail waypoint published as Point: {x}, {y}, {z}")
        return
    
    #-----CALLBACK FUNCTIONS----------------------------------------------------------------

    def trail_callback(self, camera_msg: Image, lidar_msg: PointCloud2) -> None:
        """
        Callback function for processing camera and lidar messages and publishing a target path pose.

        Args:
            camera_msg (sensor_msgs.msg.Image): Camera message.
            lidar_msg (sensor_msgs.msg.PointCloud2): Lidar message.
        """

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
        #points3d = np.array([[x, y, z, 1] for x, y, z in point_gen])

        
        points = np.array([point_gen['x'], point_gen['y'], point_gen['z']]).T
        mask = ~np.all(points == 0, axis=1)
        # Apply the mask to filter out rows with all zero coordinates
        points = points[mask]
        # # add the 4th column for matrix multiplication
        points3d = np.hstack((points, np.ones((points.shape[0], 1))))

        # points3d = np.array(points3d)
        
        #Convert pointcloud to camera uv coordinates
        points2d = self.lidar_pts_in_cam_uv(points3d)
        self.points2d = np.copy(points2d)
        self.points2d[:,0] = self.points2d[:,0]*self.f_x+self.c_x
        self.points2d[:,1] = self.points2d[:,1]*self.f_y+self.c_y
        self.fov_ids = (self.points2d[:,0] > 0) * (self.points2d[:,0] < 1280) * (self.points2d[:,1] > 0) * (self.points2d[:,1] < 720) * (self.points2d[:,2] > 0.5)
        self.points2d = self.points2d[self.fov_ids,:]

        target_pcl_index,  target_uv = self.get_target_point_location(uv_route, points2d)
        if target_pcl_index == -1:
            print("No usable centerline found!")
            return
        
        self.uv2pixel(target_uv, cv_image)
        target_point = points3d[target_pcl_index][:3] #array of [x, y, z]

        self.publish_trail_target_point(lidar_msg, target_point)
        #self.publish_ground_points(lidar_msg)
        return

    def only_camera_callback(self, camera_msg: Image) -> None:
        """
        Callback function for processing only camera messages and visualizing path.

        Args:
            camera_msg (sensor_msgs.msg.Image): Camera message.
        """

        print("Camera Message received!")
        uv_route, _ = self.compute_route_uv_and_img(camera_msg)
        if not isinstance(uv_route, np.ndarray):
            print("No centerline found!")
        return

    # def publish_ground_points(self, lidar_msg):
    #     points3d_ground = lidar_msg
    #     print(type(lidar_msg.data), lidar_msg.data)#[self.fov_ids,:][self.ground_ids,:])
    #     try:
    #         map_points = self.tf_buffer.transform(points3d_ground, 'map')
    #         self.ground_points_publisher.publish(map_points)
    #     except tf2_ros.TransformException as ex:
    #         self.get_logger().info('Could not transform os_lidar to map: {0}'.format(ex))
    #         self.ground_points_publisher.publish(points3d_ground)
    #     return 

#  ------------------------ Sub-processing and threading------------------------------
import subprocess, threading, os
def run_shell_command(command):
    with open(os.devnull, 'w') as nullfile:
        process = subprocess.Popen(command, shell=True, stdout=nullfile, stderr=subprocess.STDOUT)
        process.communicate()
#-----MAIN----------------------------------------------------------------------------------

def main(args=None):
    print("cuda is available: ", torch.cuda.is_available())
    
    rclpy.init(args=args)
    trailDetectorNode = trailDetector(ONLY_CAMERA_MODE, VISUALIZE)
    rclpy.spin(trailDetectorNode)

    trailDetectorNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("\n\nDEBUG MODE ON\n\n")
    command = "ros2 bag play /home/trailbot/bags/human_tracking/"
    thread = threading.Thread(target=run_shell_command, args=(command,))
    thread_main = threading.Thread(target=main, args=(None,True))

    thread.start()
    thread_main.start()

    thread.join()
    thread_main.join()

    

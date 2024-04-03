import torch
from .model_loader import FCN8s, PSPNet, LEDNet
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
image_height=720


'''
    The traildetector node has two subscriptions(lidar and camera) and one publisher(trail position). After it receives msgs from both lidar and camera,
    it detects the trail in the image and sends the corresponding lidar position as the trail location.
    The msgs are synchornized before processing, using buffer and sync function.
    To find the path, the node will process the image, find a line to follow (by taking the average of left and right of the path), estimate the lidar points depth, 
    and choose to go to the closest point. 
    V1_traildetection assumes that the path is pointing frontward and has only one path in front.
'''
class trailDetector(Node):
    def __init__(self, only_camera_mode, visualize,
                 pub_queue_size=10, sync_queue_size=30, cam_sub_queue_size=10, max_time_diff=0.5):
        
        super().__init__('trail_detector')
        
        self.only_camera_mode = only_camera_mode
        self.visualize = visualize

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
        # model = FCN8s(nclass=6, backbone='vgg16', pretrained_base=True, pretrained=True)
        # model = PSPNet(nclass=2, backbone='resnet50', pretrained_base=True)
        model = LEDNet(nclass=2, backbone='resnet50', pretrained_base=True)
        model_location = 'lednet_resnet50_trails_best_model.pth' # previously: psp_resnet50_pascal_voc_best_model
        full_path = os.path.expanduser(f'~/.torch/models/{model_location}')
        if os.path.isfile(full_path):
            try:
                model.load_state_dict(torch.load(full_path, map_location=self.device))
                # model.load_state_dict(torch.load(f'src/TRAILBot/trail_detection_node/trail_detection_node/model/{model_location}',map_location=torch.device('cuda:0')))
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

    def compute_route_indices_and_img(self, camera_msg):

        def pre_process_img(image):

            # Split the image into its color channels
            r, g, b = cv2.split(image)

            # Equalize the histograms for each channel
            r_eq = cv2.equalizeHist(r)
            g_eq = cv2.equalizeHist(g)
            b_eq = cv2.equalizeHist(b)

            # Merge the channels
            image = cv2.merge((r_eq, g_eq, b_eq))

            # Apply Fast Non-Local Means Denoising
            #slows down callback
            # h = 10
            # templateWindowSize = 5
            # searchWindowSize = 7
            # image = cv2.fastNlMeansDenoisingColored(image, None, h, 2*h, templateWindowSize, searchWindowSize)

            # Apply Gaussian Blur
            k_size = 23
            image = cv2.GaussianBlur(image, (k_size, k_size), 0)

            # # Increase Brightness
            brightness = 10
            M = np.ones(image.shape, dtype='uint8') * brightness  # Increase brightness by 50 
            image = cv2.add(image, M)

            return image
        
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
        
        def post_processing(model_pred):
            orig_contours, _ = cv2.findContours(model_pred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            epsilon = 3.0
            # Draw contours on a blank image (for visualization)
            mask = np.zeros_like(model_pred)
            post_blur_mask = np.zeros_like(model_pred)

            cv2.drawContours(mask, orig_contours, -1, (255), cv2.FILLED)

            blur_kernel_size = 31
            # Apply Gaussian blur to the mask
            mask = cv2.GaussianBlur(mask, (blur_kernel_size, blur_kernel_size), 0)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            min_contour_area = 150000
            for contour in contours:
                # Calculate the area of each contour
                area = cv2.contourArea(contour)
                
                # Draw contours above a certain size
                if area > min_contour_area:
                    # smoothed_contour = cv2.approxPolyDP(contour, epsilon, False)
                    cv2.drawContours(post_blur_mask, [contour], -1, (255), cv2.FILLED)

            # Show the contour image
            post_blur_mask = remove_small_black_regions(post_blur_mask)
            # cv2.imshow('Contours', post_blur_mask)
            # cv2.waitKey(25)
            return post_blur_mask
        
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

        def compute_best_path(model_pred):
            route = np.zeros_like(model_pred)
            x_idx = []
            y_idx = []
            # calculate the center line by taking the average
            for i, row in enumerate(model_pred):
                white_pixels = list(np.nonzero(row)[0])
                if white_pixels:

                    average = (white_pixels[0] + white_pixels[-1]) // 2
                    # centres[i][average] = 255
                    x_idx.append(i)
                    y_idx.append(average)
            if len(x_idx) > 0:
                x_idx = np.array(x_idx)
                y_idx = np.array(y_idx)

                coefficients = np.polyfit(x_idx, y_idx, deg=2)  # Third-degree polynomial (adjust degree as needed)
                y_new_idx = np.polyval(coefficients, x_idx).astype(int)
                y_new_idx = np.clip(y_new_idx, a_min=0, a_max=image_width-1)

                for x, y in zip(x_idx, y_idx):
                    route[x_idx, y_new_idx] = 255
            return route

        def find_route(model, device, cv_image):
            model_pred = model_output(model, device, cv_image)
            model_pred = post_processing(model_pred)
            route = compute_best_path(model_pred)

            return model_pred, route  #array of size of the original undistorted image where the centreline is of value 255, rest is 0 in camera frame

        # process camera msg
        wait_time_max = 25
        cv_image = self.bridge.imgmsg_to_cv2(camera_msg, desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        original_image = cv_image.copy()
        # if self.visualize:
        #     cv2.imshow('raw', cv_image)
        #     cv2.waitKey(wait_time_max)

        #Pre process image to prepare for segmentation model
        cv_image = pre_process_img(cv_image)
        # if self.visualize:
        #     cv2.imshow('pre-processed', cv_image)
        #     cv2.waitKey(wait_time_max)


        #TODO: Undistort image to bring it to camera frame
        #Apply Segmentation model and visualize
        model_pred, route = find_route(self.model, self.device, cv_image)

        #Extract indices of routes
        route_indices = list(zip(*np.nonzero(route)))

        if self.visualize:
            #To visualize prediction
            # from scipy.ndimage import grey_erosion
            # model_pred = grey_erosion(model_pred, size=(4,4))
            
            # cv2.imshow('segmentation_ouput',model_pred)
            # cv2.waitKey(wait_time_max)
            # cv2.imshow('route',route)
            # cv2.waitKey(wait_time_max)

            # # increase the brightness of image where is predicted as road
            # sign = cv2.cvtColor(model_pred, cv2.COLOR_GRAY2RGB) / 255 * 150
            # sign = sign.astype(original_image.dtype)  # Convert sign to the data type of original_image
            # # cv_image = cv2.add(original_image, sign)
            # cv_image = original_image.copy()
            # cv_image[:, :, 0] = cv2.add(original_image[:, :, 0], sign[:,:,0])
            # cv2.imshow('highlighted_route', cv_image)
            # cv2.waitKey(wait_time_max)

            sign = cv2.cvtColor(model_pred, cv2.COLOR_GRAY2RGB) /255 * 200
            sign = sign.astype(original_image.dtype)  # Convert sign to the data type of original_image
            sign[:, :, :2] = 0
            cv_image = cv2.add(original_image, sign)
            cv2.imshow('highlighted_route', cv_image)
            cv2.waitKey(wait_time_max)


        # Plot the points on path
        if route_indices and self.visualize:
            for index_circle in range(len(route_indices)):
                red_circle_x = route_indices[index_circle][0]
                red_circle_y = route_indices[index_circle][1]
                cv2.circle(cv_image, (red_circle_y, red_circle_x), radius=5, color=(255, 0, 0), thickness=-1)
            cv2.imshow('final_path', cv_image)
            cv2.waitKey(wait_time_max)
            
        return route_indices
    
    #-----POINTCLOUD TRANSFORMATION AND FILTERING-------------------------------------------
    
    def lidar_pts_cam_frame(self, lidar_msg):

        def convert_to_camera_frame(point_cloud):
            """
            convert 3d lidar data into 2d coordinate of the camera frame + depth
            """
            length = point_cloud.shape[0]
            translation = np.tile(translation_vector, (length, 1)).T
            
            point_cloud = point_cloud.T
            point_cloud = rotation_matrix@point_cloud + translation
            #TODO: remove below, no need to bring lidar into undistorted camera frame
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

        # process lidar msg
        point_gen = pc2.read_points(
            lidar_msg, field_names=(
                "x", "y", "z"), skip_nans=True)
        points = [[x, y, z] for x, y, z in point_gen]
        points = np.array(points)
        points2d = convert_to_camera_frame(points) #u, v, depth
        return points2d
    
    def filter_usable_route_points(self, route_indices, points2d):
        #filter points that have no lidar points near it

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

        route_indices_near_lidar_pts = []
        for index in route_indices:
            point = []
            u = index[1]
            v = image_height - index[0] #TODO: Check why image_height is subtracted and where indices point to
            point.append(u)
            point.append(v)
            point.append(estimate_depth(u, v, points2d))
            if point[2] == -1:
                continue
            else:
                route_indices_near_lidar_pts.append(point)
        return route_indices_near_lidar_pts
    
    def points_to_lidar_frame(self, route_indices_near_lidar_pts):
        # find the corresponding lidar points using the center line pixels

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

        filtered_3dPoints = []
        #TODO: eliminate for loop and convert to lidar frame through matrix mult
        for index in route_indices_near_lidar_pts:
            point = []
            point.append(index[0])
            point.append(index[1])
            point.append(index[2])
            point_3d = convert_to_lidar_frame(point)
            filtered_3dPoints.append(point_3d)

        filtered_3dPoints = np.array(filtered_3dPoints)
        return filtered_3dPoints
    
    #-----CREATE PUBLISH MESSAGE------------------------------------------------------------

    def publish_trail_target_point(self, lidar_msg, filtered_3dPoints):
        # find the nearest 3d point and set that as goal
        distances_sq = filtered_3dPoints[:,0]**2 + filtered_3dPoints[:,1]**2 + filtered_3dPoints[:,2]**2
        smallest_index = np.argmin(distances_sq)

        # publish message
        trail_location_msg = PoseStamped()
        trail_location_msg.header.stamp = lidar_msg.header.stamp
        trail_location_msg.header.frame_id = "os_lidar"
        
        # position
        trail_location_msg.pose.position.x = filtered_3dPoints[smallest_index][0]  
        trail_location_msg.pose.position.y = filtered_3dPoints[smallest_index][1]
        trail_location_msg.pose.position.z = filtered_3dPoints[smallest_index][2]

        # orientation
        yaw = math.atan2(filtered_3dPoints[smallest_index][1], filtered_3dPoints[smallest_index][0])
        trail_location_msg.pose.orientation.x = 0.0  
        trail_location_msg.pose.orientation.y = 0.0 
        trail_location_msg.pose.orientation.z = math.sin(yaw/2)
        trail_location_msg.pose.orientation.w = math.cos(yaw / 2)
        self.trail_publisher.publish(trail_location_msg)
        
        # logging
        self.get_logger().info("location published!")
        self.get_logger().info(f"Point: {filtered_3dPoints[smallest_index][0]}, {filtered_3dPoints[smallest_index][1]}, {filtered_3dPoints[smallest_index][2]}")
        return
    
    #-----CALLBACK FUNCTIONS----------------------------------------------------------------

    def trail_callback(self, camera_msg, lidar_msg):
        print("Camera and Lidar Message received!")

        # process camera msg to retrieve path
        route_indices = self.compute_route_indices_and_img(camera_msg)
        if not route_indices:
            print("No centerline found!")
            return
        
        #Convert pointcloud to camera uv coordinates
        points2d = self.lidar_pts_cam_frame(lidar_msg)

        route_indices_near_lidar_pts = self.filter_usable_route_points(route_indices, points2d)
        if not route_indices_near_lidar_pts:
            print("No usable centerline found!")
            return
        
        filtered_3dPoints = self.points_to_lidar_frame(route_indices_near_lidar_pts)
        self.publish_trail_target_point(self, lidar_msg, filtered_3dPoints)
        return

    def only_camera_callback(self, camera_msg):
        print("Camera Message received!")
        route_indices = self.compute_route_indices_and_img(camera_msg)
        if not route_indices:
            print("No centerline found!")
        return

#-----MAIN----------------------------------------------------------------------------------

def main(args=None):
    print("cuda is available: ", torch.cuda.is_available())
    only_camera_mode = True
    visualize = True
    
    rclpy.init(args=args)
    trailDetectorNode = trailDetector(only_camera_mode, visualize)
    rclpy.spin(trailDetectorNode)

    trailDetectorNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

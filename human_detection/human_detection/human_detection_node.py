
from vision_msgs.msg import Detection3DArray, Detection3D # sudo apt-get install ros-humble-vision-msgs
import argparse
from cv_bridge import CvBridge
import cv2
# from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Bool, Float32, String
# import tensorflow as tf
# import tensorflow_hub as hub
import time
import yaml

import torch

import sys
import os
# Get the current directory of the main.py file
current_directory = os.path.dirname(os.path.abspath(__file__))
# Add the current directory to the Python path
sys.path.append(current_directory)
#print(current_directory)
import yolov7
from geometry_msgs.msg import PoseStamped

SHOW_IMAGE_WINDOW = True 

def parse_arguments():
    """
    handle command line arguments
    """
    parser = argparse.ArgumentParser(description='Example command-line parser')
    parser.add_argument(
        '-v',
        '--verbose',
        action='store_true',
        help='Enable print_verbose_only outputs')
    parser.add_argument(
        '-d',
        '--download_model',
        action='store_true',
        help='Flag to download the model. This must be ran at least once')
    parser.add_argument(
        '-r',
        '--ros-args',
        action='store_true',
        help='temp fix')
    return parser.parse_args()


# def download_model(SAVED_MODEL_PATH,MODEL_URL):
#     # Create the directory if it doesn't exist
#     os.makedirs(os.path.dirname(SAVED_MODEL_PATH), exist_ok=True)

#     ### to be finished


def print_verbose_only(parser_args,*args, **kwargs):
    """
    print only if verbose==True
    """
    if parser_args.verbose:
        print(*args, **kwargs)


# def movenet(input_image, model,configs):
#     """
#     movenet model:
#     Gets input image and outputs array of keypoints with certainty score
#     downloaded from #https://tfhub.dev/google/movenet/multipose/lightning/1
#     """
#     # SavedModel format expects tensor type of int32.
#     input_image = tf.cast(input_image, dtype=tf.int32)
#     outputs = model(input_image)  # Output is a [1, 6, 56] tensor.

#     # The first 17 * 3 elements are the keypoint locations and scores in the
#     # format: [y_0, x_0, s_0, y_1, x_1, s_1, …, y_16, x_16, s_16], where y_i,
#     # x_i, s_i are the yx-coordinates (normalized to image frame, e.g. range
#     # in [0.0, 1.0]) and confidence scores of the i-th joint correspondingly.
#     # The order of the 17 keypoint joints is: [nose, left eye, right eye, left
#     # ear, right ear, left shoulder, right shoulder, left elbow, right elbow,
#     # left wrist, right wrist, left hip, right hip, left knee, right knee,
#     # left ankle, right ankle]. The remaining 5 elements [ymin, xmin, ymax,
#     # xmax, score] represent the region of the bounding box (in normalized
#     # coordinates) and the confidence score of the instance
#     keypoints = outputs['output_0'].numpy()

#     count_of_people = np.sum(keypoints[0, :, -1] > configs['people_detection_threshold'] )
#     # print_verbose_only("count_of_people", count_of_people)

#     # there are 6 people
#     # there are 17 body points and therefore 3*17=51 numbers per person
#     return keypoints[:, :, :51].reshape((6, 17, 3))[0]


def xyxy_to_centroid(xyxy):
    x1, y1, x2, y2 = xyxy
    centroid_x = (x1 + x2) / 2
    centroid_y = (y1 + y2) / 2
    return (centroid_x, centroid_y)

def get_heading_angle(
        centroid,
        fov=90,
        image_width=1,
        offset=0,
        scaling=1):
    """
    get the heading angle from the camera's perspective to the person,
     in degree, relative to the center of the field of view
    """
    centroid_x, centroid_y = centroid
    x_angle_radian = math.atan(
        (centroid_x - (image_width / 2)) / (image_width / 2) * math.tan(math.radians(fov / 2)))
    return offset + scaling * math.degrees(x_angle_radian)



class Person:
    """
    struct to store information for a detected person
    """
    def __init__(self):
        self.x = -1.0
        self.y = -1.0
        self.world_xyz = None
        self.on_screen = False
        self.heading_angle = 0.0
        self.id = 0
        self.is_valid = False


class internalState:
    human_max_speed = 2.8 # m/s
    fps = 10
    buffer_ratio = 1.5 # allow fluctuation of up to 1.5 times 
    max_movement_per_frame = human_max_speed / fps * buffer_ratio
    moving_average_weights = [10,5,3,2,1]

    def __init__(self,depth_history_length):
        # Instance attributes (unique to each instance)
        self.depth_history = []
        self.discarded_depth_history = []
        self.missing_frame_count = 0 
        self.depth_history_length =depth_history_length  

    def weighted_moving_average(self,data, weights):
        num_points = min(len(data), len(weights))
        weights_sum = 0
        weighted_data_sum = 0
        for i in range(num_points):
            weighted_data_sum += weights[i]*data[-1-i] 
            weights_sum += weights[i]
        return weighted_data_sum/weights_sum

    def get_average(self):
        return self.weighted_moving_average(self.depth_history,internalState.moving_average_weights)


    def append(self, new_depth):
        if len(self.depth_history) < self.depth_history_length:
            self.depth_history.append(new_depth)
            return 0

        avg = self.get_average()
        if abs(avg-new_depth) < internalState.max_movement_per_frame*self.missing_frame_count:
            self.depth_history.pop(0)
            self.depth_history.append(new_depth)
            self.missing_frame_count = 0
            return 0
        self.missing_frame_count +=1
        self.discarded_depth_history.append(new_depth)
        return 1

class LidarCameraSubscriber(Node):
    def print_and_log(self, string):
        self.get_logger().info(string)
        print_verbose_only(self.parser_args,string)


    def __init__(self,parser_args,model,configs):
        self.person_array = []
        self.is_there_anyone = False
        self.cur_state = "SearchState" # initial state
        self.parser_args = parser_args
        self.model = model
        self.configs=configs

        camera_transformation_k = configs['camera_transformation_k']
        self.camera_transformation_k = read_space_separated_matrix(camera_transformation_k)
        rotation_matrix = configs['rotation_matrix']
        # self.rotation_matrix = read_space_separated_matrix(rotation_matrix).T
        # self.translation_vector = np.array(configs['translation_vector'])
        # self.inverse_camera_transformation_k = np.linalg.inv(self.camera_transformation_k)
        # self.inverse_rotation_matrix = np.linalg.inv(self.rotation_matrix)
        self.radial_distortion = np.array([0.0, 0.0, 0, 0])

        self.T_CL = np.zeros((3,4))
        self.T_CL[:,:3] = read_space_separated_matrix(rotation_matrix)
        self.T_CL[:,3] = np.array(configs['translation_vector'])

        print(self.camera_transformation_k)

        self.image_height = configs['image_height']
        self.image_width = configs['image_width']
        self.new_K, _ = cv2.getOptimalNewCameraMatrix(self.camera_transformation_k, self.radial_distortion, (self.image_width, self.image_height), 1, (self.image_width, self.image_height))  

        super().__init__('image_subscriber')
        self.camera_subscription = self.create_subscription(
            Image,
            'camera',
            self.camera_callback,
            10)
        self.camera_subscription
        self.bridge = CvBridge()

        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            'ouster/points',  
            self.lidar_callback,
            10)
        self.lidar_subscription

        self.state_subscription = self.create_subscription(
            String,
            '/trailbot_state',
            self.state_callback,
            10)


        #topics to publish
        self.is_person_publisher = self.create_publisher(
            Bool,
            'is_person_topic',
            10)
        #self.pose_publisher = self.create_publisher( # uncommented the target location mahan
        #     PoseStamped,
        #     'target_location', 
        #     10)
        self.detection3DArray_publisher = self.create_publisher(
            Detection3DArray,
            'detection_location', 
            10)

        self.detection3DArray_subscriber = self.create_subscription(
            Detection3DArray,
            'detection_location',
            self.array_callback,
            10)

        self.detectionSinglePose_publisher = self.create_publisher(
            PoseStamped,
            'person_target', 
            10)


        self.timestamp = 0
        self.cv_image = None
        self.curr_msg = Detection3DArray()
        self.offset = 0.5

        # if this is -1, node will publish constantly (as camera FPS)
        # self.publishing_frequency = configs['publishing_frequency']
        self.publishing_frequency = -1

        #run the publish_message function according to publishing_frequency
        # self.create_timer(5, self.det_out_callback)
        self.print_and_log('Human Detection ready...')
               
        if SHOW_IMAGE_WINDOW:
            cv2.namedWindow("Camera Image", cv2.WINDOW_NORMAL)
            # cv2.setWindowProperty("Camera Image", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    def process_frame(self,image):
        """
        process a frame. Determine keypoints and number of people and
        heading angle.
        """
        # Run model inference

        person_array = []
        bounding_boxes, identities, confidences=self.model.process_frame(image, view_img=False)
        if identities is None:
            print("identities is none")
            return []
        for i in range(len(bounding_boxes)):
            person = Person()
            centroid = xyxy_to_centroid(bounding_boxes[i])
            person.heading_angle = get_heading_angle(centroid)
            person.x, person.y = centroid 
            person.on_screen=True
            person.id = identities[i]
            person_array.append(person)
        return person_array

    def visualize_camera(self,show_image_window=True, is_person=False, is_valid=0):
        if show_image_window and self.cv_image is not None:
            # print("CAMERA")
            image_with_dots = self.cv_image.copy()

            if is_person:
                for person in self.person_array:
                    cv2.circle(image_with_dots, (int(person.x), int(person.y)), 5, (0, 0, 255), -1)  # Draw a red circle at (x, y)
                    cv2.putText(image_with_dots, str(person.id),  (int(person.x), int(person.y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2) 

            if is_valid:
                for i in range(self.filtered.shape[0]):
                    cv2.circle(image_with_dots, (int(self.filtered[i,0]), int(self.filtered[i,1])), 5, (int(self.filtered[i,2]), 0, 0), -1)  # Draw a red circle at (x, y)
                for i in range(self.points_lidar.shape[0]):
                    cv2.circle(image_with_dots, (int(self.points_lidar[i,0]), int(self.points_lidar[i,1])), 5, (0, 255, 255), -1)  # Draw a red circle at (x, y)                
    
            cv2.imshow("Camera Image", image_with_dots)
            # Check for the 'q' key press to exit the loop
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return


    def state_callback(self, msg):
        self.cur_state = msg.data[11:] # Removes timestamp in front of string, ex: [17:44:37] 


    def camera_callback(self, msg):
        if self.cur_state!="SearchState" and self.cur_state!="ApproachState" and self.cur_state!="StandbyState":
            print('is query?', self.cur_state)
            return 

        print('is not query?', self.cur_state)
        self.cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB) #Added for colour correction to RGB
        self.cv_image = cv2.undistort(self.cv_image, self.camera_transformation_k, self.radial_distortion, None, self.new_K)
        self.person_array = self.process_frame(self.cv_image)
        self.is_there_anyone = len(self.person_array)>0
        self.timestamp = msg.header.stamp



    def lidar_callback(self, msg):
        if self.cur_state!="SearchState" and self.cur_state!="ApproachState" and self.cur_state!="StandbyState":
            self.visualize_camera(SHOW_IMAGE_WINDOW)
            return 

        if not self.is_there_anyone:
            self.visualize_camera(SHOW_IMAGE_WINDOW)
            return
        # Deserialize PointCloud2 data into xyz points
        t1 = time.time()
        point_gen = pc2.read_points(
            msg, field_names=(
                "x", "y", "z"), skip_nans=True)
        #points = [[x, y, z, 1] for x, y, z in point_gen if any([i!=0 for i in [x,y,z]])]
        
        points = np.array([point_gen['x'], point_gen['y'], point_gen['z']]).T
        mask = ~np.all(points == 0, axis=1)
        # Apply the mask to filter out rows with all zero coordinates
        points = points[mask]
        #points = np.array(points)
        # # add the 4th column for matrix multiplication
        points = np.hstack((points, np.ones((points.shape[0], 1))))

        points2d, ids_fov = self.convert_to_camera_frame(points)
        points_fov = points[ids_fov,:]
        #self.get_logger().info(f'Lidar callback time: {time.time()-t1}')
        #update depth for every person
        for person in self.person_array:
            if person.on_screen:
                print('on screen')
                person.world_xyz = self.estimate_position(person, points2d, points_fov)
                if type(person.world_xyz) == list:
                    #print("person validity")
                    person.is_valid = True
                # else:
                #     print(person.world_xyz)
        self.timestamp = msg.header.stamp
        #print(msg.header.stamp)
        # if this is -1, node will publish constantly (as camera FPS)
        # print('lidar',self.timestamp, self.get_clock().now())
        if not self.publishing_frequency>0:
            self.publish_message()
        self.det_out_callback()

    def publish_message(self):
        """ publish message if human is detected"""
        if self.cur_state!="SearchState" and self.cur_state!="ApproachState" and self.cur_state!="StandbyState":
            self.visualize_camera(SHOW_IMAGE_WINDOW)
            return 
        if not self.is_there_anyone:
            self.visualize_camera(SHOW_IMAGE_WINDOW)
            return

        # Publish the message
        is_person_msg = Bool()
        is_person_msg.data = bool(self.is_there_anyone)
        self.is_person_publisher.publish(is_person_msg)

        detection_array = Detection3DArray()
        detection_array.header.frame_id = 'os_lidar'
        for person in self.person_array:
            print(person.is_valid,person.world_xyz)
            if not person.is_valid or type(person.world_xyz) != list:
                continue
            # print(person.world_xyz)
        #     # message = f"id {person.id} coord: {round(person.world_xyz[0],2)},{round(person.world_xyz[1],2)},{round(person.world_xyz[2],2)}"
            message = "id: {0}, coord: [{1:.2f},{2:.2f},{3:.2f}]".format(person.id, person.world_xyz[0], person.world_xyz[1], person.world_xyz[2])
        #     # print_verbose_only(self.parser_args, message)
            self.print_and_log(message)

            detection3d = Detection3D()
            detection3d.bbox.center.position.x = person.world_xyz[0]
            detection3d.bbox.center.position.y = person.world_xyz[1]
            detection3d.bbox.center.position.z = person.world_xyz[2]
            detection3d.id = str(person.id)
            # detection3d.bbox.size.x = float(0)
            # detection3d.bbox.size.y = float(0) 
            # detection3d.bbox.center.orientation.w = float(0)

            detection_array.detections.append(detection3d)
        self.visualize_camera(SHOW_IMAGE_WINDOW, is_person=True, is_valid=len(detection_array.detections))
        self.detection3DArray_publisher.publish(detection_array)
        # print('d3d', self.get_clock().now())


        # pose_stamped_msg = PoseStamped()
        # pose_stamped_msg.header.stamp = self.timestamp
        # pose_stamped_msg.header.frame_id = "velodyne"

        # lidar_x,lidar_y,lidar_z = convert_to_lidar_frame(
        #     (person0.x,person0.y,person0.z),
        #     self.inverse_camera_transformation_k,
        #     self.inverse_rotation_matrix,
        #     self.translation_vector,
        #     self.configs)
        
        # #position
        # pose_stamped_msg.pose.position.x = lidar_x  
        # pose_stamped_msg.pose.position.y = lidar_y
        # pose_stamped_msg.pose.position.z = lidar_z

        # #orientation
        # yaw = math.atan2(lidar_y, lidar_x)
        # pose_stamped_msg.pose.orientation.x = 0.0  
        # pose_stamped_msg.pose.orientation.y = 0.0 
        # pose_stamped_msg.pose.orientation.z = math.sin(yaw/2)
        # pose_stamped_msg.pose.orientation.w = math.cos(yaw / 2)
        
        # self.pose_publisher.publish(pose_stamped_msg)

    def array_callback(self, msg):
        self.curr_msg = msg

    def det_out_callback(self):
        if self.cur_state == "StandbyState":
            trail_pose = PoseStamped()
            trail_pose.header.stamp = self.curr_msg.header.stamp
            trail_pose.header.frame_id = "base_link"

            trail_pose.pose.position.x = 0.0
            trail_pose.pose.position.y = 0.0
            trail_pose.pose.position.z = 0.0
            
            # # position
            # pose.pose.position.x = x
            # pose.pose.position.y = y
            # pose.pose.position.z = z

            # orientation
            trail_pose.pose.orientation.x = 0.0  
            trail_pose.pose.orientation.y = 0.0 
            trail_pose.pose.orientation.z = 0.0
            trail_pose.pose.orientation.w = 0.0
            self.detectionSinglePose_publisher.publish(trail_pose)
            
        else:
            print('det_out')
            if self.curr_msg.detections:
                trail_pose = PoseStamped()
                trail_pose.header.stamp = self.curr_msg.header.stamp
                trail_pose.header.frame_id = "os_lidar"

                trail_pose.pose.position = self.curr_msg.detections[0].bbox.center.position
                x = self.curr_msg.detections[0].bbox.center.position.x
                y = self.curr_msg.detections[0].bbox.center.position.y
                
                # # position
                # pose.pose.position.x = x
                # pose.pose.position.y = y
                # pose.pose.position.z = z

                # orientation
                yaw = math.atan2(y, x)
                trail_pose.pose.orientation.x = 0.0  
                trail_pose.pose.orientation.y = 0.0 
                trail_pose.pose.orientation.z = math.sin(yaw/2)
                trail_pose.pose.orientation.w = math.cos(yaw/2)
                self.detectionSinglePose_publisher.publish(trail_pose)

    def convert_to_lidar_frame(self,
        uv_coordinate, 
        inverse_camera_transformation_k,
        inverse_rotation_matrix,
        translation_vector,
        configs):
        """
        convert 2d camera coordinate + depth into 3d lidar frame
        """
        image_height = configs['image_height']

        point_cloud = np.empty( (3,) , dtype=float)
        point_cloud[2] = uv_coordinate[2]
        point_cloud[1] = ( image_height - uv_coordinate[1] )*point_cloud[2]
        point_cloud[0] = uv_coordinate[0]*point_cloud[2]

        point_cloud = inverse_camera_transformation_k @ point_cloud
        point_cloud = inverse_rotation_matrix @ (point_cloud-translation_vector) 
        return point_cloud


    def convert_to_camera_frame(self, point_cloud):
        """
        convert 3d lidar data into 2d coordinate of the camera frame + depth
        """
        N = point_cloud.shape[0]
        # print(point_cloud.shape)
        points3d_cam_frame = self.T_CL @ point_cloud.T # (3, N)

        uv_coordinate = np.zeros((3, N))

        uv_coordinate[0,:] = points3d_cam_frame[0,:] / points3d_cam_frame[2,:] #x / z
        uv_coordinate[1,:] = points3d_cam_frame[1,:] / points3d_cam_frame[2,:] #y / z
        # uv_coordinate[2,:] = points3d_cam_frame[2,:] # z
        uv_coordinate[2,:] = 1

        pixel_coords = self.camera_transformation_k @ uv_coordinate
        # pixel_coords[1,:] = 720 - pixel_coords[1,:]

        points2d = pixel_coords.T
        dist = np.sqrt((point_cloud[:,:3]**2).sum(axis=1))
        ids = (points2d[:,0]>0) * (points2d[:,0]<1280) * (points2d[:,1]>0) * (points2d[:,1]<720) * (dist > 0.5)
        filtered_points = points2d[ids,:]#[point_cloud[:,2] > 0,:]
        self.filtered = points2d[ids,:]
        self.filtered[:,2] = dist[ids]/10*255
        # print(np.hstack((point_cloud[ids,:],np.array([dist[ids]]).T)))
        return filtered_points, ids


        # length = point_cloud.shape[0]
        # translation = np.tile(translation_vector, (length, 1)).T
        
        # point_cloud = point_cloud.T
        # point_cloud = rotation_matrix@point_cloud + translation
        # point_cloud = camera_transformation_k @ point_cloud

        # uv_coordinate = np.empty_like(point_cloud)

        # """
        # uv = [x/z, y/z, z], and y is opposite so the minus imageheight
        # """
        # uv_coordinate[0] = point_cloud[0] / point_cloud[2]
        # uv_coordinate[1] = self.image_height - point_cloud[1] / point_cloud[2]
        # uv_coordinate[2] = point_cloud[2]

        # uv_depth = uv_coordinate[2, :]
        # filtered_uv_coordinate = uv_coordinate[:, uv_depth >= 0]
        # return filtered_uv_coordinate


    def estimate_position(self, person, points_pix, points_xyz):
        """
        estimate the position by finding points closest to x,y from the 2d array and averaging the points
        """
        # Calculate the distance between each point and the target coordinates (x, y)
        if points_pix.shape[0] < 5:
            return 0
        x = person.x
        y = person.y
        distances_sq = (points_pix[:,0] - x) ** 2 + (points_pix[:,1] - y) ** 2

        # Find the indices of the k nearest points
        k = 5     # Number of nearest neighbors we want
        closest_indices = np.argpartition(distances_sq, k)[:k]
        pixel_distance_threshold = 2000
        # print(points_uv[:10,0],x,y)

        self.points_lidar = points_pix[closest_indices,:2]

        valid_indices = [idx for idx in closest_indices if distances_sq[idx]<=pixel_distance_threshold]
        if len(valid_indices) == 0:
            # print(points_pix[closest_indices,:],x,y)
            # lidar points disappears usually around 0.4m
            distance_where_lidar_stops_working = self.configs['distance_where_lidar_stops_working']
            return distance_where_lidar_stops_working

        filtered_indices = np.array(valid_indices)
        # Get the depth value of the closest point
        closest_points = points_xyz[filtered_indices,:3]
        mean_val = np.mean(closest_points, axis=0)
        dist = mean_val.dot(mean_val)**.5
        mean_val = mean_val * (dist - self.offset)/dist

        # print(person.x, person.y, closest_points)
        return mean_val.tolist()

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

def main(args=None, debug_mode=False):

    with open('/home/trailbot/trail_ws/src/TRAILBot/human_detection/configs.yaml', 'r') as file:
        configs = yaml.safe_load(file)

    parser_args = parse_arguments()
    if debug_mode:
        parser_args.verbose = True

    # if parser_args.download_model:
    #     print('downloading model...')
    #     download_model(SAVED_MODEL_PATH,MODEL_URL)

    with torch.no_grad():
        yolo_sort_tracker=yolov7.Yolo_sort_tracker(save_result=False)

    rclpy.init(args=args)
    subscriber = LidarCameraSubscriber(parser_args, yolo_sort_tracker, configs)
    subscriber.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, False)])
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


import subprocess, threading, os
def run_shell_command(command):
    with open(os.devnull, 'w') as nullfile:
        process = subprocess.Popen(command, shell=True, stdout=nullfile, stderr=subprocess.STDOUT)
        process.communicate()


if __name__ == '__main__':

    print("\n\nDEBUG MODE ON\n\n")
    command1 = "ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera/compressed --remap out:=/camera"
    command2 = "ros2 bag play /home/trailbot/bags/IndoorTest1/aaa"

    # Create threads for each shell command and main function
    thread1 = threading.Thread(target=run_shell_command, args=(command1,))
    thread2 = threading.Thread(target=run_shell_command, args=(command2,))
    thread_main = threading.Thread(target=main, args=(None,True))

    thread1.start()
    thread2.start()
    thread_main.start()

    # Wait for the threads to finish
    thread1.join()
    thread2.join()
    thread_main.join()
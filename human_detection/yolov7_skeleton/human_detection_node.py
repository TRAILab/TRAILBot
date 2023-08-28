
from vision_msgs.msg import Detection3DArray, Detection3D # sudo apt-get install ros-humble-vision-msgs
import argparse
from cv_bridge import CvBridge
import cv2
from geometry_msgs.msg import PoseStamped
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

import yolov7
import torch

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
def process_frame(model,image,configs):
    """
    process a frame. Determine keypoints and number of people and
    heading angle.
    """
    # Run model inference
    person_array = []
    bounding_boxes, identities, confidences=model.process_frame(image,view_img=False)
    for i in range(len(bounding_boxes)):
        person = Person()
        centroid = xyxy_to_centroid(bounding_boxes[i])
        person.heading_angle = get_heading_angle(centroid)
        person.x, person.y = centroid 
        person.on_screen=True
        person.id = identities[i]
        person_array.append(person)
    return person_array


class Person:
    """
    struct to store information for a detected person
    """
    def __init__(self):
        self.x = -1.0
        self.y = -1.0
        self.z = -1.0
        self.on_screen = False
        self.heading_angle = 0.0
        self.id = 0


class LidarCameraSubscriber(Node):
    def print_and_log(self, string):
        self.get_logger().info(string)
        print(string)


    def __init__(self,parser_args,model,configs):
        #make array of 6 person
        self.person_array = []
        self.is_there_anyone = False
        self.cur_state = "SearchState" # initial state
        self.parser_args = parser_args
        self.model = model
        self.configs=configs

        camera_transformation_k = configs['camera_transformation_k']
        self.camera_transformation_k = read_space_separated_matrix(camera_transformation_k)
        rotation_matrix = configs['rotation_matrix']
        self.rotation_matrix = read_space_separated_matrix(rotation_matrix).T
        self.translation_vector = np.array(configs['translation_vector'])
        self.inverse_camera_transformation_k = np.linalg.inv(self.camera_transformation_k)
        self.inverse_rotation_matrix = np.linalg.inv(self.rotation_matrix)

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
            'velodyne_points',  
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
        # self.pose_publisher = self.create_publisher(
        #     PoseStamped,
        #     'target_location', 
        #     10)
        self.detection3DArray_publisher = self.create_publisher(
            Detection3DArray,
            'detection_location', 
            10)

        self.timestamp = 0

        # if this is -1, node will publish constantly (as camera FPS)
        self.publishing_frequency = configs['publishing_frequency']

        #run the publish_message function according to publishing_frequency
        self.create_timer(1/self.publishing_frequency, self.publish_message)
        self.print_and_log('Human Detection ready...')
        
        ascii_numbers = r"""
        ____ _ _  _ ____                 
        |___ | |  | |___                 
        |    |  \/  |___                 

        ____ ____ _  _ ____              
        |___ |  | |  | |__/              
        |    |__| |__| |  \              

        ___ _  _ ____ ____ ____          
         |  |__| |__/ |___ |___          
         |  |  | |  \ |___ |___          

        ___ _ _ _ ____                   
         |  | | | |  |                   
         |  |_|_| |__|                   

        ____ _  _ ____                   
        |  | |\ | |___                   
        |__| | \| |___                   

        ____ ___ ____ ____ ___ ____ ___  
        [__   |  |__| |__/  |  |___ |  \  |
        ___]  |  |  | |  \  |  |___ |__/  .
        """.strip().split('\n\n')

        for num in ascii_numbers[-6:]:
            self.print_and_log(f"\n{num}\n")
            time.sleep(1)


    def state_callback(self, msg):
        self.cur_state = msg.data


    def camera_callback(self, msg):

        if self.cur_state!="SearchState" and self.cur_state!="ApproachState":
            return 

        cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        self.person_array = process_frame(self.model, cv_image, self.configs)
        self.is_there_anyone = len(self.person_array)>0
        self.timestamp = msg.header.stamp
        if show_image_window:=True:
            print("CAMERA!")
            cv2.imshow("Camera Image", cv_image)
            cv2.waitKey(0)  # Wait for a key press
            cv2.destroyAllWindows()  # Close all OpenCV windows



    def lidar_callback(self, msg):
        if self.cur_state!="SearchState" and self.cur_state!="ApproachState":
            return 

        if not self.is_there_anyone:
            return
            
        # Deserialize PointCloud2 data into xyz points
        point_gen = pc2.read_points(
            msg, field_names=(
                "x", "y", "z"), skip_nans=True)
        # points = np.array(list(point_gen))
        points = [[x, y, z] for x, y, z in point_gen]
        points = np.array(points)
        points2d = convert_to_camera_frame(
            points,
            self.camera_transformation_k,
            self.rotation_matrix,
            self.translation_vector,
            self.configs)

        #update depth for every person
        for person in self.person_array:
            if not person.on_screen:
                person.z = -1.0
            else:
                person.z = estimate_depth(person.x, person.y, points2d,self.configs)
        self.timestamp = msg.header.stamp
        # if this is -1, node will publish constantly (as camera FPS)
        if not self.publishing_frequency>0:
            self.publish_message("lidar")

    def publish_message(self,source_str="timer"):
        """ publish message is somebody is detected"""
        if self.cur_state!="SearchState" and self.cur_state!="ApproachState":
            return 
        if not self.is_there_anyone:
            return

        #person0 for debugging purpse
        person0 = self.person_array[0]
        message = f"{source_str:<7}"
        message += f" person: {'YES' if self.is_there_anyone else 'NO '}"
        message += f" angle: {person0.heading_angle:<20}"
        message += f"person_coordinate: {person0.x:<22} {person0.y:<22} {person0.z:22}"
        print_verbose_only(self.parser_args, message)
        self.print_and_log(message)

        # Publish the message
        is_person_msg = Bool()
        is_person_msg.data = bool(self.is_there_anyone)
        self.is_person_publisher.publish(is_person_msg)

        detection_array = Detection3DArray()

        for person in self.person_array:
            new_object = Detection3D()
            lidar_x,lidar_y,lidar_z = convert_to_lidar_frame(
                (person.x,person.y,person.z),
                self.inverse_camera_transformation_k,
                self.inverse_rotation_matrix,
                self.translation_vector,
                self.configs)

            new_object.bbox.center.position.x = float(lidar_x)
            new_object.bbox.center.position.y = float(lidar_y)
            new_object.bbox.center.position.z = float(lidar_z)
            # new_object.bbox.size.x = float(0)
            # new_object.bbox.size.y = float(0) 
            # new_object.bbox.center.orientation.w = float(0)

            detection_array.detections.append(new_object)

        self.detection3DArray_publisher.publish(detection_array)


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


def convert_to_lidar_frame(
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


def convert_to_camera_frame(
    point_cloud,
    camera_transformation_k,
    rotation_matrix,
    translation_vector,
    configs):
    """
    convert 3d lidar data into 2d coordinate of the camera frame + depth
    """
    image_height = configs['image_height']

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


def estimate_depth(x, y, np_2d_array,configs):
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
        distance_where_lidar_stops_working = configs['distance_where_lidar_stops_working']
        return distance_where_lidar_stops_working

    filtered_indices = np.array(valid_indices)
    # Get the depth value of the closest point
    closest_depths = np_2d_array[2,filtered_indices]

    return np.mean(closest_depths)


def main(args=None, debug_mode=False):

    with open('configs.yaml', 'r') as file:
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
    subscriber = LidarCameraSubscriber(parser_args,yolo_sort_tracker,configs)
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
    command2 = ""#"ros2 bag play /home/trailbot/bags/2023-07-13-17:03"

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
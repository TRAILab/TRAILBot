import argparse
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Bool, Float32
import tarfile
import tensorflow as tf
import tensorflow_hub as hub
import time
import urllib.request

temp=""
#config constants
MODEL_URL = "https://tfhub.dev/google/movenet/multipose/lightning/1?tf-hub-format=compressed"
SAVED_MODEL_PATH = "./multipose_model"
people_detection_threshold = 0.3
point_detection_threshold = 0.3
image_width=1280
image_height=1024
camera_transformation_k = """
    628.5359544 0 676.9575694
    0 627.7249542 532.7206716
    0 0 1
"""
rotation_matrix = """
    -0.007495781893 -0.0006277316155    0.9999717092
    -0.9999516401   -0.006361853422 -0.007499625104
    0.006366381192  -0.9999795662   -0.0005800141927
"""
translation_vector = np.array([-0.06024059837, -0.08180891509, -0.3117851288])


#globals 
parser_args = tuple()
model = None 


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
    return parser.parse_args()


def download_model():
    # Create the directory if it doesn't exist
    os.makedirs(os.path.dirname(SAVED_MODEL_PATH), exist_ok=True)

    # Download the compressed model from the URL
    model_path, _ = urllib.request.urlretrieve(MODEL_URL)

    # Extract the compressed model to the specified path
    with tarfile.open(model_path, "r:gz") as tar:
        tar.extractall(SAVED_MODEL_PATH)

    model = hub.load(SAVED_MODEL_PATH).signatures['serving_default']
    return model


def load_saved_model():
    """
    load the saved model from SAVED_MODEL_PATH
    """
    if not os.path.exists(SAVED_MODEL_PATH):
        raise FileNotFoundError(f"Model not found at {SAVED_MODEL_PATH}")
    model = hub.load(SAVED_MODEL_PATH).signatures['serving_default']
    return model


def print_verbose_only(*args, **kwargs):
    """
    print only if verbose==True
    """
    if parser_args.verbose:
        print(*args, **kwargs)


def movenet(input_image, model):
    """
    movenet model:
    Gets input image and outputs array of keypoints with certainty score
    downloaded from #https://tfhub.dev/google/movenet/multipose/lightning/1
    """
    # SavedModel format expects tensor type of int32.
    input_image = tf.cast(input_image, dtype=tf.int32)
    outputs = model(input_image)  # Output is a [1, 6, 56] tensor.

    # The first 17 * 3 elements are the keypoint locations and scores in the
    # format: [y_0, x_0, s_0, y_1, x_1, s_1, …, y_16, x_16, s_16], where y_i,
    # x_i, s_i are the yx-coordinates (normalized to image frame, e.g. range
    # in [0.0, 1.0]) and confidence scores of the i-th joint correspondingly.
    # The order of the 17 keypoint joints is: [nose, left eye, right eye, left
    # ear, right ear, left shoulder, right shoulder, left elbow, right elbow,
    # left wrist, right wrist, left hip, right hip, left knee, right knee,
    # left ankle, right ankle]. The remaining 5 elements [ymin, xmin, ymax,
    # xmax, score] represent the region of the bounding box (in normalized
    # coordinates) and the confidence score of the instance
    keypoints = outputs['output_0'].numpy()

    count_of_people = np.sum(keypoints[0, :, -1] > people_detection_threshold )
    # print_verbose_only("count_of_people", count_of_people)

    # there are 6 people
    # there are 17 body points and therefore 3*17=51 numbers per person
    return keypoints[:, :, :51].reshape((6, 17, 3))[0]


def is_there_person(points):
    """
    return True/False of whether there is a person
    """
    visible_joints = np.sum(points[:, -1] > point_detection_threshold)
    return visible_joints >= 3


def is_person_facing_camera(points):
    """
    return True/False depending on if the person is facing camera or not
    """
    LEFT_EYE = 1
    NOSE = 0
    RIGHT_EYE = 2
    visible_joints_face = np.sum(points[:5, -1] > point_detection_threshold)
    facing_forward = points[LEFT_EYE][1] > points[NOSE][1] > points[RIGHT_EYE][1]
    return visible_joints_face >= 3 and facing_forward


def get_heading_angle(
        points,
        fov=90,
        image_width=1,
        offset=0,
        scaling=1):
    """
    get the heading angle from the camera's perspective to the person,
     in degree, relative to the center of the field of view
    """
    visible_points = points[points[:, -1] > point_detection_threshold]
    x_mean = np.mean(visible_points[:, 1])
    x_angle_radian = math.atan(
        (x_mean - (image_width / 2)) / (image_width / 2) * math.tan(math.radians(fov / 2)))
    return offset + scaling * math.degrees(x_angle_radian)


def get_x_y_coord(
        points,
    ):
    visible_points = points[points[:, -1] > point_detection_threshold]
    x_mean = np.mean(visible_points[:, 1])
    y_mean = np.mean(visible_points[:, 0])
    return x_mean * image_width, y_mean * image_height


def process_frame(image, person_array):
    """
    process a frame. Determine keypoints and number of people and
    heading angle.
    """

    input_size = 256
    input_image = tf.expand_dims(image, axis=0)
    input_image = tf.image.resize_with_pad(input_image, input_size, input_size)

    # Run model inference
    keypoints = movenet(input_image, model)

    person_array[0].heading_angle = get_heading_angle(keypoints)
    person_array[0].x, person_array[0].y = get_x_y_coord(keypoints)
    person_array[0].on_screen =  is_there_person(keypoints) 

    is_there_anyone = is_there_person(keypoints)
    return is_there_anyone


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

class LidarCameraSubscriber(Node):
    def __init__(self):
        #make array of 6 person
        self.person_array = [Person() for _ in range(6)]
        self.is_there_anyone = False


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
            'velodyne_points',  # Change this to the topic you're subscribing to
            self.lidar_callback,
            10)
        self.lidar_subscription

        #topics to publish
        self.is_person_publisher = self.create_publisher(
            Bool,
            'is_person_topic',
            10)
        # self.angle_publisher = self.create_publisher(
        #     Float32,
        #     'angle_topic',
        #     10)
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            'pose_stamped_topic', 
            10)

    def camera_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        self.is_there_anyone = process_frame(cv_image,self.person_array)
        self.publish_message("camera",msg.header.stamp)

    def lidar_callback(self, msg):
        if self.is_there_anyone:
            # Deserialize PointCloud2 data into xyz points
            point_gen = pc2.read_points(
                msg, field_names=(
                    "x", "y", "z"), skip_nans=True)
            # points = np.array(list(point_gen))
            points = [[x, y, z] for x, y, z in point_gen]
            points = np.array(points)
            points2d = convert_to_camera_frame(points)

        #update depth for every person
        for person in self.person_array:
            if not person.on_screen:
                person.z = -1.0
            else:
                person.z = estimate_depth(person.x, person.y, points2d)

        self.publish_message("lidar",msg.header.stamp)

    def publish_message(self,source_str, timestamp):
        #person0 for debugging purpse
        person0 = self.person_array[0]
        message = f"{source_str:<7}"
        message += f" person: {'YES' if self.is_there_anyone else 'NO '}"
        message += f" angle: {person0.heading_angle:<20}"
        message += f"person_coordinate: {person0.x:<22} {person0.y:<22} {person0.z:22}"
        message+=f" {temp}"
        print_verbose_only(message)

        # Publish the message
        is_person_msg = Bool()
        is_person_msg.data = bool(self.is_there_anyone)
        self.is_person_publisher.publish(is_person_msg)

        # angle_msg = Float32()
        # angle_msg.data = angle
        # self.angle_publisher.publish(angle_msg)

        pose_stamped_msg = PoseStamped()
        pose_stamped_msg.header.stamp = timestamp
        lidar_x,lidar_y,lidar_z = convert_to_lidar_frame((person0.x,person0.y,person0.z))
        pose_stamped_msg.pose.position.x = lidar_x  
        pose_stamped_msg.pose.position.y = lidar_y
        pose_stamped_msg.pose.position.z = lidar_z
        self.pose_publisher.publish(pose_stamped_msg)

def read_space_seperated_matrix(string):
    """
    convert space seperated matrix string to np matrix
    """
    lines = string.strip().split('\n')
    matrix = []
    for line in lines:
        values = line.split()  # Exclude the first element 'rotation_matrix'
        matrix.append([float(value) for value in values])
    numpy_matrix = np.array(matrix)
    return numpy_matrix


def parse_global_matrix():
    global rotation_matrix, translation_vector, camera_transformation_k
    camera_transformation_k = read_space_seperated_matrix(camera_transformation_k)
    rotation_matrix = read_space_seperated_matrix(rotation_matrix).T

    global inverse_camera_transformation_k, inverse_rotation_matrix
    inverse_camera_transformation_k = np.linalg.inv(camera_transformation_k)
    inverse_rotation_matrix = np.linalg.inv(rotation_matrix)


def convert_to_lidar_frame(uv_coordinate):
    """
    convert 2d camera coordinate + depth into 3d lidar frame
    """
    point_cloud = np.empty( (3,) , dtype=float)
    point_cloud[2] = uv_coordinate[2]
    point_cloud[1] = ( image_height - uv_coordinate[1] )*point_cloud[2]
    point_cloud[0] = uv_coordinate[0]*point_cloud[2]

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

    # Find the indices of the k nearest poitns
    k = 5     # Number of nearest neighbors we want
    closest_indices = np.argpartition(distances_sq, k)[:k]
    global temp
    temp =distances_sq[closest_indices]
    # Get the depth value of the closest point
    closest_depths = np_2d_array[2,closest_indices]
    return np.mean(closest_depths)


def main(args=None):
    global parser_args
    global model 

    parse_global_matrix()
    parser_args = parse_arguments()
    parser_args.verbose = True

    if parser_args.download_model:
        print('downloading model...')
        model = download_model()
    else:
        model = load_saved_model()
    print("Human detection ready...")
    rclpy.init(args=args)
    subscriber = LidarCameraSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("\n\nDEBUG MODE ON1\n\n")
    main()

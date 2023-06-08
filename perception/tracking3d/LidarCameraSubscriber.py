import argparse
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
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

MODEL_URL = "https://tfhub.dev/google/movenet/multipose/lightning/1?tf-hub-format=compressed"
SAVED_MODEL_PATH = "./multipose_model"
parser_args = tuple()

xy = 1280, 1024
is_there_person_bool = False
angle = 0.0


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

    threshold = 0.3
    count_of_people = np.sum(keypoints[0, :, -1] > threshold)
    print_verbose_only("count_of_people", count_of_people)

    # there are 6 people
    # there are 17 body points and therefore 3*17=51 numbers per person
    return keypoints[:, :, :51].reshape((6, 17, 3))[0]


def is_there_person(points, score_threshold=0.3):
    """
    return True/False of whether there is a person
    """
    visible_joints = np.sum(points[:, 2] > score_threshold)
    return visible_joints >= 3


def is_person_facing_camera(points, score_threshold=0.3):
    """
    return True/False depending on if the person is facing camera or not
    """
    LEFT_EYE = 1
    NOSE = 0
    RIGHT_EYE = 2
    visible_joints_face = np.sum(points[:5, 2] > score_threshold)
    facing_forward = points[LEFT_EYE][1] > points[NOSE][1] > points[RIGHT_EYE][1]
    return visible_joints_face >= 3 and facing_forward


def get_heading_angle(
        points,
        score_threshold=0.3,
        fov=90,
        image_width=1,
        offset=0,
        scaling=1):
    """
    get the heading angle of the person, in degree, relative to the center
    of the field of view
    """
    visible_points = points[points[:, 2] > score_threshold]
    x_mean = np.mean(visible_points[:, 1])
    x_angle_radian = math.atan(
        (x_mean - (image_width / 2)) / (image_width / 2) * math.tan(math.radians(fov / 2)))
    return offset + scaling * math.degrees(x_angle_radian)


def get_x_y_coord(
        points,
        score_threshold=0.3,
        image_width=1280,
        image_height=1024):
    visible_points = points[points[:, 2] > score_threshold]
    x_mean = np.mean(visible_points[:, 1])
    y_mean = np.mean(visible_points[:, 0])
    return x_mean * image_width, y_mean * image_height


def process_frame(image):
    """
    process a frame. Determine keypoints and number of people and
    heading angle.
    """
    global xy
    global is_there_person_bool
    global angle

    input_size = 256
    input_image = tf.expand_dims(image, axis=0)
    input_image = tf.image.resize_with_pad(input_image, input_size, input_size)

    # Run model inference
    keypoints = movenet(input_image, model)

    xy = get_x_y_coord(keypoints)
    angle = get_heading_angle(keypoints)
    is_there_person_bool = is_there_person(keypoints)
    log = ""
    log += f'is there person: {is_there_person_bool}\n'
    log += f"Person {'is' if is_person_facing_camera(keypoints) else 'NOT'} facing laptop!\n"
    log += f"heading angle: {angle}\n"

    if abs(angle) < 10:
        log += "CENTER\n"
    elif angle < 0:
        log += "LEFT\n"
    else:
        log += "RIGHT\n"
    print_verbose_only(log)


class LidarCameraSubscriber(Node):
    def __init__(self):
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

        self.is_person_publisher = self.create_publisher(
            Bool,
            'is_person_topic',
            10)
        self.angle_publisher = self.create_publisher(
            Float32,
            'angle_topic',
            10)
        self.depth_publisher = self.create_publisher(
            Float32,
            'depth_topic',
            10)
        self.xy_publisher = self.create_publisher(
            Point,
            'xy_topic',
            10)

    def camera_callback(self, msg):
        timestamp = msg.header.stamp
        cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        process_frame(cv_image)
        message = f"{'camera '}"
        message += f" person: {'YES' if is_there_person_bool else 'NO '}"
        message += f" angle: {angle:<20}"
        message += f" depth: {closest_depth:<20}"
        message += f"xy: {xy[0]:<10}{xy[1]:<10}"
        print_verbose_only(message)

        # Publish the message
        is_person_msg = Bool()
        is_person_msg.data = is_there_person_bool
        self.is_person_publisher.publish(is_person_msg)

        angle_msg = Float32()
        angle_msg.data = angle
        self.angle_publisher.publish(angle_msg)

        depth_msg = Float32()
        depth_msg.data = closest_depth
        self.depth_publisher.publish(depth_msg)

        xy_msg = Point()
        xy_msg.x = xy[0]
        xy_msg.y = xy[1]
        self.xy_publisher.publish(xy_msg)

    def lidar_callback(self, msg):
        global closest_depth

        timestamp = msg.header.stamp

        # Deserialize PointCloud2 data into a generator of (x, y, z) points
        point_gen = pc2.read_points(
            msg, field_names=(
                "x", "y", "z"), skip_nans=True)

        points = [[x, y, z] for x, y, z in point_gen]

        if is_there_person_bool:
            points = np.array(points)
            test_x, test_y = xy
            closest_depth = find_closest_point_depth(
                test_x, test_y, convert_to_2d(points))
        else:
            closest_depth = -1.0
        message = f"{'lidar  '}"
        message += f" person: {'YES' if is_there_person_bool else 'NO '}"
        message += f" angle: {angle:<20}"
        message += f" depth: {closest_depth:<20}"
        message += f"xy: {xy[0]:<10}{xy[1]:<10}"
        print_verbose_only(message)

        # Publish the message
        is_person_msg = Bool()
        is_person_msg.data = is_there_person_bool
        self.is_person_publisher.publish(is_person_msg)

        angle_msg = Float32()
        angle_msg.data = angle
        self.angle_publisher.publish(angle_msg)

        depth_msg = Float32()
        depth_msg.data = closest_depth
        self.depth_publisher.publish(depth_msg)

        xy_msg = Point()
        xy_msg.x = xy[0]
        xy_msg.y = xy[1]
        self.xy_publisher.publish(xy_msg)


def read_matrix(string):
    """
    convert string matrix to np matrix
    """
    lines = string.strip().split('\n')
    matrix = []
    for line in lines:
        values = line.split()  # Exclude the first element 'Rotation'
        matrix.append([float(value) for value in values])
    numpy_matrix = np.array(matrix)
    return numpy_matrix


def convert_to_2d(point_cloud, image_height=1024):
    """
    convert 3d lidar data into 2d coordinate
    """

    length = point_cloud.shape[0]

    rotation = read_matrix("""
    -0.007495781893 -0.0006277316155    0.9999717092
    -0.9999516401   -0.006361853422 -0.007499625104
    0.006366381192  -0.9999795662   -0.0005800141927
    """)
    translation = np.array([-0.06024059837, -0.08180891509, -0.3117851288])

    rotation = rotation.T
    translation = np.tile(translation, (length, 1)).T

    k = np.array([
        [628.5359544, 0, 676.9575694],
        [0, 627.7249542, 532.7206716],
        [0, 0, 1],
    ])

    point_cloud = point_cloud.T
    point_cloud = np.dot(rotation, point_cloud) + translation
    point_cloud = np.dot(k, point_cloud)

    uv_coordinate = np.empty_like(point_cloud)

    """
    uv = [x/z, y/z, z], and y is opposite so the minus imageheight
    """
    uv_coordinate[0] = point_cloud[0] / point_cloud[2]
    uv_coordinate[1] = image_height - point_cloud[1] / point_cloud[2]
    uv_coordinate[2] = point_cloud[2]

    filtered_uv_coordinate = uv_coordinate[:, uv_coordinate[2, :] >= 0]
    return filtered_uv_coordinate


def find_closest_point_depth(x, y, np_2d_array):
    """
    find the point closest to x,y from a 2d np array
    """
    # Calculate the Euclidean distance between each point and the target
    # coordinates (x, y)
    distances_sq = (np_2d_array[0, :] - x) ** 2 + (np_2d_array[1, :] - y) ** 2
    # Find the index of the point with the minimum distance
    closest_index = np.argmin(distances_sq)
    # Retrieve the depth value of the closest point
    closest_depth = np_2d_array[2, closest_index]
    return closest_depth


def main(args=None):
    print("ready...")
    rclpy.init(args=args)
    subscriber = LidarCameraSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()
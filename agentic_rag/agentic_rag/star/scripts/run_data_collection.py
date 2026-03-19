import argparse
from copyreg import pickle
import os
import time
import multiprocessing
import subprocess
import threading
from typing import List

import pickle
import copy
from queue import Queue
from datetime import datetime
import tf_transformations
import numpy as np
import sys
import cv2
from PIL import Image as PILImage

# load this directory
sys.path.append(sys.path[0] + '/..')
sys.path.append("/home/trailbot/RAG/remembr/")
# This forces Python to load utils from your intended folder
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, project_root)
# from utils.util import get_frames

# from scipy.spatial.transform import Rotation
import json

from pathlib import Path
from omegaconf import DictConfig

# import sys
import hydra
import numpy
import numpy.core
import numpy.core.numeric
from rclpy.node import Node
import tf2_ros
from tf2_msgs.msg import TFMessage
# import tf_transformations

import message_filters
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import TransformStamped, PoseStamped
from cv_bridge import CvBridge

import rclpy
# from utils.utils import get_observation
# TODO: Move to this file later
# from OpenNav_issacsim import run_scenegraph_generation
from agentic_rag.star.captioners.captioner import CaptionManager
from scipy.spatial.transform import Rotation as R
from scenegraph.scenegraph_constructor import run_scenegraph_generation

# ✅ Patch BOTH the _core and numeric module directly
numpy._core = numpy.core
numpy._core.numeric = numpy.core.numeric

# 🔧 Force into sys.modules
sys.modules['numpy._core'] = numpy.core
sys.modules['numpy._core.numeric'] = numpy.core.numeric

SAVE_EVERY_N: int = 3 # Save every N seconds
MAX_RETRIES: int = 2
DEFAULT_QUERY: str = "You are a mobile robot navigating a warehouse. From your egocentric point of view, describe the current scene and your own behavior in as much detail as possible. Think step by step. \n\
    Focus on:\n\
        1. Your motion and activity: Are you moving forward, turning, stopped, or interacting with something? Did you just enter or exit a building, turn at a corner, or wait for someone?\n\
        2. People around you: Describe each person you see. What are they doing? Are they walking, waiting, helping, opening a door, riding a bike? Mention their appearance (e.g., clothing color, backpack, item in hand) and their relative location to you.\n\
        3. Visible objects and infrastructure: Carefully describe notable objects or structures like trash bins, sculptures, signs, poles, benches, vehicles, crosswalks, cones, bollards, elevators, etc. Include their color, material, size, and approximate position relative to you (e.g., in front, behind, to the left/right).\n\
        4. Environmental features: Describe the physical space. Are you indoors or outdoors? What is the surface type (brick, grass, tile, concrete)? Is the area narrow or spacious, crowded or empty, shaded or sunny?\n\
        5. Events or activities: Mention any dynamic actions or interactions — such as doors opening, vehicles driving by, stopping at intersections, etc.\n\
        6. Interesting or unique details: Note anything visually distinct or surprising (e.g., a large sculpture, colorful banner, rainbow flag, food truck, or an emergency phone pole).\n\
        Be very specific and descriptive. Imagine someone blind is listening and needs to visualize the entire moment. Focus on relevant and grounded details that could be remembered and queried later."

def process_cfg(cfg: DictConfig):
    cfg.basedir = Path(cfg.basedir)
    cfg.save_vis_path = Path(cfg.save_vis_path)
    cfg.save_cap_path = Path(cfg.save_cap_path)
    cfg.start_timestamp = time.time()
    time_struct = time.localtime(cfg.start_timestamp)
    cfg.sequence = time.strftime("%Y-%m-%d %H:%M:%S", time_struct)

    return cfg

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

def preprocess_position_matrix(position_matrix: np.ndarray) -> tuple:
    # x: horizontal, y: vertical, z: depth
    position: np.ndarray = position_matrix[:3, 3]
    R: np.ndarray = position_matrix[:3, :3]
    theta: float = np.arcsin(R[2, 0]) # radian
    return position, theta

def to_seconds(t: rclpy.time.Time) -> float:
    return t.sec + t.nanosec * 1e-9

class ObservationHub(Node):
    def __init__(
        self,
        cfg: DictConfig,
        observation_buffer: Queue,
        sync_queue_size: int = 1000,
        max_time_diff: float = 0.05
    ) -> None:
        super().__init__('captioner_node')
        self.cfg: DictConfig = cfg
        self.observation_buffer: List[Queue] = observation_buffer
        self.radial_distortion = np.array([0.0, 0.0, 0, 0])
        self.observation_ready = False
        self.memory_buffer_captioner = []  # Store a sequence of observations
        self.memory_buffer_sg = []

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        camera_transformation_k = cfg.camera_transformation_k
        self.camera_transformation_k = read_space_separated_matrix(camera_transformation_k)

        self.image_height = cfg['image_height']
        self.image_width = cfg['image_width']

        self.new_K, _ = cv2.getOptimalNewCameraMatrix(self.camera_transformation_k, self.radial_distortion, (self.image_width, self.image_height), 1, (self.image_width, self.image_height))

        print("Mode: Lidar and Camera")
        # create publishers
        self.tf_subscriber = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        self.base_link_publisher = self.create_publisher(
            TransformStamped,
            'base_link_transform',
            10
        )
        # create subscribers
        self.image_sub = message_filters.Subscriber(self, Image, self.cfg.rgb_cam_topic)
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, self.cfg.lidar_topic)
        self.lidarpos_sub = message_filters.Subscriber(self, TransformStamped, 'base_link_transform')

        # Caches for manual sync
        self.image_cache = message_filters.Cache(self.image_sub, 100)
        self.pose_cache = message_filters.Cache(self.lidarpos_sub, 100)
        self.scan_duration_sec = 0.1

        # create callback
        ts = message_filters.ApproximateTimeSynchronizer(
            [self.image_sub, self.lidar_sub, self.lidarpos_sub],
            sync_queue_size, max_time_diff
        )
        ts.registerCallback(self.observation_callback)

        self.timestamp, self.callback_times = 0, 0
        self.cv_image = None
        self.first_pose = None

        # Start the data preprocessing thread
        self.livedata_preprocessing = threading.Thread(target=self.run_livedata_processing)
        self.livedata_preprocessing.start()

    # TODO: We may need to use some kind of memory buffer
    def observation_callback(self, camera_msg: Image, lidar_msg: PointCloud2, tf_msg: PoseStamped) -> None:
        """
        Callback function for processing camera and lidar messages and pose message.

        Args:
            camera_msg (sensor_msgs.msg.Image): Camera message.
            lidar_msg (sensor_msgs.msg.PointCloud2): Lidar message.
            lidarpos_msg (geometry_msgs.msg.PoseStamped): Lidar pose message.

        """
        # print("Received synchronized messages")

        cv_image = self.bridge.imgmsg_to_cv2(
            camera_msg, desired_encoding='rgb8') # rgb8
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) #Added for colour correction to RGB
        self.cv_image = cv_image
        # self.cv_image = cv2.undistort(cv_image, self.camera_transformation_k, self.radial_distortion, None, self.new_K)
        self.timestamp_cam = camera_msg.header.stamp

        # Deserialize PointCloud2 data into xyz points
        point_gen = pc2.read_points(lidar_msg, field_names=("x", "y", "z"), skip_nans=True)
        # Filter out points with all 0s
        # self.points = np.array([[x, y, z, 1] for x, y, z in point_gen if any([i!=0 for i in [x,y,z]])])

        points = np.array([point_gen['x'], point_gen['y'], point_gen['z']]).T
        mask = ~np.all(points == 0, axis=1)
        # Apply the mask to filter out rows with all zero coordinates
        self.points = points[mask]

        self.timestamp_lidar = lidar_msg.header.stamp

        # get the transformation matrix of the lidar
        translation = tf_msg.transform.translation
        rotation = tf_msg.transform.rotation
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        rotation_matrix = tf_transformations.quaternion_matrix(quaternion)
        translation = [translation.x, translation.y, translation.z]
        translation_matrix = tf_transformations.translation_matrix(translation)
        self.transformation_matrix = np.dot(translation_matrix, rotation_matrix)

        self.observation_ready = True
        self.callback_times += 1

    def tf_callback(self, msg):
        # Loop through each transform in the TFMessage
        for transform in msg.transforms:
            # Check if the child frame is base_link
            if transform.child_frame_id == 'base_link':
                # Extract the relevant transform and publish it
                self.publish_base_link_transform(transform)

    def publish_base_link_transform(self, transform):
        # Prepare the TransformStamped message
        TF_msg = TransformStamped()
        # get the time of the transform
        TF_msg.header.stamp = transform.header.stamp
        TF_msg.header.frame_id = 'map'
        TF_msg.transform.translation.x = transform.transform.translation.x
        TF_msg.transform.translation.y = transform.transform.translation.y
        TF_msg.transform.translation.z = transform.transform.translation.z
        TF_msg.transform.rotation.x = transform.transform.rotation.x
        TF_msg.transform.rotation.y = transform.transform.rotation.y
        TF_msg.transform.rotation.z = transform.transform.rotation.z
        TF_msg.transform.rotation.w = transform.transform.rotation.w

        # Publish the base_link transform
        self.base_link_publisher.publish(TF_msg)

    def run_livedata_processing(self):
        self.get_logger().info("Initialized live data processing...")
        first_transformation_matrix = None
        # for Isaac Sim
        # T_baselink_to_lidar = np.array([
        #                     [ 1,  0,  0,  0],  # Flip the X-axis
        #                     [ 0,  1,  0,  0],  # Flip the Y-axis
        #                     [ 0,  0,  1,  0], # 0.6019407510757446],  # Z-axis remains the same
        #                     [ 0,  0,  0,  1]   # Homogeneous coordinate
        #                 ])
        # For Husky Hardware
        T_baselink_to_lidar = np.array([
                            [ 0, -1,  0,  0],  # Flip the X-axis
                            [ 1,  0,  0,  0],  # Flip the Y-axis
                            [ 0,  0,  1,  0],  # Z-axis remains the same
                            [ 0,  0,  0,  1]   # Homogeneous coordinate
                        ])

        T_lidar_to_baselink = np.linalg.inv(T_baselink_to_lidar)
        pre_callback_times = 0
        last_scenegraph_emit_ts = None
        last_captioner_emit_ts = None
        offset_time = 0.0

        while rclpy.ok():
            if self.observation_ready:
                observation = (
                    self.callback_times, # idx 0
                    self.cv_image, # color 1
                    self.points, # point_cloud 2
                    self.transformation_matrix, # pose 3
                    to_seconds(self.timestamp_cam), # timestamp 4
                )

                if observation[0] == pre_callback_times: # observation[2] - pre_timestamp > 0.2: #  and
                    continue

                if first_transformation_matrix is None:
                    first_transformation_matrix = observation[3]
                    first_transformation_matrix_inv = np.linalg.inv(first_transformation_matrix)
                    # self.get_logger().info(f"First transformation matrix: {first_transformation_matrix}")
                now_ts = observation[4]
                if last_scenegraph_emit_ts is None:
                    last_scenegraph_emit_ts = now_ts
                    offset_time = now_ts
                if last_captioner_emit_ts is None:
                    last_captioner_emit_ts = now_ts

                base_link_2map_TF = np.dot(first_transformation_matrix_inv, observation[3])
                # self.get_logger().info(f"self.transformation_matrix: {self.transformation_matrix}")
                lidar_2map_TF = base_link_2map_TF @ T_lidar_to_baselink

                if now_ts - last_captioner_emit_ts >= 1.0: # self.cfg.caption_freq:
                    data_captioner = self.memory_buffer_captioner
                    self.observation_buffer['captioner'].put(data_captioner.copy())
                    self.memory_buffer_captioner = []
                    last_captioner_emit_ts = now_ts
                # if now_ts - last_scenegraph_emit_ts >= 1.0:
                    data_scenegraph = self.memory_buffer_sg
                    self.observation_buffer['scenegraph'].put(data_scenegraph.copy())
                    self.memory_buffer_sg = []
                    # last_scenegraph_emit_ts = now_ts

                timestamp = observation[4] - offset_time + self.cfg.start_timestamp
                self.memory_buffer_sg.append((
                    copy.deepcopy(observation[0]),
                    copy.deepcopy(observation[1]),
                    copy.deepcopy(observation[2]),
                    copy.deepcopy(lidar_2map_TF),
                    copy.deepcopy(timestamp),
                ))

                self.memory_buffer_captioner.append((
                    copy.deepcopy(observation[0]),
                    copy.deepcopy(observation[1]),
                    copy.deepcopy(timestamp),
                    copy.deepcopy(lidar_2map_TF)
                ))

                pre_callback_times = observation[0]

                # if (now_ts - last_captioner_emit_ts) >= 3.0: # self.memory_buffer_captioner[-1][2] - self.memory_buffer_captioner[0][2] > self.cfg.caption_freq:
                #     data = self.memory_buffer_captioner
                #     self.observation_buffer['captioner'].put(data.copy())
                #     self.memory_buffer_captioner = []
                #     last_captioner_emit_ts = now_ts

                # if (now_ts - last_scenegraph_emit_ts) >= 1.0: # and len(self.memory_buffer_sg) > 0:
                #     image, pc, pose, his_pcs, his_poses = get_observation(self.cfg.stride, self.memory_buffer_sg)
                #     data = (image, pc, pose, his_pcs, his_poses, observation[2])
                #     self.observation_buffer['scenegraph'].put(data.copy())
                #     self.memory_buffer_sg = []
                #     last_scenegraph_emit_ts = now_ts

            # print(f"{time.time()-t1} seconds for processing one group of data")
                #print(f"Trajectory: {trajectory}")



# def start_ros_node(observation_buffer, args, cfg):
#     print("Starting ROS2 node...")
#     rclpy.init(args=args)
#     subscriber = ObservationHub(cfg, observation_buffer)
#     subscriber.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, False)])
#     rclpy.spin(subscriber)
#     subscriber.destroy_node()
#     rclpy.shutdown()

def start_ros_node(observation_buffer, args, cfg):
    print("Starting ROS2 node...")
    rclpy.init(args=args)
    subscriber = ObservationHub(cfg, observation_buffer)
    subscriber.set_parameters([
        rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, False)
    ])

    try:
        while rclpy.ok():
            rclpy.spin_once(subscriber, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Ctrl-C received, shutting down ros2 node...")
    finally:
        subscriber.destroy_node()
        rclpy.shutdown()

def run_shell_command(command):
    with open(os.devnull, 'w') as nullfile:
        process = subprocess.Popen(command, shell=True, stdout=nullfile, stderr=subprocess.STDOUT)
        process.communicate()

def save_observations(observation_buffer, cfg, loop_event):
    data_in_timewindow = {
        'position': [],
        'rotation': [],
        'timestamps': [],
        'images': []
    }

    timewindow_counter = 0
    timewindow = []
    while not loop_event.is_set():
        if not observation_buffer.empty():
            print("Accessing observation buffer...")
            data = observation_buffer.get()
            for observation in data:
                _, image, timestamp, pose = observation
                position, rotation = preprocess_position_matrix(pose)

                data_in_timewindow['position'].append(position)
                data_in_timewindow['rotation'].append(rotation)
                data_in_timewindow['timestamps'].append(timestamp)
                data_in_timewindow['images'].append(image)

            if not os.path.exists(f"/home/trailbot/RAG/results/{cfg.sequence}"):
                os.makedirs(f"/home/trailbot/RAG/results/{cfg.sequence}/processed")
            with open(f"/home/trailbot/RAG/results/{cfg.sequence}/processed/{data_in_timewindow['timestamps'][0]}.pkl", 'wb') as f:
                pickle.dump(data_in_timewindow, f, protocol=pickle.HIGHEST_PROTOCOL)
                print("Data saved successfully.")

            data_in_timewindow = {
                'position': [],
                'rotation': [],
                'timestamps': [],
                'images': []
            }
        else:
            time.sleep(0.1)
            continue

def run_online_captioning(observation_buffer, captioner, args, loop_event):
    caption_manager = CaptionManager(args, captioner)
    data_in_timewindow = {
        'position': [],
        'rotation': [],
        'timestamps': [],
        'images': []
    }

    while not loop_event.is_set():
        if not observation_buffer.empty():
            print("Accessing observation buffer...")
            data = observation_buffer.get()
            for observation in data:
                _, image, timestamp, pose = observation
                # print("POSE type:", type(pose), pose.dtype if isinstance(pose, np.ndarray) else None)

                position, rotation = preprocess_position_matrix(pose)
                image = PILImage.fromarray(image) # Convert to PIL Image

                data_in_timewindow['position'].append(position)
                data_in_timewindow['rotation'].append(rotation)
                data_in_timewindow['timestamps'].append(timestamp)
                data_in_timewindow['images'].append(image)

            caption_manager.caption_video(data_in_timewindow, query=DEFAULT_QUERY, max_retries=MAX_RETRIES)
            data_in_timewindow = {
                'position': [],
                'rotation': [],
                'timestamps': [],
                'images': []
            }
        else:
            time.sleep(0.1)
            continue

        caption_manager.save_caption_data()

@hydra.main(version_base=None, config_path="../configs", config_name="config")
def main(cfg : DictConfig):
    sg_cfg = cfg['scenegraph']
    # seq = cfg.sequence

    parser = argparse.ArgumentParser()
    parser.add_argument("--seq_id", type=str, default='05')
    parser.add_argument("--out_path", type=str, default=f"./data/captions/{4}/captions")
    parser.add_argument("--data_path", type=str, default="./data/semantickitti_data") # coda_data
    parser.add_argument("--seconds_per_caption", type=int, default=3)

    parser.add_argument("--num-video-frames", type=int, default=15)
    parser.add_argument("--captioner_name", type=str, default="NVILA-Lite-2B")

    parser.add_argument("--model-path", type=str, default="Efficient-Large-Model/NVILA-Lite-2B")
    parser.add_argument("--conv-mode", "-c", type=str, default="auto")
    parser.add_argument("--query", type=str, default=DEFAULT_QUERY)
    parser.add_argument("--text", type=str)
    # parser.add_argument("--media", type=str, nargs="+", default= [f"{base_path}{img}{200}.png", f"{base_path}{img}{400}.png", f"{base_path}{img}{800}.png"])#["/home/mfyuan/local_folder/OpenNav_v2/deps1/VILA/demo_images/cam0.png"])
    parser.add_argument("--media", type=str, default=["/home/mfyuan/local_folder/OpenNav_v2/deps1/VILA/demo_images/output.mp4"])#/media/ssd/Local_data/CODa_dataset/videos/0/output.mp4
    parser.add_argument("--json-mode", action="store_true")
    parser.add_argument("--json-schema", type=str, default=None)
    args = parser.parse_args()
    # captioner: NVILACaptioner = NVILACaptioner(args)

    debug_mode = False
    # sg_cfg.sequence = seq
    sg_cfg = process_cfg(sg_cfg)
    buffer_names = ['captioner', 'scenegraph']
    if sg_cfg.observation_buffer_size > 0:
        observation_buffer = {name: multiprocessing.Queue(maxsize=sg_cfg.observation_buffer_size) for name in buffer_names}
    else:
        observation_buffer = {name: multiprocessing.Queue() for name in buffer_names}
    prepare_event = multiprocessing.Event()
    loop_event = multiprocessing.Event()

    sg_process = multiprocessing.Process(
        target=run_scenegraph_generation,
        args=(
            sg_cfg,
            observation_buffer['scenegraph'],
            prepare_event,
            loop_event
        )
    )
    sg_process.start()
    while not prepare_event.is_set():
        time.sleep(0.5)
    prepare_event.clear()

    ros_process = multiprocessing.Process(target=start_ros_node, args=(observation_buffer, None, sg_cfg))
    # p = multiprocessing.Process(target=run_gradio_gui)
    ros_process.start()
    print("Processes started")

    try:
        # run_online_captioning(observation_buffer['captioner'], None, sg_cfg, loop_event)
        save_observations(observation_buffer['captioner'], sg_cfg, loop_event)
    except Exception as e:
        print(f"Error occurred: {e}")
    except KeyboardInterrupt:
        print("KeyboardInterrupt received, shutting down main thread...")
    finally:
        loop_event.set()


if __name__ == '__main__':
    multiprocessing.set_start_method("spawn", force=True) # Important!

    # embedder = HuggingFaceEmbeddings(model_name='mixedbread-ai/mxbai-embed-large-v1')
    # nvila_model = NVILACaptioner(args)
    # image = cv2.imread('rabbit.jpeg')
    # image = PILImage.fromarray(image)
    # query = "Please describe what did you see in the image."
    # out_text: str = nvila_model.caption([image, image, image, image, image], query=query)
    # print(f"Test caption output: {out_text}")

    # main()

    print("\n\nDEBUG MODE ON\n\n")
    command1 = "ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera/compressed --remap out:=/camera"
    command2 = "ros2 bag play /home/trailbot/bags/lab1_2026" #2Indoor9 Outdoor #two_cones Indoor_3d

    # Create threads for each shell command and main function

    thread1 = threading.Thread(target=run_shell_command, args=(command1,))
    thread2 = threading.Thread(target=run_shell_command, args=(command2,))
    # main()
    thread_main = threading.Thread(target=main, args=(None,))

    thread1.start()
    thread2.start()
    thread_main.start()

    #Wait for the threads to finish
    thread1.join()
    thread2.join()
    thread_main.join()
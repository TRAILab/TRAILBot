"""
2024.10.14 

load three large models to get the caption and mask of the image in the dataset
"""

import multiprocessing
import sys
import os
# Add the parent directory to sys.path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

import numpy as np
import cv2
import matplotlib.pyplot as plt
import torch
from pathlib import Path
from typing import List, Dict, Optional, Any
import glob
import gzip
import pickle
from utils.utils import load_models, project, gobs_to_detection_list, denoise_objects, filter_objects, merge_objects, accumulate_pc, distance_filter, show_captions, class_objects, get_observation, load_calib
from utils.merge import compute_spatial_similarities, compute_caption_similarities, compute_ft_similarities, aggregate_similarities, merge_detections_to_objects, caption_merge, captions_ft

from tqdm import trange

from some_class.map_calss import MapObjectList
import open3d as o3d
import hydra
from omegaconf import DictConfig

from sentence_transformers import SentenceTransformer
from datetime import datetime

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray, Detection3D # sudo apt-get install ros-humble-vision-msgs
import argparse
from cv_bridge import CvBridge

# ROS2 related packages
import math

from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Bool, Float32, String

import time
import yaml

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, PoseStamped

import tf_transformations
import tf2_ros
import message_filters
from collections import deque
import threading
import queue

from utils.merge import merge_obj2_into_obj1
import copy
import platform
import multiprocessing
import logging
SHOW_IMAGE_WINDOW = True 

# some common captions of the background: road, sidewalk
# possible captions of the background

# BG_CAPTIONS = ["paved city road", "a long narrow street", "paved city street", \
#     "white lines on the road", "shadows on the street", "a concerte sidewalk", "a shadow on the ground", \
#     "white lines on the street", "brick sidewalk", "a sidewalk next to the street",\
#     "a train boarding platform", "the train tracks", "shadow of fence",\
#     "a sidewalk next to the train tracks", "shadow of bench", "a paved city sidewalk",\
#     "some trees on the both sides of the street", "a street with trees", "a street with trees",\
#     "brick wall in the front", "a brick wall", "a brick wall", "a brick wall",\
#     "there is building in the front", "a building", "a building", "a building"]
# # 背景映射出来的标签
# # 
# BG_CAPTIONS_Pro = ["paved road", "paved road", "paved road",\
#     "paved road", "paved road", "paved road", "paved road", \
#     "paved road", "sidewalk", "sidewalk",\
#     "sidewalk", "sidewalk", "sidewalk", \
#     "sidewalk", "sidewalk", "sidewalk",\
#     "tree", "forest", "forest",
#     "wall", "wall", "wall", "wall",\
#     "building", "building", "building", "building"]
# # 背景映射出来的标签有哪些
# BG_CAPTIONS_Pro_Sim = ["paved road", "sidewalk", "tree", "wall", "building"] 

BG_CAPTIONS = ["some trees on the both sides of the street", "a street with trees", "a street with trees"]
# 背景映射出来的标签
# 
BG_CAPTIONS_Pro = ["tree", "forest", "forest"]
# 背景映射出来的标签有哪些
BG_CAPTIONS_Pro_Sim = ["tree"] 

def process_cfg(cfg: DictConfig):
    '''
    配置文件预处理
    '''
    cfg.basedir = Path(cfg.basedir)
    cfg.save_vis_path = Path(cfg.save_vis_path)
    cfg.save_cap_path = Path(cfg.save_cap_path)
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

def print_verbose_only(parser_args, *args, **kwargs):
    """
    print only if verbose==True
    """
    if parser_args.verbose:
        print(*args, **kwargs)

def clear_terminal():
    if platform.system() == "Windows":
        os.system('cls')  # Windows
    else:
        os.system('clear')  # macOS/Linux





def generate_trajectory(start, end, num_points=100):
    """
    Generate a linear trajectory from start to end.
    
    Parameters:
    - start: Starting position [x, y, z, heading]
    - end: Ending position [x, y, z, heading]
    - num_points: Number of points in the trajectory
    
    Returns:
    - List of trajectory points
    """
    trajectory = []
    for i in range(num_points):
        x = start[0] + (end[0] - start[0]) * i / (num_points - 1)
        y = start[1] + (end[1] - start[1]) * i / (num_points - 1)
        z = start[2] + (end[2] - start[2]) * i / (num_points - 1)
        heading = start[3] + (end[3] - start[3]) * i / (num_points - 1)
        trajectory.append([x, y, z, heading])
    return trajectory

def run_scenegraph_generation(configs, observation_buffer, objects, mask_generator, calib):
    """
    Runs caption and mask generation as soon as new data is available.
    """
    frame_id = 0

    print("Initialized scene graph...")
    sleep_time, index = 0.5, 0
    point_clouds = []
    logger = multiprocessing.log_to_stderr()
    logger.setLevel(logging.INFO)
    while True: # todos
        if not observation_buffer.empty():
            start = time.time()
            image, pc, pose, his_pcs, his_poses = observation_buffer.get()
            #check the image type
 
            #print(f"Processing {frame_id}th frame..., and {observation_buffer.qsize()} frames left in the buffer")
            save_path_vis = Path(os.path.join(configs.save_vis_path, f"vis_{frame_id}"))
            masks_result = mask_generator.generate(image, save_path=save_path_vis, save_vis=configs.save_vis)
            p1 = time.time()
            # the following lines are for semantic mapping
            mos_model = None
            # TODO: debug this part. load the mos model to filter the dynamic objects if needed
            if configs.filter_dynamic:
                import sys
                sys.path.append("/home/mfyuan/local_folder/4DMOS/src/")
                import mos4d.models.models as models_workingbackup
                weights = configs.mos_path
                mos_cfg = torch.load(weights)["hyper_parameters"]
                ckpt = torch.load(weights)
                mos_model = models_workingbackup.MOSNet(mos_cfg)
                mos_model.load_state_dict(ckpt["state_dict"])
                mos_model = mos_model.cuda()
                mos_model.eval()
                mos_model.freeze()
            # process the background as a whole
            if configs.use_bg:
                bg_objects = {c: None for c in BG_CAPTIONS_Pro_Sim}
                # load the SBERT model
                sbert_model = SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2')
                sbert_model = sbert_model.to("cuda")
                # 把这些背景的caption先编码了
                bg_fts = []
                for bg_cation in BG_CAPTIONS:
                    bg_ft = sbert_model.encode(bg_cation, convert_to_tensor=True)
                    bg_ft = bg_ft / bg_ft.norm(dim=-1, keepdim=True)
                    bg_ft = bg_ft.squeeze()
                    bg_fts.append(bg_ft)
            else:
                bg_objects = None
                bg_fts = []
            
            # the first frame of the point cloud is too sparse, and cannot estimate the dynamic target! 
            # Do I really need to skip the first frame?
            if frame_id == 0:
                frame_id += 1
                continue
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            p2 = time.time()
            if his_pcs is not None:
                pc = accumulate_pc(configs, mos_model, pc, pose, his_pcs, his_poses)

            # filter the point cloud that is too far away
            if configs.filter_dis:
                pc = distance_filter(configs.max_depth, pc)
            # get the pixel value and the remaining point cloud after projection, which correspond one by one

            pro_point, pixels,z = project(pc, image, calib)
            if True:
            # Plot the image
                import matplotlib.pyplot as plt
            
                # Extract pixel coordinates
                x_pix = pixels[:, 1]
                y_pix = pixels[:, 0]

                # Plot the image
                plt.figure(figsize=(12, 8))
                plt.imshow(image)
                plt.axis('off')  # Hide axis

                # Overlay the projected points onto the image
                plt.scatter(x_pix, y_pix, c=z, cmap='jet', s=1)

        
                # Show color bar (representing depth, or another feature of the points)
                cbar = plt.colorbar()
                cbar.set_label('Depth')

                # Show the plot
                os.mkdir("/home/trailbot/Documents/data_process_Kitti/results/jult_4_ptp", exist_ok=True)
                plt.savefig(f"/home/trailbot/Documents/data_process_Kitti/results/jult_4_ptp/saved_{frame_id}.png", dpi=300, bbox_inches='tight')
                # plt.show()
            p3 = time.time()
            used_idx = frame_id
            detection_list, bg_list = gobs_to_detection_list(
                cfg = configs,
                image = image,
                pc = pro_point,
                pixels = pixels,
                idx = frame_id,
                gobs = masks_result,
                trans_pose = pose,
                bg_fts = bg_fts,
                BG_CAPTIONS_Pro = BG_CAPTIONS_Pro,
                )
            
            
            # process the background first
            if len(bg_list) > 0:
                for detected_object in bg_list:
                    class_name = detected_object['bg_class']
                    if bg_objects[class_name] is None:
                        bg_objects[class_name] = detected_object
                    else:
                        matched_obj = bg_objects[class_name]
                        matched_det = detected_object
                        bg_objects[class_name] = merge_obj2_into_obj1(configs, matched_obj, matched_det, bg=True, class_name = class_name)
            p4 = time.time()
            # no objects need to be added to the map
            if len(detection_list) == 0:
                frame_id += 1
                continue
            # for the first frame, add all the objects to the map
            if len(objects) == 0:
                for i in range(len(detection_list)):
                    objects.append(detection_list[i])
                # skip the similarity calculation
                frame_id += 1
                continue
            # visualize the map if needed
            if configs.vis_all:
                point_clouds.extend([detection_list[i]["pcd"] for i in range(len(detection_list))])
            
            spatial_sim = compute_spatial_similarities(detection_list, objects)
            caption_sim = compute_caption_similarities(detection_list, objects)
            ft_sim = compute_ft_similarities(detection_list, objects)
            agg_sim = aggregate_similarities(configs, spatial_sim, ft_sim, caption_sim)
            p5 = time.time()
            # DEBUG: similarity judgment
            # DEBUG: 相似性判断
            # debug_sim = np.dstack((spatial_sim, caption_sim,ft_sim,agg_sim))
            # for i in range(debug_sim.shape[0]):
            #     for j in range(debug_sim.shape[1]):
            #         # 只看有重叠的
            #         if (debug_sim[i][j][0]>0):
            #             print(detection_list[i]["caption"], "***VS***",objects[j]["caption"],debug_sim[i][j])

                    # calculate the similarity between the new object and the existing object
            # if the similarity is lower than the threshold, set it to negative infinity
            agg_sim[agg_sim < configs.sim_threshold] = float('-inf')
            # merge according to similarity
            objects = merge_detections_to_objects(configs, detection_list, objects, agg_sim)

            VLT = False
            #clear_terminal()
            from LLT.api import API
            # api = API(configs, objects, used_idx, logger)
            # ee_start_position = api.matrix_to_xyz_yaw(pose)
            # api.detect_object()
            #api.detect_object()
            frame_id += 1
            if configs.vis_all and used_idx % 5 == 0: #observation_buffer.empty() and
                if VLT:
                    from io import StringIO
                    import traceback
                    
                    from LLT.prompts.main_prompt import MAIN_PROMPT
                    from LLT.prompts.error_correction_prompt import ERROR_CORRECTION_PROMPT
                    from LLT.prompts.print_output_prompt import PRINT_OUTPUT_PROMPT
                    from LLT import models_workingbackup
                    from contextlib import redirect_stdout
                    
                    
                    # from config import OK, PROGRESS, FAIL, ENDC
                    # Output
                    OK = "\033[92m"
                    PROGRESS = "\033[93m"
                    FAIL = "\033[91m"
                    ENDC = "\033[0m"

                    
                    api = API(configs, objects, used_idx, logger)
                    ee_start_position = api.matrix_to_xyz_yaw(pose)
                    # api.detect_object()
                    # api.execute_trajectory()
                    #api.task_completed()
                    
                    command = "Go to the left door"#"Help me find a tool that can be used to get onto the roof and avoid stepping on the grass. The trajectory must be smooth, continuous, and collision-free." #"Go to the blue thing on your right front, and avoid stepping on the grass. "#"Smooth U-turns and make sure you are always on the driveable road surfaces"

                    # convert the pose to x, y, z, yaw
                    
                    new_prompt = MAIN_PROMPT.replace("[INSERT EE POSITION]", str(ee_start_position)).replace("[INSERT TASK]", command)
                    messages = []
                    error = False
                    # Path to your image
                    image_path = image #"/home/trailbot/Documents/data_process_Kitti/04/image_2/000001.png"#2508
                    # save the image
                    cv2.imwrite('current.png', image)
                    messages = models_workingbackup.get_chatgpt_output("gpt-4o", new_prompt, messages, "system", command, image_path)
                    # User input
                    try_num = 0
                    logger.info('before the while loop')
                    while not api.completed_task:
                        new_prompt = ""

                        if len(messages[-1]["content"].split("```python")) > 1:
                            
                            code_block = messages[-1]["content"].split("```python")
                            block_number = 0

                            for block in code_block:
                                if len(block.split("```")) > 1:
                                    code = block.split("```")[0]
                                    block_number += 1
                                    try:
                                        f = StringIO()
                                        with redirect_stdout(f):
                                            logger.info('inside the try statement: exec')
                                            exec(code)
                                    except Exception:
                                        error_message = traceback.format_exc()
                                        new_prompt += ERROR_CORRECTION_PROMPT.replace("[INSERT BLOCK NUMBER]", str(block_number)).replace("[INSERT ERROR MESSAGE]", error_message)
                                        new_prompt += "\n"
                                        error = True
                                        logger.info('inside the try statement: error')
                                    else:
                                        s = f.getvalue()
                                        error = False
                                        logger.info('inside the try statement: no error')
                                        if s != "" and len(s) < 2000:
                                            new_prompt += PRINT_OUTPUT_PROMPT.replace("[INSERT PRINT STATEMENT OUTPUT]", s)
                                            new_prompt += "\n"
                                            error = True
                        if error:
                            api.completed_task = False
                        time.sleep(1)
                        try_num += 1
                        if not api.completed_task:
                            if api.failed_task:

                                logger.info(FAIL + "FAILED TASK! Generating summary of the task execution attempt..." + ENDC)

                                new_prompt += TASK_SUMMARY_PROMPT
                                new_prompt += "\n"

                                logger.info(PROGRESS + "Generating ChatGPT output..." + ENDC)
                                messages = models_workingbackup.get_chatgpt_output("gpt-4o", new_prompt, messages, "user", command, image_path)
                            
                                logger.info(OK + "Finished generating ChatGPT output!" + ENDC)

                                logger.info(PROGRESS + "RETRYING TASK..." + ENDC)

                                new_prompt = MAIN_PROMPT.replace("[INSERT EE POSITION]", str(config.ee_start_position)).replace("[INSERT TASK]", command)
                                new_prompt += "\n"
                                new_prompt += TASK_FAILURE_PROMPT.replace("[INSERT TASK SUMMARY]", messages[-1]["content"])

                                messages = []

                                error = False

                                logger.info(PROGRESS + "Generating ChatGPT output..." + ENDC)
                                messages = models_workingbackup.get_chatgpt_output(args.language_model, new_prompt, messages, "system")
                                logger.info(OK + "Finished generating ChatGPT output!" + ENDC)

                                api.failed_task = False

                            else:
                                logger.info(PROGRESS + "Generating ChatGPT output..." + ENDC)
                                messages = models_workingbackup.get_chatgpt_output("gpt-4o", new_prompt, messages, "user", command, image_path)
                                logger.info(OK + "Finished generating ChatGPT output!" + ENDC)
                
                # start_position = [0.014, -0.03, 0.077, -0.007]
                # road_position = [3.821, 0.483, -0.95, 0.0]
                # trajectory = generate_trajectory(start_position, road_position)
                # road_to_door_position = [10.0, 0.0, -0.95, 0.0]
                # trajectory += generate_trajectory(road_position, road_to_door_position)                
                o3d.visualization.draw_geometries(point_clouds)

                
                # if bg_objects is not None:
                #     bg_objects = MapObjectList([_ for _ in bg_objects.values() if _ is not None])
                #     bg_objects = denoise_objects(configs, bg_objects, bg = True)

                # objects = denoise_objects(configs, objects)
                # objects = filter_objects(configs, objects)
                # objects = merge_objects(configs, objects)
                # # show_captions(objects, bg_objects)
                # # 根据最后的结果，融合物体
                # objects, generator = caption_merge(configs, objects)
                # # 最后再计算一下融合的caption的特征
                # if configs.caption_merge_ft:
                #     objects, bg_objects = captions_ft(objects, bg_objects, sbert_model)
                # # show_captions(objects, bg_objects)
                # # 根据最后的结果，分类得到class
                # objects, bg_objects = class_objects(configs, sbert_model, objects, bg_objects, generator)
                # show_captions(objects, bg_objects)
                    
                # print(f"total: {time.time()-start} secs/frame, p1: {p1-start}, p2: {p2-p1}, p3: {p3-p2}, p4: {p4-p3}, p5: {p5-p4}") 
                # print(f"{frame_id}th frame processed, and {observation_buffer.qsize()} frames left in the buffer")
                    # # 保存一个处理的地图
                
        else:
            #print(f"Observation buffer is empty, waiting for new data...")
            time.sleep(0.1)
            if frame_id ==0:
                print("Waiting for the first frame to be processed...")
                # If the first frame is not processed, continue to wait
                continue
            if configs.save_pcd:
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                results = {
                    'objects': objects.to_serializable(),
                    # 'bg_objects': None if bg_objects is None else bg_objects.to_serializable(),
                    'cfg': configs,
                }
                # Combine paths using string concatenation or os.path.join
                pcd_save_path = os.path.join(configs.save_pcd_path, "full_pcd.pkl.gz")

                # Make sure the parent directories exist
                os.makedirs(os.path.dirname(pcd_save_path), exist_ok=True)

                # pcd_save_path = configs.save_pcd_path / "full_pcd.pkl.gz"
                # # 如果目录不存在则创建
                # pcd_save_path.parent.mkdir(parents=True, exist_ok=True)
                with gzip.open(pcd_save_path, "wb") as f:
                    pickle.dump(results, f)
                print(f"保存点云地图到 {pcd_save_path}")
                # break
    
        print("The task is completed OR the try number is exceeded")
    

class SceneGraph(Node):
    def print_and_log(self, string):
        self.get_logger().info(string)
        print_verbose_only(self.configs, string)

    def __init__(self, cfg, observation_buffer, sync_queue_size: int = 1000, max_time_diff: float = 0.05):
        self.configs= cfg
        self.observation_ready = False
        self.lock = threading.Lock()
        camera_transformation_k = cfg['camera_transformation_k']
        self.camera_transformation_k = read_space_separated_matrix(camera_transformation_k)
        rotation_matrix = cfg['rotation_matrix']
        self.radial_distortion = np.array([0.0, 0.0, 0, 0])

        # self.T_CL = np.zeros((3,4))
        # self.T_CL[:,:3] = read_space_separated_matrix(rotation_matrix)
        # self.T_CL[:,3] = np.array(cfg['translation_vector'])


        self.image_height = cfg['image_height']
        self.image_width = cfg['image_width']
        
        self.new_K, _ = cv2.getOptimalNewCameraMatrix(self.camera_transformation_k, self.radial_distortion, (self.image_width, self.image_height), 1, (self.image_width, self.image_height))  

        super().__init__('scene_graph')
        '''self.camera_subscription = self.create_subscription(
            Image,
            'camera',
            self.camera_callback,
            10)
        self.camera_subscription
        self.lidar_subscription = self.create_subscription(
            PointCloud2,
            'ouster/points',  
            self.lidar_callback,
            10)
        self.lidar_subscription
        self.trailbotpose_subscriber = self.create_subscription(
            PoseStamped, 
            "trail_location", 
            self.trailbotpose_callback, 
            10)
        self.trailbotpose_subscriber'''

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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

        # self.lidar_pos_to_map_publisher = self.create_publisher(
        #     PoseStamped,
        #     'lidarpose_to_map',
        #     10)
        
        # self.timer = self.create_timer(0.01, self.publish_lidarpos_to_map)

        # create subscribers
        self.image_sub = message_filters.Subscriber(self, Image, 'camera')
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, 'ouster/points')
        self.lidarpos_sub = message_filters.Subscriber(self, TransformStamped, 'base_link_transform')

        
        self.observation_buffer = observation_buffer
        self.memory_buffer = [] #
        # create callback
        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub, self.lidarpos_sub], sync_queue_size, max_time_diff)
        ts.registerCallback(self.observation_callback) # divide to different core?

        self.timestamp, self.callback_times = 0, 0
        
        self.cv_image = None
        self.first_pose = None

        self.frame_id = 0
        self.publishing_frequency = -1
        # Start the scene graph construction
        self.livedata_preprocessing = threading.Thread(target=self.run_livedata_processing)
        # self.scenegraph  = threading.Thread(target=self.run_scenegraph_generation)
        
        self.livedata_preprocessing.start()
        # self.scenegraph.start()



    # def publish_lidarpos_to_map(self):
    #     try:
    #         transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
    #         pos_msg = PoseStamped()
    #         # get the time of the transform
    #         pos_msg.header.stamp = transform.header.stamp
    #         pos_msg.header.frame_id = 'map'
    #         pos_msg.pose.position.x = transform.transform.translation.x
    #         pos_msg.pose.position.y = transform.transform.translation.y
    #         pos_msg.pose.position.z = transform.transform.translation.z
    #         pos_msg.pose.orientation.x = transform.transform.rotation.x
    #         pos_msg.pose.orientation.y = transform.transform.rotation.y
    #         pos_msg.pose.orientation.z = transform.transform.rotation.z
    #         pos_msg.pose.orientation.w = transform.transform.rotation.w
    #         self.lidar_pos_to_map_publisher.publish(pos_msg)
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #         self.get_logger().info('Could not transform os_lidar to map: {0}'.format(e))
    #         time.sleep(1)
    #         return
    
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
                

    def publish_lidarpos_to_map(self):
        try:
            transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            pos_msg = PoseStamped()
            # get the time of the transform
            pos_msg.header.stamp = transform.header.stamp
            pos_msg.header.frame_id = 'map'
            pos_msg.pose.position.x = transform.transform.translation.x
            pos_msg.pose.position.y = transform.transform.translation.y
            pos_msg.pose.position.z = transform.transform.translation.z
            pos_msg.pose.orientation.x = transform.transform.rotation.x
            pos_msg.pose.orientation.y = transform.transform.rotation.y
            pos_msg.pose.orientation.z = transform.transform.rotation.z
            pos_msg.pose.orientation.w = transform.transform.rotation.w
            self.lidar_pos_to_map_publisher.publish(pos_msg)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().info('Could not transform os_lidar to map: {0}'.format(e))
            time.sleep(1)
            return

    def observation_callback(self, camera_msg: Image, lidar_msg: PointCloud2, tf_msg: PoseStamped) -> None:
        """
        Callback function for processing camera and lidar messages and pose message.

        Args:
            camera_msg (sensor_msgs.msg.Image): Camera message.
            lidar_msg (sensor_msgs.msg.PointCloud2): Lidar message.
            lidarpos_msg (geometry_msgs.msg.PoseStamped): Lidar pose message.

        """
        t1 = time.time()
        cv_image = self.bridge.imgmsg_to_cv2(
            camera_msg, desired_encoding='rgb8')#rgb8
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) #Added for colour correction to RGB
        self.cv_image = cv2.undistort(cv_image, self.camera_transformation_k, self.radial_distortion, None, self.new_K)
        self.timestamp_cam = camera_msg.header.stamp
        #self.get_logger().info(f"Camera image shape: {self.cv_image.shape}")
        t2 = time.time()
        
        # Deserialize PointCloud2 data into xyz points
        point_gen = pc2.read_points(
            lidar_msg, field_names=(
                "x", "y", "z"), skip_nans=True)
        # Filter out points with all 0s
        #self.points = np.array([[x, y, z, 1] for x, y, z in point_gen if any([i!=0 for i in [x,y,z]])])
  
        points = np.array([point_gen['x'], point_gen['y'], point_gen['z']]).T
        mask = ~np.all(points == 0, axis=1)
        # Apply the mask to filter out rows with all zero coordinates
        self.points = points[mask]

        t3 = time.time()
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
        
        t4 = time.time()
        #print(f'time for processing one pair of data: {t4-t1} with camera: {t2-t1}, lidar: {t3-t2} and transformation: {t4-t3}')  
        
    
    def camera_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough')
        self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB) #Added for colour correction to RGB
        self.cv_image = cv2.undistort(self.cv_image, self.camera_transformation_k, self.radial_distortion, None, self.new_K)
        self.timestamp = msg.header.stamp
        #self.get_logger().info(f"Camera image shape: {self.cv_image.shape}")

    def lidar_callback(self, msg):
        # Deserialize PointCloud2 data into xyz points
        point_gen = pc2.read_points(
            msg, field_names=(
                "x", "y", "z"), skip_nans=True)
        # Filter out points with all 0s
        points = [[x, y, z, 1] for x, y, z in point_gen if any([i!=0 for i in [x,y,z]])]
        self.points = np.array(points)       
        self.timestamp = msg.header.stamp
        self.get_logger().info(f"Lidar points received: {len(points)}")

    def trailbotpose_callback(self, msg):
        self.number_of_trail_points = 1
        
        try:
            self.trailbot_pose = self.tf_buffer.transform(msg, 'map')
            #self.get_logger().info(f'the {self.number_of_trail_points}th trailbot pose is {self.trailbot_pose}')
            if self.number_of_trail_points == 1:
                self.first_pose = np.array([self.trailbot_pose.pose.position.x, self.trailbot_pose.pose.position.y, self.trailbot_pose.pose.position.z])
            self.number_of_trail_points += 1
            #self.get_logger().info(f"number of trail points: {self.number_of_trail_points}")
        except tf2_ros.TransformException as ex:
            # self.get_logger().info('Keep old trail location', throttle_duration_sec=1)
            self.get_logger().info('Could not transform os_lidar to map: {0}'.format(ex))
            return

    def run_livedata_processing(self):
        self.get_logger().info("Initialized live data processing...")
        first_transformation_matrix = None
        T_baselink_to_lidar = np.array([
                            [ 0, -1,  0,  0],  # Flip the X-axis
                            [ 1,  0,  0,  0],  # Flip the Y-axis
                            [ 0,  0,  1,  0],  # Z-axis remains the same
                            [ 0,  0,  0,  1]   # Homogeneous coordinate
                        ])
        T_lidar_to_baselink = np.linalg.inv(T_baselink_to_lidar)
        trajectory = np.array([])
        pre_callback_times = 0
        while rclpy.ok():
            # with self.lock:
            t1 = time.time()
            
            if self.observation_ready:
                observation = (copy.deepcopy(self.callback_times), copy.deepcopy(self.cv_image), copy.deepcopy(self.points), copy.deepcopy(self.transformation_matrix))
                if first_transformation_matrix is None:
                    first_transformation_matrix = observation[3]
                    first_transformation_matrix_inv = np.linalg.inv(first_transformation_matrix)
                    #self.get_logger().info(f"First transformation matrix: {first_transformation_matrix}")
                base_link_2map_TF = np.dot(first_transformation_matrix_inv, observation[3])    
                #self.get_logger().info(f"self.transformation_matrix: {self.transformation_matrix}")
                lidar_2map_TF = base_link_2map_TF @ T_lidar_to_baselink

                
                if observation[0] != pre_callback_times:
                    self.memory_buffer.append((observation[0], observation[1], observation[2], lidar_2map_TF)) 
                    #self.get_logger().info(f"Observation added to memory buffer: {observation[0]}")
                    # append the xyz coordinates to the trajectory for plotting trajectory
                    #trajectory = np.append(trajectory, np.array([self.transformation_matrix[0, 3], self.transformation_matrix[1, 3], self.transformation_matrix[2, 3]]), axis=0)
                    
                # elif self.memory_buffer[-1][0] != pre_callback_times:
                #     self.memory_buffer.append((observation[0], observation[1], observation[2], lidar_2map_TF))
                #     # append the xyz coordinates to the trajectory for plotting trajectory
                #     #trajectory = np.append(trajectory, np.array([self.transformation_matrix[0, 3], self.transformation_matrix[1, 3], self.transformation_matrix[2, 3]]), axis=0)
                #     #self.get_logger().info(f"Observation added to memory buffer: {self.callback_times}")
                else:
                    time.sleep(0.1)
                    continue

                if len(self.memory_buffer) % self.configs.stride == 0 and len(self.memory_buffer) > 0:
                    image, pc, pose, his_pcs, his_poses = get_observation(self.configs.stride, self.memory_buffer)
                    data = (image, pc, pose, his_pcs, his_poses)
                    self.observation_buffer.put(data)
                    self.memory_buffer = []
            # print(f"{time.time()-t1} seconds for processing one group of data")
                #print(f"Trajectory: {trajectory}")
                pre_callback_times = observation[0]
    
                
 
def start_ros_node(observation_buffer, args, cfg):
    rclpy.init(args=args)
    subscriber = SceneGraph(cfg, observation_buffer)
    subscriber.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, False)])
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()    
            

@hydra.main(version_base=None, config_path="../config", config_name="scenegraph")
def main(cfg : DictConfig, args=None):
    debug_mode = False
    cfg = process_cfg(cfg)
    mask_generator = load_models(cfg)
    objects = MapObjectList(device="cuda")
    observation_buffer = multiprocessing.Queue()

    ros_process = multiprocessing.Process(target=start_ros_node, args=(observation_buffer, args, cfg))
    ros_process.start()
    calib = load_calib(os.path.join(cfg['calib_path'], "calib.txt"))
    run_scenegraph_generation(cfg, observation_buffer, objects, mask_generator, calib)



import subprocess, threading, os
def run_shell_command(command):
    
    with open(os.devnull, 'w') as nullfile:
        process = subprocess.Popen(command, shell=True, stdout=nullfile, stderr=subprocess.STDOUT)
        process.communicate()

if __name__ == '__main__':

    print("\n\nDEBUG MODE ON\n\n")
    command1 = "ros2 run image_transport republish compressed raw --ros-args --remap in/compressed:=/camera/compressed --remap out:=/camera"
    command2 = "ros2 bag play /home/trailbot/bags/ptp_time" #2Indoor9 Outdoor

    # Create threads for each shell command and main function
    thread1 = threading.Thread(target=run_shell_command, args=(command1,))
    thread2 = threading.Thread(target=run_shell_command, args=(command2,))
    thread_main = threading.Thread(target=main, args=(None,))

    thread1.start()
    thread2.start()
    thread_main.start()

    # Wait for the threads to finish
    thread1.join()
    thread2.join()
    thread_main.join()


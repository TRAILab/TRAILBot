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
from utils.helpers import visualize_map, update_flag, read_update_flag
from utils.world_map import ObjectVisualizer, visualize_objects_org
import copy
import platform
import multiprocessing
import logging
import io

from scipy.spatial import KDTree

from nav_msgs.msg import Path as NavPath
import json
from rclpy.time import Duration
from rclpy.time import Time
# import keyboard  # pip install keyboard



from io import StringIO
import traceback

from contextlib import redirect_stdout
import matplotlib.pyplot as plt


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

# BG_CAPTIONS = ["some trees on the both sides of the street", "a street with trees", "a street with trees"]
# # 背景映射出来的标签
# # 
# BG_CAPTIONS_Pro = ["tree", "forest", "forest"]
# # 背景映射出来的标签有哪些
# BG_CAPTIONS_Pro_Sim = ["tree"] 

BG_CAPTIONS =["paved city road", "a long narrow street", "paved city street", "a paved grey road",\
               "white lines on the road", "white lines on the road", "white lines on the road",\
                "white number on the road", "white letter painted on the street", "a candle", "white number on the road",\
                "a sidewalk next to the train tracks", "shadow of bench", "a paved city sidewalk", "a train boarding platform",\
                "shadow of the tree", "shadow of bench", "shadow of sidewalk", "a shadow on the ground",\
                "white lines on the street", "brick sidewalk", "a sidewalk next to the street",\
                "a concrete sidewalk", "a sidewalk", "a paved sidewalk", "a sidewalk next to the street/road",\
                "a scene inside", "a scene of a room", "a kichen",\
                "the floor is white", "the floor", "the floor is black/blue", "the floor is"]
BG_CAPTIONS_Pro = ["paved road", "paved road", "paved road", "paved road",\
                "paved road", "paved road", "paved road",\
                "paved road", "paved road", "paved road", "paved road",\
                "sidewalk", "sidewalk", "sidewalk", "sidewalk",\
                "sidewalk", "sidewalk", "sidewalk", "sidewalk",\
                "paved road", "sidewalk", "sidewalk",\
                "sidewalk", "sidewalk", "sidewalk", "sidewalk",\
                "scene", "scene", "scene",\
                "floor", "floor", "floor", "floor"]
BG_CAPTIONS_Pro_Sim = ["paved road", "paved road", "paved road", "sidewalk", "sidewalk", "sidewalk", "sidewalk", "scene", "floor"]#

BG_CAPTIONS = []
BG_CAPTIONS_Pro = []
BG_CAPTIONS_Pro_Sim = []

# === Function to run a shell command in a thread ===
def run_shell_command(command):
    # Suppress output by redirecting to null
    with open(os.devnull, 'w') as nullfile:
        process = subprocess.Popen(command, shell=True, stdout=nullfile, stderr=subprocess.STDOUT)
        process.wait()  # Wait until the command completes

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

def build_terrain_map(map, points, map_origin, resolution, height_threshold=0.09):
    points = np.asarray(points)
    if points.shape[0] == 0:
        return map
    else:
        tree = KDTree(points[:,[0, 1]])

        for pcd in points:
            _, idx = tree.query([pcd[0], pcd[1]], k=5)
            
            neihbor_highs = points[idx][:, 1]
            #print("idx", idx, neihbor_highs)
            height_var = abs(np.max(neihbor_highs) - np.min(neihbor_highs))
            pcd_indices = np.floor(([pcd[0], pcd[1]] - map_origin) / resolution).astype(int)
            
            map[pcd_indices[0], pcd_indices[1]] = 5 if height_var > height_threshold else 0
            # if height_var > height_threshold:
            #     print(f"---------{pcd_indices[0], pcd_indices[1]]}---{height_var}----------------")
        
        return map

def fill_holes_in_occupancy_map(occupancy_map, mode="morphological", gap_closing_size=8, dilation_size=8, drive_area=None):
    from scipy.ndimage import binary_closing, binary_fill_holes, binary_dilation, center_of_mass
    from scipy.interpolate import griddata
    """
    Fill holes in the occupancy map using morphological operations.

    Args:
        occupancy_map (np.ndarray): The input occupancy map.

    Returns:
        np.ndarray: Processed occupancy map with holes filled.
    """
    if mode == "morphological":
        # Initialize the filled map with the same shape as the input map
        filled_map = np.full_like(occupancy_map, -1)  # Unknown regions initialized to -1

        unique_classes = np.unique(occupancy_map)
        for cls in unique_classes:
            if cls == -1:
                # Skip unknown regions
                continue

            # Create a binary mask for the current class
            class_mask = (occupancy_map == cls)

            if cls == drive_area:  # Free cells
                # Apply hole filling to the free region
                filled_class = binary_fill_holes(class_mask).astype(int)
            else:  # Occupied or other semantic classes
                # Apply binary closing to the regions to fill small holes
                filled_class = binary_closing(class_mask, structure=np.ones((gap_closing_size, gap_closing_size))).astype(int)

            # Update the filled map
            filled_map[filled_class == 1] = cls
        return filled_map

    elif mode == "interpolation":
        # Step 1: Initialize the filled map with the same shape
        filled_map = np.full_like(occupancy_map, -1)  # Unknown regions initialized to -1

        # Get all unique classes in the map, excluding unknown (-1)
        unique_classes = np.unique(occupancy_map)
        unique_classes = unique_classes[unique_classes != -1]  # Exclude unknown

        for cls in unique_classes:
            # Step 2: Create a binary mask for the current class
            class_mask = (occupancy_map == cls)

            # Step 3: Close small gaps in the current class
            closed_class = binary_closing(
                class_mask, structure=np.ones((gap_closing_size, gap_closing_size))
            ).astype(int)

            # Step 4: Fill holes in the free-space areas (if cls == 0)
            if cls == 0:  # Free cells
                closed_class = binary_fill_holes(closed_class).astype(int)

            # Step 5: Dilate the class region to smooth boundaries
            dilated_class = binary_dilation(
                closed_class, structure=np.ones((dilation_size, dilation_size))
            ).astype(int)

            # Update the filled map with the processed class
            filled_map[dilated_class == 1] = cls

        # Step 6: Interpolate unknown regions (-1) using the nearest-neighbor method
        road_boundary_mask = (filled_map != -1)  # Non-unknown areas
        known_coords = np.argwhere(road_boundary_mask)  # Coordinates of known cells
        known_values = filled_map[road_boundary_mask]   # Values of known cells

        # Create a grid for the map
        grid_z, grid_x = np.meshgrid(
            np.arange(filled_map.shape[0]),
            np.arange(filled_map.shape[1]),
            indexing='ij'
        )
        grid_coords = np.column_stack((grid_z.ravel(), grid_x.ravel()))

        # Interpolate missing values (-1)
        interpolated_map = griddata(
            known_coords, known_values, grid_coords, method='nearest', fill_value=-1
        )
        interpolated_map = interpolated_map.reshape(filled_map.shape)

        # Ensure interpolation doesn't overwrite unknown regions outside the boundary
        interpolated_map[~road_boundary_mask] = -1

        return interpolated_map

def create_2d_map_point(detected_objects, bg_list, pose, prev_occupancy_map, prev_map_origin, pre_detection_list, pre_bg_list, resolution=0.1):
    """
    Create a 2D occupancy map based on detected objects by projecting onto the Z-X plane.

    Args:
        detected_objects (list of dict): List of detected objects, each containing 'pcd' (point cloud).
        resolution (float): The resolution of the map in meters per grid cell.

    Returns:
        occupancy_map (np.ndarray): A 2D occupancy map (Z-X plane).
        map_origin (np.ndarray): Origin of the map in world coordinates (Z, X).
    """
    all_points = []
    sematic_info = []
    keypoint_list = []
    object_indices_dict = {}
    
    # Collect all points from detected objects
    for obj in detected_objects + bg_list + pre_detection_list + pre_bg_list:
        pcd = obj['pcd']  # Assuming 'pcd' is an open3d.geometry.PointCloud
        points = np.asarray(pcd.points)  # Convert to NumPy array
        all_points.append(points[:, [0, 1]])  # Take Z and X coordinates
    all_points.append(pose[[0, 1], 3])
    # for obj in bg_list:
    #     pcd = obj['pcd']
    #     points = np.asarray(pcd.points)
    #     all_points.append(points[:, [0, 2]])

    # Combine all points into a single array
    all_points = np.vstack(all_points) if all_points else np.array([])

    if all_points.size == 0:
        raise ValueError("No points found in detected objects.")

    # Determine the bounds of the map (in Z-X plane)
    min_bound = np.min(all_points, axis=0)  # [min_z, min_x]
    max_bound = np.max(all_points, axis=0)  # [max_z, max_x]
    global_map = False
    # Expand previous map if it exists
    # if prev_occupancy_map is not None and prev_map_origin is not None and global_map:
    #     prev_shape = np.array(prev_occupancy_map.shape)
    #     prev_max_bound = prev_map_origin + prev_shape * resolution
    #     # global map
        
    #     if global_map:
    #         min_bound = np.minimum(min_bound, prev_map_origin)
    #         max_bound = np.maximum(max_bound, prev_max_bound)

    # Calculate the size of the occupancy map
    map_size = np.ceil((max_bound - min_bound) / resolution).astype(int)
    occupancy_map = np.full(map_size, -1, dtype=np.int8)
    sematic_map = np.full(map_size, -1, dtype=np.int8)
    terrain_map = np.full(map_size, 0.0, dtype=np.float32)

    # Store the origin of the map in world coordinates (x, z)
    map_origin = min_bound
    road_pcd_list = []
    
    from scipy.spatial.transform import Rotation as R
    rot = R.from_matrix(pose[:3, :3])
    # robot_theta = rot.as_euler('zyx')
    robot_theta = rot.as_euler('xyz')[2]  # Get the yaw angle (rotation around Z-axis)

    pos_indices = np.floor((pose[[0, 1], 3]
                             - map_origin) / resolution).astype(int)


    drivable_indices = []
    terrain_pcd = []
    terrain_indices = []
    centerline ={}
    for obj in bg_list+pre_bg_list:
        pcd = obj['pcd']   
        points_ = np.asarray(pcd.points)
        points = points_[:, [0, 1]]
        indices = np.floor((points - map_origin) / resolution).astype(int)
        indices = np.clip(indices, 0, map_size - 1)
        for i, idx in enumerate(indices):
            occupancy_map[idx[0], idx[1]] = 0
            
    for obj in bg_list:
        pcd = obj['pcd']
        drivable_indices.append(obj['img_bbox'])        
        points_ = np.asarray(pcd.points)
        points = points_[:, [0, 1]]
        indices = np.floor((points - map_origin) / resolution).astype(int)
        indices = np.clip(indices, 0, map_size - 1)
        # if obj["img_bbox"] in object_indices_dict:
        #     object_indices_dict[obj["img_bbox"]].append([indices, points_])
        # else:
        terrain_pcd.extend(points_)
        print(f"terrain_pcd: {len(terrain_pcd)}")
        object_indices_dict[obj["img_bbox"]] = [indices, points_]
        for i, idx in enumerate(indices):
            sematic_map[idx[0], idx[1]] = obj["img_bbox"]
            if idx[0] % 15 == 0 and idx[1] % 15 == 0 and idx.tolist() not in keypoint_list:
                road_pcd_list.append(points_[i])
                keypoint_list.append(idx.tolist())
                # print(i, points_[i], type(points_[i]))
        # TODO: issue
        sematic_info.append({"cap": obj["caption"], "img_bbox": obj["img_bbox"]}) # 'driveable area'
    
    terrain_map = build_terrain_map(terrain_map, terrain_pcd, map_origin, resolution)
    drive_area = obj["img_bbox"]
    #sematic_map = fill_holes_in_occupancy_map(sematic_map, mode="interpolation", gap_closing_size=4, dilation_size=4, resolution=resolution)
    
    occupancy_map = fill_holes_in_occupancy_map(occupancy_map, mode="interpolation", gap_closing_size=8, dilation_size=8, drive_area=drive_area)
    sematic_map = fill_holes_in_occupancy_map(sematic_map, mode="interpolation", gap_closing_size=8, dilation_size=8, drive_area=drive_area)
    #terrain_map = fill_holes_in_occupancy_map(terrain_map, mode="interpolation", gap_closing_size=8, dilation_size=8, drive_area=drive_area)
    # Populate the 2D occupancy map
    object_pcd_list = []
    for i, obj in enumerate(detected_objects):
    # for obj in detected_objects:
        pcd = obj['pcd']
        points_ = np.asarray(pcd.points)  # Extract Z and X coordinates
        points = points_[:, [0, 1]]
        # Convert world coordinates to map indices
        indices = np.floor((points - map_origin) / resolution).astype(int)
        indices = np.clip(indices, 0, map_size - 1)  # Ensure indices are within bounds
        # if obj["img_bbox"] in object_indices_dict:
        #     object_indices_dict[obj["img_bbox"]].append([indices, points_])
        # else:
        
        object_indices_dict[obj["img_bbox"]] = [indices, points_]
        # Mark occupied cells
        for i, idx in enumerate(indices):
            occupancy_map[idx[0], idx[1]] = 1
            sematic_map[idx[0], idx[1]] = obj["img_bbox"]
            if (idx[0] % 10) == 0 and (idx[1] % 10 == 0):
                object_pcd_list.append(points_[i])
                keypoint_list.append(points_[i])
        
        caption_obj = obj["caption"]
        #class_name = objects[i]["class"]
        if ", " in caption_obj:
            last_caption = caption_obj[caption_obj.rfind(', ')+2:]
        else:
            last_caption = caption_obj
        sematic_info.append({"cap": last_caption, "img_bbox": obj["img_bbox"]})
    
    robot_state = {"robot_indices": pos_indices, "robot_state": (pose[:3, 3], robot_theta)}
    occupancy_map = fill_holes_in_occupancy_map(occupancy_map, mode="morphological", gap_closing_size=4, dilation_size=4, drive_area=drive_area)
    sematic_map = fill_holes_in_occupancy_map(sematic_map, mode="morphological", gap_closing_size=4, dilation_size=4, drive_area=drive_area)
    return occupancy_map, map_origin, sematic_map, sematic_info, np.array(road_pcd_list), np.array(object_pcd_list), object_indices_dict, robot_state, drivable_indices, terrain_map

def get_semantic_area_grid_points(semantic_map, semantic_index, grid_interval=10, robot_state=None):
    copied_map = copy.deepcopy(semantic_map)
    object_mask = (copied_map == semantic_index)
    x_indices = np.arange(copied_map.shape[0])
    y_indices = np.arange(copied_map.shape[1])
    # grid_point_mask = (x_indices[:, None] % grid_interval_x == 0) & (y_indices % grid_interval_z == 0)
    # if robot_state["robot_state"]
    # grid_point_mask1 = ((x_indices[:, None] % (grid_interval*2) == 0) & (y_indices % (grid_interval*2) == 0) & ((y_indices // (grid_interval*2)) % 2== 1))
    # grid_point_mask2 = ((x_indices[:, None] % (grid_interval) == 0) & ((x_indices[:, None] // (grid_interval)) %2 == 1) & (y_indices % (grid_interval*4) == 0))
    grid_point_mask1 = ((x_indices[:, None] % (grid_interval) == 0) & ((x_indices[:, None] // (grid_interval)) %2 == 0)) & ((y_indices % (grid_interval) == 0) & ((y_indices // (grid_interval)) %2 == 0))
    grid_point_mask2 = ((x_indices[:, None] % (grid_interval) == 0) & ((x_indices[:, None] // (grid_interval)) %2 == 1)) & ((y_indices % (grid_interval) == 0) & ((y_indices // (grid_interval)) %2 == 1))
    grid_point_mask = grid_point_mask1 | grid_point_mask2
    return np.argwhere(object_mask & grid_point_mask)#[:, ::-1]

def invert_transformation_matrix(matrix):
    R = matrix[:3, :3]  # Extract the rotation part
    t = matrix[:3, 3]   # Extract the translation part

    # Compute the inverse
    R_inv = R.T
    t_inv = -np.dot(R_inv, t)

    # Construct the inverse transformation matrix
    inv_matrix = np.eye(4)
    inv_matrix[:3, :3] = R_inv
    inv_matrix[:3, 3] = t_inv
    return inv_matrix

def backward_projection_to_rgb(sematic_info, road_interval, object_interval, sematic_map, pose, resolution, map_origin, object_indices_dict, calib, robot_state):
    for j, info in enumerate(sematic_info):
        
        if info["cap"] == "driveable area":
            road_index = info["img_bbox"]
            #road_index = find_semantic_index(sematic_info, "driveable area")
            road_grid_points = get_semantic_area_grid_points(sematic_map, road_index, road_interval, robot_state)
            global_points = road_grid_points * resolution + map_origin
        
            indices_all = object_indices_dict[road_index][0]
            points_all = object_indices_dict[road_index][1]
            indices_tree = KDTree(indices_all)   
            idx_results = [] 
            for i, data in enumerate(road_grid_points):
                _, idx = indices_tree.query(data, k=1)
                idx_results.append(idx)
                #print("idx", idx, i, indices_all[idx], data)
            org_points = points_all[idx_results]
            avg_value = np.mean(org_points[:, 1])
            updated_data = np.zeros((org_points.shape[0], 3))
            updated_data[:, 0] = global_points[:, 0]
            updated_data[:, 1] = avg_value
            updated_data[:, 2] = global_points[:, 1]
            if road_grid_points.shape[0] > 0:
                sematic_info[j]["SoM"] = updated_data
                sematic_info[j]["Occ_idx"] = road_grid_points
            else:
                sematic_info[j]["SoM"] = []
                sematic_info[j]["Occ_idx"] = []
        else:
            object_index = info["img_bbox"]
            object_grid_points = get_semantic_area_grid_points(sematic_map, object_index, object_interval)
            global_points = object_grid_points * resolution + map_origin
            indices_all = object_indices_dict[object_index][0]
            points_all = object_indices_dict[object_index][1]
            indices_tree = KDTree(indices_all)
            idx_results = []
            for i, data in enumerate(object_grid_points):
                _, idx = indices_tree.query(data, k=1)
                idx_results.append(idx)
            org_points = points_all[idx_results]
            updated_data = np.zeros((org_points.shape[0], 3))
            updated_data[:, 0] = global_points[:, 0]
            updated_data[:, 1] = org_points[:, 1]
            updated_data[:, 2] = global_points[:, 1]
            
            if object_grid_points.shape[0] > 0:
                sematic_info[j]["SoM"] = updated_data
                sematic_info[j]["Occ_idx"] = object_grid_points
            else:
                sematic_info[j]["SoM"] = []
                sematic_info[j]["Occ_idx"] = []

        road_pcd_list = info['SoM']
        if len(road_pcd_list) >0:
            road_pcd_homogeneous = np.hstack((road_pcd_list, np.ones((len(road_pcd_list), 1)))).T
            #object_pcd_homogeneous = np.hstack((object_pcd_list, np.ones((len(object_pcd_list), 1)))).T
        
            trans_pose_inv  = invert_transformation_matrix(pose)
            road_pcd_homo_inv = trans_pose_inv @ road_pcd_homogeneous
            #object_pcd_homo_inv = trans_pose_inv @ object_pcd_homogeneous
        
            # road_lidar_frame = road_pcd_homo_inv[:3, :].T
            # object_lidar_frame = object_pcd_homo_inv[:3, :].T

            road_cam = calib['P_rect_20'].dot(calib['T_cam2_velo']).dot(road_pcd_homo_inv) 
            #object_cam = calib['P_rect_20'].dot(calib['T_cam2_velo']).dot(object_pcd_homo_inv)
            road_cam[:2, :] /= road_cam[2, :]  
            #object_cam[:2, :] /= object_cam[2, :]  
            u,v,z  = road_cam
            pixels = np.dstack((v,u)).squeeze()
        else:
            pixels = np.array([])
        sematic_info[j]["pixels"] = pixels
    return sematic_info

def convert_to_nav2_frame(path_base_link):
    """
    Convert a list of 3D points from base_link frame (x-forward, y-left, z-up)
    to Nav2 map frame (x-left, y-backward, z-up).

    Args:
        path_base_link (list or np.ndarray): List of [x, y, z] points.

    Returns:
        List of [x, y, z] points in Nav2 frame.
    """
    nav2_path = []
    for pt in path_base_link:
        x_bl, y_bl, z_bl = pt
        x_nav2 = y_bl
        y_nav2 = -x_bl
        z_nav2 = z_bl
        nav2_path.append([x_nav2, y_nav2, z_nav2])
    return nav2_path

def write_latest_frame_idx(idx, base_path="/home/trailbot/Documents/data_process_Kitti/results/04"):
    """
    Save the current frame index to latest_frame.txt so the GUI can display the most recent image.
    """
    os.makedirs(base_path, exist_ok=True)
    latest_file = os.path.join(base_path, "latest_frame.txt")
    with open(latest_file, "w") as f:
        f.write(str(idx * 10))  # assuming your frame naming is idx*10 + configs.start


def run_scenegraph_generation(configs, observation_buffer, objects, mask_generator, calib):
    """
    Runs caption and mask generation as soon as new data is available.
    """
    #frame_id = 0

    print("Initialized scene graph...")
    interval, idx = 20, 0
    prev_occupancy_map, prev_map_origin, prev_sematic_map = None, None, None
    pre_detection_list, pre_bg_list = {}, {}
    point_clouds = []
    logger = multiprocessing.log_to_stderr()
    logger.setLevel(logging.INFO)
    vis_idx =0
    object_vis = ObjectVisualizer()

    while True: # todos
        # point_clouds = []
        if not observation_buffer.empty():
            start = time.time()
            image, pc, pose, his_pcs, his_poses = observation_buffer.get()
            # pcd = pc[:, :3]  # Assuming pc is a Nx4 array with x, y, z, intensity
            
            # if observation_buffer.qsize() > 1:
            #     print(f"Observation buffer size: {observation_buffer.qsize()}")
            #     continue

            #check the image type
 
            #print(f"Processing {frame_id}th frame..., and {observation_buffer.qsize()} frames left in the buffer")
            save_path_vis = Path(os.path.join(configs.save_vis_path, f"vis_{idx}"))
            masks_result, anotated_image = mask_generator.generate(image, save_path=save_path_vis, save_vis=configs.save_vis)
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
            if idx == 0:
                idx += 1
                continue
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            p2 = time.time()
            if his_pcs is not None:
                pc = accumulate_pc(configs, mos_model, pc, pose, his_pcs, his_poses)

            # filter the point cloud that is too far away
            if configs.filter_dis:
                pc = distance_filter(configs.max_depth, pc)
            # get the pixel value and the remaining point cloud after projection, which correspond one by one
            pro_point, pixels, z = project(pc, image, calib)
            # pro_point, pixels,z = project(pc, image, calib)
            if True: # for extrinsic calibration
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
                save_path = "/home/trailbot/Documents/data_process_Kitti/results/ptp_MO/"
                # Show the plot
                
                os.makedirs(save_path, exist_ok=True)
                depth_path = os.path.join(save_path, f"depth_{idx}.png")
                plt.savefig(depth_path, dpi=300, bbox_inches='tight')
                # plt.show()

            detection_list, bg_list = gobs_to_detection_list(
                cfg = configs,
                image = image,
                pc = pro_point,
                pixels = pixels,
                idx = idx,
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

            # no objects need to be added to the map
            if len(detection_list) == 0:
                idx += 1
                continue
            # for the first frame, add all the objects to the map
            if len(objects) == 0:
                for i in range(len(detection_list)):
                    objects.append(detection_list[i])
                # skip the similarity calculation
                idx += 1
                continue
            # visualize the map if needed
            if configs.vis_all:
                point_clouds.extend([detection_list[i]["pcd"] for i in range(len(detection_list))])
                point_clouds.extend([bg_objects[_]["pcd"] for _ in bg_objects if bg_objects[_] is not None])
            
            spatial_sim = compute_spatial_similarities(detection_list, objects)
            caption_sim = compute_caption_similarities(detection_list, objects)
            ft_sim = compute_ft_similarities(detection_list, objects)
            agg_sim = aggregate_similarities(configs, spatial_sim, ft_sim, caption_sim)
            
            agg_sim[agg_sim < configs.sim_threshold] = float('-inf')
            # merge according to similarity
            objects = merge_detections_to_objects(configs, detection_list, objects, agg_sim)

            # VLN block
            instruction_path = os.path.join(configs.instruction_path, "instruction.json")
            os.makedirs(os.path.dirname(instruction_path), exist_ok=True)
            if not os.path.isfile(instruction_path):
                with open(instruction_path, "w") as f:
                    json.dump({"trigger": False, "command": ""}, f)
            with open(instruction_path, 'r') as f:
                data = json.load(f)
            # object_vis.update(objects)

            if idx > 0 and idx % 5 == 0:
                visualize_objects_org(objects)
            # object_vis.update_pointclouds(point_clouds)
                

            if (len(bg_list)>0 and configs.VLT) and (read_update_flag(os.path.join(configs.UPDATE_FLAG_PATH, "update.txt")) or data.get('trigger', False)):
                from LLT.api_hardware import API
                from LLT.prompts.main_prompt_Kitty_hardware import MAIN_PROMPT
                from LLT.prompts.error_correction_prompt import ERROR_CORRECTION_PROMPT
                from LLT.prompts.print_output_prompt import PRINT_OUTPUT_PROMPT
                from LLT.prompts.task_failure_prompt import TASK_FAILURE_PROMPT
                from LLT.prompts.task_summary_prompt import TASK_SUMMARY_PROMPT
                from LLT import models
                write_latest_frame_idx(idx)
                if read_update_flag(os.path.join(configs.UPDATE_FLAG_PATH, "update.txt")):
                    print("✅ Update was triggered.")
                    update_flag(os.path.join(configs.UPDATE_FLAG_PATH, "update.txt"), trigger =False)

                occupancy_map, map_origin, sematic_map, sematic_info, _, _, object_indices_dict, robot_state, drivable_indices, terrain_map = create_2d_map_point(detection_list, bg_list, pose, prev_occupancy_map, prev_map_origin, pre_detection_list, pre_bg_list, configs.resolution)
                
                pre_detection_list, pre_bg_list = copy.deepcopy(detection_list), copy.deepcopy(bg_list)
                prev_occupancy_map = occupancy_map
                prev_sematic_map = sematic_map
                prev_map_origin = map_origin

                sematic_info= backward_projection_to_rgb(sematic_info, interval, interval, sematic_map, pose, configs.resolution, map_origin, object_indices_dict, calib, robot_state)
                SoM_new = anotated_image#create_SoM_keypts(anotated_image, sematic_info, drivable_indices, interval)
                # # check if the path exists
                if not os.path.exists(configs.annotated_rgb_path):
                    os.makedirs(configs.annotated_rgb_path)
                
                if configs.save_SoM:
                    cv2.imwrite(os.path.join(configs.annotated_rgb_path, f"annotated_rgb_{idx*10 + configs.start}.png"), SoM_new)
            # #cv2.imwrite(f'SoM{idx}.png', SoM_new)       
            
            # #fig = visualize_map(sematic_map, map_origin, resolution, idx, robot_state, map_type="sematic", sematic_info=sematic_info, zoom=True)
                if configs.OccMap:
                    fig = visualize_map(occupancy_map, map_origin, configs.resolution, idx, robot_state, map_type="occ", sematic_info=sematic_info, interval=interval)
                    if configs.save_occ_map:
                        # check if the path exists
                        if not os.path.exists(configs.occ_map_path):
                            os.makedirs(configs.occ_map_path)
                        fig.savefig(os.path.join(configs.occ_map_path, f"occ_map_{idx*10 + configs.start}.png"), format='png')
                    else:
                        with io.BytesIO() as buffer:
                            fig.savefig(buffer, format='png')
                            buffer.seek(0)
                            occ_map_img = buffer.getvalue()

                if configs.SemanticMap:
                    fig = visualize_map(sematic_map, map_origin, configs.resolution, idx, robot_state, map_type="sematic", sematic_info=sematic_info, zoom= False, drivable_indices=drivable_indices, interval=interval)
                    if configs.save_semantic_map:
                        if not os.path.exists(configs.sem_map_path):
                            os.makedirs(configs.sem_map_path)
                        fig.savefig(os.path.join(configs.sem_map_path, f"sem_map_{idx*10 + configs.start}.png"), format='png', dpi=500)
                    else:
                        with io.BytesIO() as buffer:
                            fig.savefig(buffer, format='png')
                            buffer.seek(0)
                            seg_map_img = buffer.getvalue()
                            # Load the image from bytes
                            # image = Image.open(io.BytesIO(seg_map_img))
                            # # Show the image
                            # image.show()
        
                api = API(configs, detection_list, bg_list, pose, idx, logger, sematic_info, drivable_indices, robot_state, sematic_map, configs.resolution, map_origin, occupancy_map, terrain_map)
                api.detect_object_label(activate=False)
        
            
            
            # api.visualize_3D_map(point_clouds)
            
        
            #if configs.vis_all: #observation_buffer.empty() and     and used_idx % 2 == 0 and used_idx>2
                if data.get('trigger', False):
                    command = data.get('command', "")
                    # Reset the trigger
                    data['trigger'] = False
                    
                    update_flag(os.path.join(configs.UPDATE_FLAG_PATH, "update.txt"), trigger =True)
                    with open(instruction_path, 'w') as f:
                        json.dump(data, f)
                # if configs.VLT:
                    
                    ee_start_position = api.matrix_to_xyz_yaw()
                    
                    
                    image_path1 = SoM_new 
                    image = cv2.cvtColor(SoM_new, cv2.COLOR_BGR2RGB)

                    # Show image with a title
                    # plt.figure(f"{idx*10 + configs.start}")  # Set custom title
                    # plt.imshow(image)
                    # plt.axis("off")  # Hide axes for better visualization
                    # plt.show()
                    # from config import OK, PROGRESS, FAIL, ENDC
                    # Output
                    OK = "\033[92m"
                    PROGRESS = "\033[93m"
                    FAIL = "\033[91m"
                    ENDC = "\033[0m"
                    

                    api.update_command(command)
                    COT_LOG_PATH = os.path.join(configs.COT_LOG_PATH, f"cot_log_{idx*10 + configs.start}.txt")
                    os.makedirs(os.path.dirname(COT_LOG_PATH), exist_ok=True)
                    
                    # check if command is empty
                    if command == "":
                        continue
                    else:
                        if os.path.exists(COT_LOG_PATH):
                            os.remove(COT_LOG_PATH)
                    new_prompt = MAIN_PROMPT.replace("[INSERT TASK]", command)
                    # new_prompt += VQAs
                    messages = []
                    error = False
        
                    # save the image with high resolution
                    logger.info(f'First token length: exec {api.count_tokens(new_prompt)}')
                    messages = models.get_chatgpt_output("gpt-4.1", new_prompt, messages, "system", COT_LOG_PATH, image_path1)
                    
                    logger.info(OK + "Finished generating ChatGPT output - Outside the while loop!" + ENDC)
                    # Append latest content to CoT log
                    # with open(COT_LOG_PATH, "a") as f:
                    #     if "content" in messages[-1]:
                    #         f.write(f"{messages[-1]['role'].capitalize()}:\n{messages[-1]['content']}\n\n")

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
                                            
                                            exec(code)
                                            time.sleep(1)
                                    except Exception:
                                        error_message = traceback.format_exc()
                                        with open(COT_LOG_PATH, "a") as f:
                                                f.write(f"Output:\n{error_message}\n\n")
                                        new_prompt += ERROR_CORRECTION_PROMPT.replace("[INSERT BLOCK NUMBER]", str(block_number)).replace("[INSERT ERROR MESSAGE]", error_message)
                                        new_prompt += "\n"
                                        error = True
                                        
                                    else:
                                        # wait for the print statement to finish
                                        s = f.getvalue()
                                        error = False
                                        if s != "":
                                            # with open(COT_LOG_PATH, "a") as f:
                                            #     f.write(f"Output:\n{s}\n\n")

                                            new_prompt += PRINT_OUTPUT_PROMPT.replace("[INSERT PRINT STATEMENT OUTPUT]", s)
                                            new_prompt += "\n"
                                            error = False
                                        
                        if error:
                            api.completed_task = False
                            #api.failed_task = True

                        time.sleep(1)
                        try_num += 1
                        if not api.completed_task:
                            if api.failed_task:

                                logger.info(FAIL + "FAILED TASK! Generating summary of the task execution attempt..." + ENDC)

                                new_prompt += TASK_SUMMARY_PROMPT
                                new_prompt += "\n"

                                logger.info(PROGRESS + "Generating ChatGPT output..." + ENDC)
                                messages = models.get_chatgpt_output("gpt-4.1", new_prompt, messages, "user", COT_LOG_PATH, image_path1)
                                # if "content" in messages[-1]:
                                    # with open(COT_LOG_PATH, "a") as f:
                                    #     role = messages[-1].get("role", "Assistant").capitalize()
                                    #     f.write(f"{role}:\n{messages[-1]['content']}\n\n")
                            
                                logger.info(OK + "Finished generating ChatGPT output!" + ENDC)

                                logger.info(PROGRESS + "RETRYING TASK..." + ENDC)

                                new_prompt = MAIN_PROMPT.replace("[INSERT EE POSITION]", str(config.ee_start_position)).replace("[INSERT TASK]", command)
                                new_prompt += "\n"
                                new_prompt += TASK_FAILURE_PROMPT.replace("[INSERT TASK SUMMARY]", messages[-1]["content"])

                                messages = []

                                error = False

                                logger.info(PROGRESS + "Generating ChatGPT output..." + ENDC)
                                messages = models.get_chatgpt_output(args.language_model, new_prompt, messages, "system")
                                # 🔁 STREAM TO FILE
                                # if "content" in messages[-1]:
                                #     with open(COT_LOG_PATH, "a") as f:
                                #         role = messages[-1].get("role", "Assistant").capitalize()
                                #         f.write(f"{role}:\n{messages[-1]['content']}\n\n")
                                logger.info(OK + "Finished generating ChatGPT output!" + ENDC)

                                api.failed_task = False

                            else:
                                logger.info(PROGRESS + "Generating ChatGPT output..." + ENDC)
                                logger.info(f'inside Generating: {api.count_tokens(new_prompt)}')
                                messages = models.get_chatgpt_output("gpt-4.1", new_prompt, messages, "user", COT_LOG_PATH, image_path1)
                                # 🔁 STREAM TO FILE
                                # if "content" in messages[-1]:
                                #     with open(COT_LOG_PATH, "a") as f:
                                #         role = messages[-1].get("role", "Assistant").capitalize()
                                #         f.write(f"{role}:\n{messages[-1]['content']}\n\n")
                                logger.info(OK + "Finished generating ChatGPT output!" + ENDC)
                    
                    if not os.path.exists(configs.message_path):
                        os.makedirs(configs.message_path)
                    
                    filename = os.path.join(configs.message_path, f"{command.replace(' ', '_')}_{idx*10 + configs.start}.txt")
                    

                    # Save only the content field from each response
                    with open(filename, "w", encoding="utf-8") as file:
                        for msg in messages:
                            if "content" in msg:  # Ensure key exists
                                file.write(f"{msg['role'].capitalize()}:\n{msg['content']}\n\n")
                    api.completed_task = False
                
                    # for every N frames, visualize the point clouds
                    # if idx > 0 and idx % 1 == 0:
                    #     o3d.visualization.draw_geometries(point_clouds)
                

                if configs.publish_path and api.global_path_final is not None:
                    nav2_planed_path = convert_to_nav2_frame(api.global_path_final.tolist())
                    save_dict = {'trigger': True, 'path': nav2_planed_path}

                    output_path = os.path.join(configs.planed_path, "global_path_raw.json")
                    
                    os.makedirs(os.path.dirname(output_path), exist_ok=True)
                    if os.path.exists(output_path):
                        os.remove(output_path)
                    with open(output_path, "w") as f:
                        json.dump(save_dict, f)
                    
                    print("✅ Trajectory saved to global_path_raw.json")
                
                    # api.global_path_final = None
                    #os._exit(0)
            else:
                print(f'Robot is in idle state')        
                #time.sleep(0.1)        
            idx += 1
        else:
            #print(f"Observation buffer is empty, waiting for new data...")
            time.sleep(0.1)
            if idx ==0:
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
                # pcd_save_path.parent.mkdir(parents=True, exist_ok=True)
                with gzip.open(pcd_save_path, "wb") as f:
                    pickle.dump(results, f)
                print(f"Saving pcd {pcd_save_path}")
                # break
    
        print("The task is completed OR the try number is exceeded")

def run_gradio_gui():
    import gradio_viewer
    gradio_viewer.demo.launch(share=False)   

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
        # self.image_sub = message_filters.Subscriber(self, Image, 'camera')
        self.image_sub = message_filters.Subscriber(self, Image, 'camera/shifted')
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, 'ouster/points')
        # self.lidar_sub = self.create_subscription(PointCloud2, 'ouster/points', self.lidar_callback_func, 10)
        self.lidarpos_sub = message_filters.Subscriber(self, TransformStamped, 'base_link_transform')

        # Caches for manual sync
        self.image_cache = message_filters.Cache(self.image_sub, 100)
        self.pose_cache = message_filters.Cache(self.lidarpos_sub, 100)
        self.scan_duration_sec = 0.1

        self.observation_buffer = observation_buffer
        self.memory_buffer = [] #
        # create callback
        ts = message_filters.ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub, self.lidarpos_sub], sync_queue_size, max_time_diff)
        ts.registerCallback(self.observation_callback) # divide to different core?

        self.timestamp, self.callback_times = 0, 0
        
        self.cv_image = None
        self.first_pose = None

        self.publishing_frequency = -1
        # Start the scene graph construction
        self.livedata_preprocessing = threading.Thread(target=self.run_livedata_processing)
        # self.scenegraph  = threading.Thread(target=self.run_scenegraph_generation)
        
        self.livedata_preprocessing.start()
        # self.scenegraph.start()

    def get_latest_before(self, cache, target_time):
        def time_to_ns(time_msg):
            if hasattr(time_msg, 'sec'):
                return time_msg.sec * 1_000_000_000 + time_msg.nanosec
            else:
                return int(time_msg.nanoseconds)
        target_ns = time_to_ns(target_time)

        closest = None
        max_time_ns = -1

        for msg, t in zip(cache.cache_msgs, cache.cache_times):
            t_ns = time_to_ns(t)
            if t_ns <= target_ns and t_ns > max_time_ns:
                closest = msg
                max_time_ns = t_ns

        return closest

    def lidar_callback_func(self, lidar_msg: PointCloud2):
        # LiDAR timestamp is the end of the scan
        scan_end_time = lidar_msg.header.stamp  # builtin_interfaces.msg.Time
        scan_end_ros_time = Time.from_msg(scan_end_time)

        # Estimate mid-scan time (e.g., for a 10Hz LiDAR, subtract 0.05s)
        scan_mid_ros_time = scan_end_ros_time - Duration(seconds=self.scan_duration_sec / 2)

        # ✅ Convert rclpy.Time to builtin_interfaces.msg.Time
        scan_mid_time_msg = scan_mid_ros_time.to_msg()

        # Query caches using correct time type
        # image_msg = self.image_cache.getElemBeforeTime(scan_mid_time_msg)
        # pose_msg = self.pose_cache.getElemBeforeTime(scan_mid_time_msg)
        image_msg = self.get_latest_before(self.image_cache, scan_mid_time_msg)
        pose_msg = self.get_latest_before(self.pose_cache, scan_mid_time_msg)

        if image_msg is None or pose_msg is None:
            self.get_logger().warn("No matching image or pose found for corrected LiDAR time.")
            return

        self.get_logger().info("Synchronized set found (corrected)")
        self.observation_callback(image_msg, lidar_msg, pose_msg)
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
    if cfg.observation_buffer_size > 0:
        observation_buffer = multiprocessing.Queue(maxsize=cfg.observation_buffer_size)
    else:
        observation_buffer = multiprocessing.Queue()

    ros_process = multiprocessing.Process(target=start_ros_node, args=(observation_buffer, args, cfg))
    # p = multiprocessing.Process(target=run_gradio_gui)
    ros_process.start()
    # p.start()
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
    command2 = "ros2 bag play /home/trailbot/bags/Indoor_3d" #2Indoor9 Outdoor #two_cones Indoor_3d Outdoor2_3D

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


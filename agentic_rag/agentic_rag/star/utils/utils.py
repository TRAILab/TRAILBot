"""
2024.01.17
工具函数文件
"""
# import sys
# sys.path.append("/home/mfyuan/local_folder/OpenGraph")

import sys
import os
# Add the parent directory to sys.path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

# sys.path.append("/home/trailbot/trail_ws/src/TRAILBot/object_nav/third_parties")
# #sys.path.append("/home/trailbot/trail_ws/src/TRAILBot/object_nav/third_parties/tokenize-anything")
# sys.path.append("/home/trailbot/trail_ws/src/TRAILBot/object_nav/third_parties/Tag2Text")

# sys.path.append("/home/trailbot/Documents/third_parties")
# sys.path.append("/home/trailbot/Documents/third_parties/Tag2Text")

import numpy as np
import cv2
import matplotlib.pyplot as plt
import torch
from pathlib import Path
from typing import List, Dict, Optional, Any
sys.path.append("original_documents/third_parties/GroundingDINO")
sys.path.append("third_parties/Tag2Text")
sys.path.append("original_documents/third_parties")
sys.path.append("original_documents/third_parties/tokenize-anything")
from tokenize_anything import model_registry

# sys.path.append("/home/mfyuan/local_folder/Tag2Text")
# sys.path.append("/home/mfyuan/local_folder/OpenGraph")
# sys.path.append("/home/mfyuan/local_folder")
# sys.path.append("/home/mfyuan/local_folder/GroundingDINO")

from agentic_rag.star.some_class.amg_class import MyAutomaticMaskGenerator
from agentic_rag.star.some_class.map_calss import DetectionList
import open3d as o3d
from collections import Counter
from sentence_transformers import SentenceTransformer, util
import spacy
from agentic_rag.star.some_class.map_calss import MapObjectList
import torch.nn.functional as F
import json
#from llama import Llama, Dialog
import faiss
import re
# import openai
from openai import OpenAI
from termcolor import colored
from tqdm import trange

try:
    # from Tag2Text.models import tag2text
    from ram.models import tag2text
    import torchvision.transforms as TS
except ImportError as e:
    print("Tag2text sub-package not found. Please check your PATH. ")
    raise e

try:
    from groundingdino.util.inference import Model
except ImportError as e:
    print("Import Error: Please install Grounded Segment Anything following the instructions in README.")
    raise e


# spacy分词时的一些先验词汇表
CONFUSED_NOUNS = ["metal", "back", "part", "row", "triangular","patch"]
INTEREST_NOUNS = ["van", "house"]
INTEREST_ADJS = ["grassy","white"]
EGO_TO_CAM: np.ndarray = np.array([
    [0, 0, 1],
    [-1, 0, 0],
    [0, -1, 0]
])
CAM_TO_EGO: np.ndarray = np.linalg.inv(EGO_TO_CAM)

def print_to_cot_log(message, log_file="cot_log.txt", color=None):
    if color:
        print(colored(message, color))
    else:
        print(message)

    if log_file is None:
        return

    with open(log_file, "a") as f:
        f.write(message + "\n")

def load_models(cfg):
    '''
    加载各个模型，传入三个大模型+SBERT
    '''
    # 加载一下ram模型
    TAG2TEXT_CHECKPOINT_PATH = cfg.tag2text_path
    print(TAG2TEXT_CHECKPOINT_PATH)
    delete_tag_index = []
    for i in range(3012, 3429):
        delete_tag_index.append(i)
    # load model
    tagging_model = tag2text(
        pretrained=TAG2TEXT_CHECKPOINT_PATH,
        image_size=384,
        vit='swin_b',
        delete_tag_index=delete_tag_index,
    ).to("cuda")
    tagging_model = tagging_model.eval().to("cuda")
    # dino模型分割
    grounding_dino_model = Model(
        model_config_path = cfg.gd_path,
        model_checkpoint_path = cfg.gd_weights,
        device="cuda"
    )
    # 使用模型和图像创建分割器

    model_type = "tap_vit_l"
    checkpoint = cfg.tap_path
    tap_model = model_registry[model_type](checkpoint=checkpoint).to("cuda")

    concept_weights = cfg.tap_merge_path
    tap_model.concept_projector.reset_weights(concept_weights)
    tap_model.text_decoder.reset_cache(max_batch_size=1000)

    # SBERT文本编码器
    #Pay attention to the model path!!!
    #sbert_model = SentenceTransformer(cfg.sbert_path)
    sbert_model = SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2').to("cuda")
    # 创建分割器
    mask_generator = MyAutomaticMaskGenerator(tagging_model=tagging_model, grounding_dino_model=grounding_dino_model, tap_model=tap_model, sbert_model=sbert_model)
    print("Congratulations! All large models have been loaded and can be used at will!")
    return mask_generator

def project(points, image, calib):
    '''
    把点云投影到图像上，输入点云，输出点云对应的图像横纵坐标
    '''
    points_homo = np.insert(points, 3, 1, axis=1).T

    #be careful since our lidar with y as front!!!!
    front_axis = 'y'
    if front_axis == 'x':
        idx = 0
    elif front_axis == 'y':
        idx = 1

    pointCloud = np.delete(points, np.where(points_homo[idx, :] < 0), axis=0)
    points_homo = np.delete(points_homo, np.where(points_homo[idx, :] < 0), axis=1)
    # 相机坐标系3D点=相机02内参*camera to lidar的变换矩阵*雷达3D点
    # print(calib['P_rect_20'])
    # print(calib['T_cam2_velo'])

    proj_lidar = calib['P_rect_20'].dot(calib['T_cam2_velo']).dot(points_homo)
    # proj_lidar = calib['P_rect_20'].dot(calib['T_cam2_velo']).dot(points_homo)
    # proj_lidar = np.linalg.inv(calib['T_cam2_velo']).dot(points_homo)[:3, :]


    # 以列为基准, 删除投影图像点中深度z<0(在投影图像后方)的点 #3xN
    cam = np.delete(proj_lidar, np.where(proj_lidar[2, :] < 0), axis=1)
    pointCloud = np.delete(pointCloud, np.where(proj_lidar[2, :] < 0), axis=0)
    # 前两行元素分布除以第三行元素(归一化到相机坐标系z=1平面)(x=x/z, y =y/z)
    cam[:2, :] /= cam[2, :]
    # 投影到图像
    IMG_H, IMG_W, _ = image.shape
    # 过滤掉不在相机上的
    u, v, z = cam
    u_out = np.logical_or(u < 0, u > IMG_W)
    v_out = np.logical_or(v < 0, v > IMG_H)
    outlier = np.logical_or(u_out, v_out)
    cam = np.delete(cam, np.where(outlier), axis=1)
    points = np.delete(pointCloud, np.where(outlier), axis=0)
    u,v,z  = cam
    pixels = np.dstack((v,u)).squeeze()
    # return points, pixels
    return points, pixels, z

def prepare_labeled_contours_merge(image, masks, detections, image_width, image_height, instance_id, valid_mask_indices):
    """
    Display object contours with merged map object IDs as labels.

    Args:
        image (np.ndarray): The original RGB image as a NumPy array.
        masks (list of np.ndarray): Binary masks for each object.
        detections (list or np.ndarray): Bounding boxes with coordinates [x_min, y_min, x_max, y_max].
        image_width (int): Width of the image.
        image_height (int): Height of the image.
        instance_id (dict): Mapping from detection index (1-based) to map object index.

    Returns:
        np.ndarray: Annotated RGB image with contours and labels drawn.
    """

    # Define distinct colors for drawing contours
    distinct_colors = [
        '#A52A2A', '#5F9EA0', '#D2691E', '#9ACD32', '#DA70D6', '#7FFFD4',
        '#FF8000', '#8000FF', '#0080FF', '#80FF00', '#FF0080', '#00FF80',
        '#FF0000', '#00FF00', '#0000FF', '#FF00FF', '#00FFFF', '#FFFF00',
        '#FF4500', '#2E8B57'
    ]
    distinct_colors_rgb = [tuple(int(color.lstrip('#')[i:i + 2], 16) for i in (0, 2, 4)) for color in distinct_colors]

    # Copy the original image for annotation
    annotated_image = image.copy()

    # Track label positions
    label_positions = []
    offset_step = 5  # Offset for small contours

    def clamp(value, min_value, max_value):
        """Clamp a value within a specified range."""
        return max(min_value, min(value, max_value))

    def find_safe_label_position(contour, bbox, is_small):
        """Find a safe label position inside or near the object contour."""
        x_min, y_min, x_max, y_max = bbox
        bbox_center = (int((x_min + x_max) / 2), int((y_min + y_max) / 2))

        if not is_small:
            return bbox_center
        return (
            clamp(x_max, 0, image_width),
            clamp(y_min - offset_step, 0, image_height)
        )

    label_dict = {}

    # Iterate over detections and masks

    for i, (mask, box) in enumerate(zip(masks, detections.xyxy)):
        if i not in valid_mask_indices:
            # print(f"Warning: Mask index {i} is not valid.")
            continue

        color = distinct_colors_rgb[instance_id.get(i) % len(distinct_colors_rgb)]
        # Validate mask
        if mask is None or mask.size == 0:
            print(f"Warning: Empty mask detected at index {i}")
            continue

        mask_uint8 = (mask.astype(np.uint8) * 255)
        if mask_uint8.ndim == 3 and mask_uint8.shape[0] == 1:
            mask_uint8 = mask_uint8.squeeze(0)

        contours, _ = cv2.findContours(mask_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            print(f"Warning: No contours found for mask at index {i}")
            continue

        x_min, y_min, x_max, y_max = map(int, box)
        bbox_width = x_max - x_min
        bbox_height = y_max - y_min
        is_small = bbox_width < 30 or bbox_height < 30

        # Draw contour
        cv2.drawContours(annotated_image, contours, -1, color, 2)

        # Find safe label position
        label_pos_x, label_pos_y = find_safe_label_position(contours[0], (x_min, y_min, x_max, y_max), is_small)

        # ⚡ Use instance_id to get the merged object id
        detection_idx = i #+ 1  # because instance_id is 1-based
        map_object_idx = instance_id.get(detection_idx, -1)
        label_text = str(map_object_idx) if map_object_idx >= 0 else "?"

        font_size = 0.4
        label_width, label_height = 36, 14

        label_positions.append((label_pos_x, label_pos_y))
        label_dict[label_text] = (
            label_pos_x,
            label_pos_y - label_height,
            label_pos_x + label_width,
            label_pos_y,
            color,
            label_pos_x,
            label_pos_y - 3
        )

    # Draw all labels
    for key in label_dict.keys():
        cv2.rectangle(
            annotated_image,
            (label_dict[key][0], label_dict[key][1]),
            (label_dict[key][2], label_dict[key][3]),
            label_dict[key][4],
            -1  # Filled rectangle
        )
        cv2.putText(
            annotated_image,
            key,
            (label_dict[key][5], label_dict[key][6]),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_size,
            (0, 0, 0),  # Black text
            1
        )

    return annotated_image

def create_object_pcd(image, pc, pixels, mask, obj_color=None) -> o3d.geometry.PointCloud:
    '''
    得到rgb的点云
    '''
    mask_for_pc = mask[pixels[:, 0].astype(int), pixels[:, 1].astype(int)]
    points = pc[mask_for_pc]
    pixels = pixels[mask_for_pc]
    colors = image[pixels[:,0].astype(int), pixels[:,1].astype(int)]/255.0
    # 对点稍加扰动以避免共线性，看不出来是雷达线条
    #points += np.random.normal(0, 4e-3, points.shape)
    # 创建一个Open3D PointCloud对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

# def create_object_pcd(image, pc, pixels, mask, obj_color=None) -> o3d.geometry.PointCloud:
#     '''
#     得到RGB的点云
#     '''
#     # Get the height and width of the mask (image)
#     height, width = mask.shape[:2]

#     # Ensure pixel values are within valid range and log invalid entries
#     pixels[:, 0] = np.clip(pixels[:, 0], 0, height - 1)
#     pixels[:, 1] = np.clip(pixels[:, 1], 0, width - 1)

#     # Debugging logs to see any strange pixel values
#     if np.any(pixels < 0):
#         print(f"Invalid pixel values (less than 0) found: {pixels[pixels < 0]}")

#     if np.any(pixels >= height) or np.any(pixels >= width):
#         print(f"Invalid pixel values (greater than mask size) found. Max values in pixels: {pixels.max(axis=0)}")

#     # Make sure the pixel indices are valid
#     try:
#         mask_for_pc = mask[pixels[:, 0].astype(int), pixels[:, 1].astype(int)]
#     except IndexError as e:
#         print(f"IndexError encountered! Pixels: {pixels}, Mask shape: {mask.shape}")
#         raise e  # Re-raise after logging

#     # Filter the point cloud using the valid mask points
#     points = pc[mask_for_pc]
#     pixels = pixels[mask_for_pc]

#     # Get colors for the valid points
#     colors = image[pixels[:, 0].astype(int), pixels[:, 1].astype(int)] / 255.0

#     # Add small noise to avoid collinearity
#     points += np.random.normal(0, 4e-3, points.shape)

#     # Create Open3D PointCloud object
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(points)
#     pcd.colors = o3d.utility.Vector3dVector(colors)
#     return pcd

# def create_object_pcd(image, pc, pixels, mask, obj_color=None) -> o3d.geometry.PointCloud:
#     '''
#     得到RGB的点云
#     '''
#     # Get the height and width of the mask (image)
#     height, width = mask.shape[:2]

#     # Ensure pixel values are within valid range
#     pixels[:, 0] = np.clip(pixels[:, 0], 0, height - 1)
#     pixels[:, 1] = np.clip(pixels[:, 1], 0, width - 1)

#     # Remove any NaN values in the pixel array
#     valid_mask = ~np.isnan(pixels).any(axis=1)  # Find rows without NaNs
#     if np.any(~valid_mask):
#         print(f"Found {np.sum(~valid_mask)} invalid (NaN) pixel entries, removing them.")

#     # Keep only valid pixels and corresponding point cloud points
#     pixels = pixels[valid_mask]
#     pc = pc[valid_mask]

#     # Make sure the pixel indices are valid
#     try:
#         mask_for_pc = mask[pixels[:, 0].astype(int), pixels[:, 1].astype(int)]
#     except IndexError as e:
#         print(f"IndexError encountered! Pixels: {pixels}, Mask shape: {mask.shape}")
#         raise e  # Re-raise after logging

#     # Filter the point cloud using the valid mask points
#     points = pc[mask_for_pc]
#     pixels = pixels[mask_for_pc]

#     # Get colors for the valid points
#     colors = image[pixels[:, 0].astype(int), pixels[:, 1].astype(int)] / 255.0

#     # Add small noise to avoid collinearity
#     points += np.random.normal(0, 4e-3, points.shape)

#     # Create Open3D PointCloud object
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(points)
#     pcd.colors = o3d.utility.Vector3dVector(colors)
#     return pcd



def pcd_denoise_dbscan(pcd: o3d.geometry.PointCloud, eps=0.01, min_points=10) -> o3d.geometry.PointCloud:
    '''
    通过聚类，移除噪点
    '''
    pcd_clusters = pcd.cluster_dbscan(
        eps=eps,
        min_points=min_points,
    )
    # 转换为 numpy 数组
    obj_points = np.asarray(pcd.points)
    obj_colors = np.asarray(pcd.colors)
    pcd_clusters = np.array(pcd_clusters)
    # 统计群组中的所有标签
    counter = Counter(pcd_clusters)
    # 去除噪音标签
    if counter and (-1 in counter):
        del counter[-1]
    if counter:
        # 找出最大集群的标签
        most_common_label, _ = counter.most_common(1)[0]
        # 为最大群组中的点创建掩码
        largest_mask = pcd_clusters == most_common_label
        # 应用 mask
        largest_cluster_points = obj_points[largest_mask]
        largest_cluster_colors = obj_colors[largest_mask]
        # 如果最大群集太小，则返回原始点云
        if len(largest_cluster_points) < 5:
            return pcd
        # 创建新的 PointCloud 对象
        largest_cluster_pcd = o3d.geometry.PointCloud()
        largest_cluster_pcd.points = o3d.utility.Vector3dVector(largest_cluster_points)
        largest_cluster_pcd.colors = o3d.utility.Vector3dVector(largest_cluster_colors)
        pcd = largest_cluster_pcd
    return pcd

def process_pcd(cfg, pcd, use_db = True):
    '''
    对pcd降噪处理，剔除离群点
    '''
    # 以2.5cm降采样
    pcd = pcd.voxel_down_sample(voxel_size=cfg.voxel_size)
    # dubug，降噪前后对比
    # o3d.visualization.draw_geometries([pcd])
    # 进行降噪，但是这种降噪方式对于点云不太好
    if cfg.dbscan_remove_noise and use_db:
        # cl, index = pcd.remove_statistical_outlier(nb_neighbors=50,std_ratio=1.0)
        # pcd = pcd.select_by_index(index)
        pcd = pcd_denoise_dbscan(
            pcd,
            eps=cfg.dbscan_eps,
            min_points=cfg.dbscan_min_points
        )
    # o3d.visualization.draw_geometries([pcd])
    return pcd



def get_bounding_box(pcd):
    '''
    得到点云的bbox
    '''
    # 推荐使用定向的
    if len(pcd.points) >= 4:
        try:
            return pcd.get_oriented_bounding_box(robust=True)
        except RuntimeError as e:
            print(f"Met {e}, use axis aligned bounding box instead")
            return pcd.get_axis_aligned_bounding_box()
    else:
        return pcd.get_axis_aligned_bounding_box()

def gobs_to_detection_list_2(
    cfg,
    image,
    pc,
    pixels,
    idx,
    gobs,
    trans_pose = None,
    bg_fts = None,
    BG_CAPTIONS_Pro = None,
):
    '''
    从gobs返回一个DetectionList对象,所有对象在当前帧中。
    '''
    detection_lists = DetectionList()
    bg_list = DetectionList()
    # 没有数据则返回空的
    if len(gobs) == 0:
        return detection_lists, bg_list
    n_masks = len(gobs)
    # 对每个mask处理
    SoM = None
    mask_dic = {}
    valid_mask_indices = []
    bg_indices = []
    detect_indices = []

    for mask_idx in range(n_masks):
        mask = gobs[mask_idx]['mask'].squeeze()
        #print(f"mask shape: {mask.shape}")

        caption = gobs[mask_idx]['caption']
        caption_ft = gobs[mask_idx]['caption_ft']
        img_bbox = gobs[mask_idx]['img_bbox']
        # 得到pcd
        camera_object_pcd = create_object_pcd(
            image,
            pc,
            pixels,
            mask,
            obj_color = None
        )
        # 实例设置随机颜色
        color = np.random.random(3)
        # 这个对象最少得5个点吧，否则不要也罢
        if len(camera_object_pcd.points) < max(cfg.min_points_threshold, 5):#
            continue
        if trans_pose is not None:
            global_object_pcd = camera_object_pcd.transform(trans_pose)
        else:
            global_object_pcd = camera_object_pcd
        valid_mask_indices.append(mask_idx)
        # 获取最大群，以过滤噪音
        global_object_pcd = process_pcd(cfg, global_object_pcd)

        pcd_bbox = get_bounding_box(global_object_pcd)
        pcd_bbox.color = [0,1,0]
        # 如果物体太小了，也要删掉
        # if pcd_bbox.volume() < 1e-6:
        #     continue
        bg_class = None

        mask_dic[mask_idx+1] = mask_idx+1
        # 如果使用背景的话，比较与背景的相似度是否超过阈值
        if cfg.use_bg:
            caption_ft_cuda = caption_ft.to("cuda")
            for i in range(len(bg_fts)):
                similarity = F.cosine_similarity(bg_fts[i], caption_ft_cuda, dim=-1)
                if similarity > cfg.bg_rate:
                    bg_class = BG_CAPTIONS_Pro[i]
                    SoM = None
                    #SoM = create_SoM_road(image, pc, pixels, mask, dots_size_w=10, dots_size_h=10, font_path='/home/mfyuan/local_folder/OpenGraph/config/arial.ttf')
                    # SoM = create_SoM_keypts(image, pc, mask, pixels, font_path='/home/mfyuan/local_folder/OpenGraph/config/arial.ttf')
                    # 有高的就直接跳出去
                    break
        # 把这个物体存储下来吧
        detected_object = {
            'image_idx' : [idx],                             # which image is it from, be careful with the stride
            'num_detections' : 1,                            # how many times this object is detected
            'n_points': len(global_object_pcd.points),       # 这个物体的点数量
            "inst_color": color,                             # 该段实例使用的随机颜色，后面按照semantickitti赋值
            "bg_class": bg_class,                            # 该段实例属于哪个背景，不是背景则不需要
            # 下面这些事针对全局地图中物体的，因为这个有可能是新物体
            'class_sk':None,                                 # 该实例的类别，后面可视化会用
            'caption':caption,                               # 该实例的caption，后面会融合
            'captions_ft':None,                              # 该实例的融合后的caption的编码特征
            'ft':caption_ft,                                 # caption的编码结果
            'pcd': global_object_pcd,                        # 点云pcd
            'bbox': pcd_bbox,                                # 该实例的bbox
            'img_bbox': mask_idx+1,                            # 该实例的img_bbox
            }
        #print(f'image_idx {idx}, caption {caption}, bbox {pcd_bbox}')
        # 分类归纳
        if cfg.use_bg and bg_class is not None:
            bg_list.append(detected_object)
            bg_indices.append(mask_idx)
        else:
            detection_lists.append(detected_object)
            detect_indices.append(mask_idx)
    print(f"the valid indices are {valid_mask_indices}, bg indices are {bg_indices}, detect indices are {detect_indices}")
                # detection_lists.append(detected_object)
    #print(f'---------------------above is for image {idx}---------------------')
    return detection_lists, bg_list, SoM, mask_dic, detect_indices

def gobs_to_detection_list(
    cfg,
    image,
    pc,
    pixels,
    idx,
    gobs,
    trans_pose = None,
    bg_fts = None,
    BG_CAPTIONS_Pro = None,
):
    '''
    从gobs返回一个DetectionList对象,所有对象在当前帧中。
    '''
    detection_lists = DetectionList()
    bg_list = DetectionList()
    # 没有数据则返回空的
    if len(gobs) == 0:
        return detection_lists, bg_list
    n_masks = len(gobs)
    print(f"Processing {n_masks} masks for image index {idx}...")
    # 对每个mask处理
    for mask_idx in range(n_masks):
        mask = gobs[mask_idx]['mask'].squeeze()
        # print(f"mask shape: {mask.shape}")

        caption = gobs[mask_idx]['caption']
        caption_ft = gobs[mask_idx]['caption_ft']
        img_bbox = gobs[mask_idx]['img_bbox']
        # 得到pcd
        camera_object_pcd = create_object_pcd(
            image,
            pc,
            pixels,
            mask,
            obj_color = None
        )
        # 实例设置随机颜色
        color = np.random.random(3)
        # 这个对象最少得5个点吧，否则不要也罢
        if len(camera_object_pcd.points) < max(cfg.min_points_threshold, 5):
            continue
        if trans_pose is not None:
            global_object_pcd = camera_object_pcd.transform(trans_pose)
        else:
            global_object_pcd = camera_object_pcd
        # 获取最大群，以过滤噪音
        global_object_pcd = process_pcd(cfg, global_object_pcd)
        pcd_bbox = get_bounding_box(global_object_pcd)
        pcd_bbox.color = [0,1,0]
        # 如果物体太小了，也要删掉
        if pcd_bbox.volume() < 1e-6:
            continue
        bg_class = None
        # 如果使用背景的话，比较与背景的相似度是否超过阈值
        if cfg.use_bg:
            caption_ft_cuda = caption_ft.to("cuda")
            for i in range(len(bg_fts)):
                similarity = F.cosine_similarity(bg_fts[i], caption_ft_cuda, dim=-1)
                if similarity > cfg.bg_rate:
                    bg_class = BG_CAPTIONS_Pro[i]
                    # 有高的就直接跳出去
                    break
        # 把这个物体存储下来吧
        detected_object = {
            'image_idx' : [idx],                             # 哪个图像看到的，注意stride倍数关系
            'num_detections' : 1,                            # 这个物体的检测数量，现在一个物体所以是1
            'n_points': len(global_object_pcd.points),       # 这个物体的点数量
            "inst_color": color,                             # 该段实例使用的随机颜色，后面按照semantickitti赋值
            "bg_class": bg_class,                            # 该段实例属于哪个背景，不是背景则不需要
            # 下面这些事针对全局地图中物体的，因为这个有可能是新物体
            'class_sk':None,                                 # 该实例的类别，后面可视化会用
            'caption':caption,                               # 该实例的caption，后面会融合
            'captions_ft':None,                              # 该实例的融合后的caption的编码特征
            'ft':caption_ft,                                 # caption的编码结果
            'pcd': global_object_pcd,                        # 点云pcd
            'bbox': pcd_bbox,                                # 该实例的bbox
            #'img_bbox': img_bbox,                            # 该实例的img_bbox
            'img_bbox': mask_idx+1,
            }
        #print(f'image_idx {idx}, caption {caption}, bbox {pcd_bbox}')
        # 分类归纳
        if cfg.use_bg and bg_class is not None:
            bg_list.append(detected_object)
        else:
            detection_lists.append(detected_object)
    #print(f'---------------------above is for image {idx}---------------------')
    return detection_lists, bg_list


def gobs_to_detection_list_depth(
    cfg,
    image,
    depth_array,
    cam_K_depth,
    cam_K_rgb,
    cam_R,
    idx,
    gobs,
    trans_pose = None,
    bg_fts = None,
    BG_CAPTIONS_Pro = None,
):
    '''
    Return a DetectionList object from the gobs
    All object are still in the camera frame.
    '''

    fg_detection_list = DetectionList()
    bg_detection_list = DetectionList()

    SoM = None
    mask_dic = {}
    detected_indices = []
    valid_mask_indices = []
    bg_indices = []
    detected_indices = []
    # gobs = resize_gobs(gobs, image)
    # gobs = filter_gobs(cfg, gobs, image, BG_CLASSES)

    if len(gobs) == 0:
        return fg_detection_list, bg_detection_list
    n_masks = len(gobs)
    for mask_idx in range(n_masks):
        # local_class_id = gobs['class_id'][mask_idx]
        mask = gobs[mask_idx]['mask'].squeeze()
        # print(f"mask shape: {mask.shape}")

        caption = gobs[mask_idx]['caption']
        caption_ft = gobs[mask_idx]['caption_ft']
        # class_name = gobs['classes'][local_class_id]
        # global_class_id = -1 if class_names is None else class_names.index(class_name)

        # make the pcd and color it
        camera_object_pcd = create_object_pcd_with_extrinsics(
            depth_array,
            mask,
            cam_K_depth,
            image,
            cam_K_rgb,
            cam_R,
        )

        # o3d.visualization.draw_geometries([camera_object_pcd])

        color = np.random.random(3)
        # It at least contains 5 points
        if len(camera_object_pcd.points) < max(cfg.min_points_threshold, 5):
            continue

        if trans_pose is not None:
            global_object_pcd = camera_object_pcd.transform(trans_pose)
        else:
            global_object_pcd = camera_object_pcd

        global_points = np.asarray(camera_object_pcd.points)
        mask = global_points[:, 2] <= 4.5
        global_object_pcd.points = o3d.utility.Vector3dVector(global_points[mask])
        colors = np.asarray(global_object_pcd.colors)
        global_object_pcd.colors = o3d.utility.Vector3dVector(colors[mask])

        valid_mask_indices.append(mask_idx)
        # get largest cluster, filter out noise
        global_object_pcd = process_pcd(cfg, global_object_pcd)
        pcd_bbox = get_bounding_box(global_object_pcd)
        pcd_bbox.color = [0,1,0]

        if pcd_bbox.volume() < 1e-6:
            continue
        bg_class = None
        mask_dic[mask_idx+1] = mask_idx+1
        # Treat the detection in the same way as a 3D object
        # Store information that is enough to recover the detection

        if cfg.use_bg:
            caption_ft_cuda = caption_ft.to("cuda")
            for i in range(len(bg_fts)):
                similarity = F.cosine_similarity(bg_fts[i], caption_ft_cuda, dim=-1)
                if similarity > cfg.bg_rate:
                    bg_class = BG_CAPTIONS_Pro[i]
                    break

        detected_object = {
            'image_idx' : [idx],                             # idx of the image
            'num_detections': 1,                            # number of detections in this object
            'n_points': [len(global_object_pcd.points)],
            "inst_color": color,
            "bg_class": bg_class,
            'class_sk':None,
            'caption':caption,
            'captions_ft':None,
            'ft':caption_ft,
            'pcd': global_object_pcd,
            'bbox': pcd_bbox,
            'img_bbox': mask_idx+1,
        }

        # o3d.visualization.draw_geometries([global_object_pcd, pcd_bbox])

        if cfg.use_bg and bg_class is not None:
            bg_detection_list.append(detected_object)
            bg_indices.append(mask_idx)
        else:
            fg_detection_list.append(detected_object)
            detected_indices.append(mask_idx)
    print(f"the valid indices are {valid_mask_indices}, bg indices are {bg_indices}, detect indices are {detected_indices}")

    return fg_detection_list, bg_detection_list, SoM, mask_dic, detected_indices


def denoise_objects(cfg, objects: MapObjectList, bg=False):
    '''
    整个地图去噪
    '''
    for i in range(len(objects)):
        og_object_pcd = objects[i]['pcd']
        if bg:
            objects[i]['pcd'] = process_pcd(cfg, objects[i]['pcd'], use_db=False)
        else:
            objects[i]['pcd'] = process_pcd(cfg, objects[i]['pcd'], use_db=True)
        if len(objects[i]['pcd'].points) < 4:
            objects[i]['pcd'] = og_object_pcd
            continue
        objects[i]['bbox'] = get_bounding_box(objects[i]['pcd'])
        objects[i]['bbox'].color = [0,1,0]
    return objects



def filter_objects(cfg, objects: MapObjectList):
    '''
    后处理，移除掉太少点云的物体和太少观测的物体
    '''
    #print("Before final map filtering:", len(objects))
    objects_to_keep = []
    for obj in objects:
        if len(obj['pcd'].points) >= cfg.obj_min_points and obj['num_detections'] >= cfg.obj_min_detections:
            objects_to_keep.append(obj)
    objects = MapObjectList(objects_to_keep)
    #print("After final map filtering: ", len(objects))
    return objects

def compute_3d_iou(bbox1, bbox2, padding=0, use_iou=True):
    '''
    计算3diou的占比，太小了就不进行评判最后的融合了
    '''
    # 获取第一个包围盒的坐标
    bbox1_min = np.asarray(bbox1.get_min_bound()) - padding
    bbox1_max = np.asarray(bbox1.get_max_bound()) + padding
    # 获取第二个包围盒的坐标
    bbox2_min = np.asarray(bbox2.get_min_bound()) - padding
    bbox2_max = np.asarray(bbox2.get_max_bound()) + padding
    # 计算两个边界框的重叠部分
    overlap_min = np.maximum(bbox1_min, bbox2_min)
    overlap_max = np.minimum(bbox1_max, bbox2_max)
    overlap_size = np.maximum(overlap_max - overlap_min, 0.0)
    overlap_volume = np.prod(overlap_size)
    bbox1_volume = np.prod(bbox1_max - bbox1_min)
    bbox2_volume = np.prod(bbox2_max - bbox2_min)
    obj_1_overlap = overlap_volume / bbox1_volume
    obj_2_overlap = overlap_volume / bbox2_volume
    max_overlap = max(obj_1_overlap, obj_2_overlap)
    iou = overlap_volume / (bbox1_volume + bbox2_volume - overlap_volume)
    if use_iou:
        return iou
    else:
        return max_overlap


def compute_overlap_matrix(cfg, objects: MapObjectList):
    '''
    用最近邻点计算对象间的成对重叠。假设我们有一个包含n个点云的列表，每个点云都是一个o3d.geometry.PointCloud对象。
    现在，我们要构建一个大小为nxn的矩阵，其中(i, j)条目是点云i中的点与任意一个点的距离在阈值范围内的点与点云j中任意点的距离阈值的比率。
    '''
    n = len(objects)
    overlap_matrix = np.zeros((n, n))
    # 将点云转换为 numpy 数组，然后转换为 FAISS 索引，以便高效搜索
    point_arrays = [np.asarray(obj['pcd'].points, dtype=np.float32) for obj in objects]
    indices = [faiss.IndexFlatL2(arr.shape[1]) for arr in point_arrays]
    # 将 numpy 数组中的点添加到相应的 FAISS 索引中
    for index, arr in zip(indices, point_arrays):
        index.add(arr)
    # 计算成对重叠
    for i in range(n):
        for j in range(n):
            if i != j:  # 跳过对角线元素
                box_i = objects[i]['bbox']
                box_j = objects[j]['bbox']
                # 如果方框完全不重叠，则跳过（节省计算）
                iou = compute_3d_iou(box_i, box_j)
                if iou == 0:
                    continue
                # 使用range_search查找阈值范围内的点
                # _, I = indices[j].range_search(point_arrays[i], threshold ** 2)
                D, I = indices[j].search(point_arrays[i], 1)
                # 如果在阈值范围内发现任何点，则增加重叠计数
                # overlap += sum([len(i) for i in I])
                overlap = (D < cfg.voxel_size ** 2).sum() # D 是距离的平方
                # 计算阈值内点的比率
                overlap_matrix[i, j] = overlap / len(point_arrays[i])
    return overlap_matrix


def to_numpy(tensor):
    '''
    转为numpy
    '''
    if isinstance(tensor, np.ndarray):
        return tensor
    return tensor.detach().cpu().numpy()


def to_tensor(numpy_array, device=None):
    '''
    转为tensor
    '''
    if isinstance(numpy_array, torch.Tensor):
        return numpy_array
    if device is None:
        return torch.from_numpy(numpy_array)
    else:
        return torch.from_numpy(numpy_array).to(device)

def merge_overlap_objects(cfg, objects: MapObjectList, overlap_matrix: np.ndarray):
    '''
    最后后处理，融合重叠物体
    '''
    x, y = overlap_matrix.nonzero()
    overlap_ratio = overlap_matrix[x, y]
    sort = np.argsort(overlap_ratio)[::-1]
    x = x[sort]
    y = y[sort]
    overlap_ratio = overlap_ratio[sort]
    kept_objects = np.ones(len(objects), dtype=bool)
    for i, j, ratio in zip(x, y, overlap_ratio):
        ft_sim = F.cosine_similarity(
            to_tensor(objects[i]['ft']),
            to_tensor(objects[j]['ft']),
            dim=0
        )
        if ratio > cfg.merge_overlap_thresh and ft_sim > cfg.merge_ft_thresh:
                if kept_objects[j]:
                    # 然后将对象 i 并入对象 j
                    from utils.merge import merge_obj2_into_obj1
                    objects[j] = merge_obj2_into_obj1(cfg, objects[j], objects[i])
                    kept_objects[i] = False
        else:
            break
    # 删除已合并的对象
    new_objects = [obj for obj, keep in zip(objects, kept_objects) if keep]
    objects = MapObjectList(new_objects)
    return objects

def merge_objects(cfg, objects: MapObjectList):
    '''
    后处理，最后融合一次重叠度太高的
    '''
    if cfg.merge_final:
        overlap_matrix = compute_overlap_matrix(cfg, objects)
        print("Before final map fusion:", len(objects))
        objects = merge_overlap_objects(cfg, objects, overlap_matrix)
        print("After Final Map Fusion:", len(objects))
    return objects


def transform_point_cloud(past_point_clouds, from_pose, to_pose):
    '''
    把tensor点云正常映射到全局坐标
    '''
    transformation = torch.Tensor(np.linalg.inv(to_pose) @ from_pose)
    NP = past_point_clouds.shape[0]
    xyz1 = torch.hstack([past_point_clouds, torch.ones(NP, 1)]).T
    past_point_clouds = (transformation @ xyz1).T[:, :3]
    return past_point_clouds


def timestamp_tensor(tensor, time):
    '''
    增加时间作为增加的一列，用与判断动态与否
    '''
    n_points = tensor.shape[0]
    time = time * torch.ones((n_points, 1))
    timestamped_tensor = torch.hstack([tensor, time])
    return timestamped_tensor


def accumulate_pc(cfg, mos_model, pc, pose, his_pcs, his_poses):
    '''
    输入当前帧的点云、位姿和历史累计帧的点云和位姿
    '''
    # 不需要强度值
    pc = pc[:,:3]
    his_pcs = [arr[:,:3] for arr in his_pcs]
    # all_pcs和all_poses按照时间顺序反着来[9,8,7,...,0]，其中9对应当前帧
    all_pcs = []
    all_poses = []
    # 将当前帧的点云和位姿插入到列表的开头
    all_pcs.insert(0, pc)
    all_pcs.extend(his_pcs)
    all_poses.insert(0, pose)
    all_poses.extend(his_poses)
    if cfg.filter_dynamic:
        # his_pcs和his_poses都是按照时间顺序来[0,1,2,...,9]，其中9对应当前帧
        his_pcs_copy = all_pcs[:]
        his_poses_copy = all_poses[:]
        his_pcs_copy.reverse()
        his_poses_copy.reverse()
        his_pcs_copy = [torch.tensor(arr) for arr in his_pcs_copy]
        list_his_pcs = his_pcs_copy
        # 把位姿对齐
        inv_frame0 = np.linalg.inv(his_poses_copy[0])
        new_poses = []
        for pose in his_poses_copy:
            new_poses.append(inv_frame0.dot(pose))
        poses = np.array(new_poses)
        # 计算这最近十帧点云的动态物体
        for i, pcd in enumerate(list_his_pcs):
            from_pose = poses[i]
            to_pose = poses[-1]
            pcd = transform_point_cloud(pcd, from_pose, to_pose)
            time_index = i - cfg.stride + 1
            timestamp = round(time_index * 0.1, 3)
            list_his_pcs[i] = timestamp_tensor(pcd, timestamp)
        past_point_clouds = torch.cat(list_his_pcs, dim=0)
        past_point_clouds = past_point_clouds.to('cuda')
        past_point_clouds_list = []
        past_point_clouds_list.append(past_point_clouds)
        out = mos_model.forward(past_point_clouds_list)
        for step in range(cfg.stride):
            coords = out.coordinates_at(0)
            logits = out.features_at(0)
            t = round(-step * 0.1, 3)
            mask = coords[:, -1].isclose(torch.tensor(t))
            masked_logits = logits[mask]
            masked_logits[:, [0]] = -float("inf")
            pred_softmax = F.softmax(masked_logits, dim=1)
            pred_softmax = pred_softmax.detach().cpu().numpy()
            assert pred_softmax.shape[1] == 3
            assert pred_softmax.shape[0] >= 0
            sum = np.sum(pred_softmax[:, 1:3], axis=1)
            assert np.isclose(sum, np.ones_like(sum)).all()
            moving_confidence = pred_softmax[:, 2]
            # colors = np.zeros((all_pcs[step].shape[0], 3))
            # moving_mask = moving_confidence > cfg.moving_thre
            # colors[moving_mask] = [1, 0, 0]  # Set moving points to red
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(all_pcs[step])
            # pcd.colors = o3d.utility.Vector3dVector(colors)
            # o3d.visualization.draw_geometries([pcd])
            # 按照阈值判断哪些是动态物体
            moving_mask = moving_confidence < cfg.moving_thre
            all_pcs[step] = all_pcs[step][moving_mask]
            # print((moving_confidence > cfg.moving_thre).sum().item())
            # colors = np.zeros((all_pcs[step].shape[0], 3))
            # pcd = o3d.geometry.PointCloud()
            # pcd.points = o3d.utility.Vector3dVector(all_pcs[step])
            # pcd.colors = o3d.utility.Vector3dVector(colors)
            # o3d.visualization.draw_geometries([pcd])
            # print(moving_confidence.shape)
            # print((moving_confidence > cfg.moving_thre).sum().item())
            # print(all_pcs[step].shape)
            # all_pcs[step] = all_pcs[step][moving_mask]
            # print(all_pcs[step].shape)
    pose_inv = np.linalg.inv(all_poses[0])
    for i in range(len(all_poses)):
        if i == 0:
            accumulate_pcs = all_pcs[0]
        else:
            # 计算相对位姿
            pose_rel = np.dot(pose_inv, all_poses[i])
            # 将点云的坐标添加一列，变成齐次坐标
            homogeneous_points = np.column_stack((all_pcs[i], np.ones(all_pcs[i].shape[0])))
            transformed_points = np.dot(homogeneous_points, pose_rel.T)
            # 去掉最后一列，得到新的点云坐标
            transformed_points = transformed_points[:, :3]
            accumulate_pcs = np.vstack((accumulate_pcs, transformed_points))
    return accumulate_pcs


def distance_filter(max_depth, pc):
    '''
    过滤掉深度值太大的点云
    '''
    # 计算每个点的距离
    distances = np.linalg.norm(pc, axis=1)
    # 筛选出距离在max_depth之内的点
    filtered_points = pc[distances <= max_depth]
    return filtered_points


def caption_extract(idx, spacy_nlp, caption_ori):
    # 使用spaCy处理句子
    doc = spacy_nlp(str(caption_ori))
    tokens = [token.text for token in doc]
    main_noun = "none"
    main_adj = []
    extra_captions = []

    # 获取名词
    nouns = [token.text for token in doc if token.pos_ == "NOUN"]
    # 获取形容词
    adjectives = [token.text for token in doc if token.pos_ == "ADJ"]

    # 将名词属性中的第一个名词视为关键词 但是要排除一些混淆概念in confused_nouns
    for i in range(len(nouns)):
        if nouns[i] not in CONFUSED_NOUNS:
            main_noun = nouns[i]
            break
    for token in tokens:
        if token in INTEREST_NOUNS:
            main_noun = token
            break

    # 记录当前提取的主语在token中idx
    main_noun_idx = tokens.index(main_noun)

    # 只有idx位于主语之前的形容词保留
    for adj in adjectives:
        if (tokens.index(adj) < main_noun_idx) and (len(main_adj)<2):
            main_adj += [adj]

    # 如果没有提取到有效的形容词 那得看看是不是遗漏了一些
    if not main_adj:
        for token in tokens:
            if token in INTEREST_ADJS and (len(main_adj)<2) and (tokens.index(token) < main_noun_idx):
                main_adj += [token]


    extra_captions = main_adj + [main_noun]
    extra_captions = " ".join(extra_captions)
    # print(f"Extracted Captions {idx} as: {extra_captions}")
    return extra_captions

def class_objects(cfg, sbert_model, objects: MapObjectList, bg_objects: MapObjectList, generator):
    '''
    按照caption和ft给物体分类semantickitti的类别，并设置inst_color
    '''
    # 加载语义和颜色文件
    file_path = cfg.class_colors_json
    with open(file_path, 'r') as json_file:
        class_colors_sk_disk = json.load(json_file)
        class_names_sk = list(class_colors_sk_disk.keys())
        class_colors_sk = list(class_colors_sk_disk.values())
        class_colors_sk = [list(map(lambda x: x / 255.0 if isinstance(x, (int, float)) else x, color)) for color in class_colors_sk]
    if cfg.class_methods == "sbert" or cfg.class_methods == "llama" or cfg.class_methods == "chatgpt":
        # 计算所有class的特征，不仅sbert用，对于llama以及gpt没有输出对的也能用
        class_name_fts = None
        for class_name in class_names_sk:
            class_name_ft = sbert_model.encode(class_name, convert_to_tensor=True)
            class_name_ft = class_name_ft / class_name_ft.norm(dim=-1, keepdim=True)
            class_name_ft = class_name_ft.squeeze()
            if class_name_fts is None:
                class_name_fts = class_name_ft
            else:
                class_name_fts = torch.vstack((class_name_fts,class_name_ft))
    if cfg.class_methods == "sbert":
        # 是否先spacy分词在进行相似性判断
        if cfg.spacy:
            # 加载英语模型
            spacy_nlp = spacy.load("en_core_web_sm")
            print("Spacy English loaded successfully! Ready for caption extraction!")
        # 为每个物体找最相似的语义类别，记录颜色
        for i in range(len(objects)): #(trange)
            # 先使用最后的caption计算特征
            caption = objects[i]['caption']
            if cfg.spacy:
                caption = caption_extract(i, spacy_nlp, caption)
            caption_only_ft = sbert_model.encode(caption, convert_to_tensor=True)
            caption_only_ft = caption_only_ft / caption_only_ft.norm(dim=-1, keepdim=True)
            caption_only_ft = caption_only_ft.squeeze()
            # 再使用融合的caption_ft
            objects_sbert_fts = objects[i]["ft"]
            objects_sbert_fts = objects_sbert_fts.to("cuda")
            # 两个加权融合
            final_ft = caption_only_ft*cfg.vis_caption_weight+objects_sbert_fts*cfg.vis_ft_weight
            # 与class计算相似性
            similarities = F.cosine_similarity(class_name_fts, final_ft.unsqueeze(0), dim=-1)
            if cfg.spacy and cfg.caption_only:
                similarities = F.cosine_similarity(class_name_fts, caption_only_ft.unsqueeze(0), dim=-1)
            max_indices = torch.argmax(similarities)
            # 设置好类别和颜色
            objects[i]['class_sk'] = class_names_sk[max_indices]
            objects[i]['inst_color'] = class_colors_sk[max_indices]
        if bg_objects is not None:
            for i in range(len(bg_objects)):#trange
                # 先使用最后的caption计算特征
                caption = bg_objects[i]['caption']
                if cfg.spacy:
                    caption = caption_extract(i, spacy_nlp, caption)
                caption_only_ft = sbert_model.encode(caption, convert_to_tensor=True)
                caption_only_ft = caption_only_ft / caption_only_ft.norm(dim=-1, keepdim=True)
                caption_only_ft = caption_only_ft.squeeze()
                # 再使用融合的caption_ft
                objects_sbert_fts = bg_objects[i]["ft"]
                objects_sbert_fts = objects_sbert_fts.to("cuda")
                # 两个加权融合
                final_ft = caption_only_ft*0.5+objects_sbert_fts*0.5
                # 与class计算相似性
                similarities = F.cosine_similarity(class_name_fts, final_ft.unsqueeze(0), dim=-1)
                if cfg.spacy and cfg.caption_only:
                    similarities = F.cosine_similarity(class_name_fts, caption_only_ft.unsqueeze(0), dim=-1)
                max_indices = torch.argmax(similarities)
                # 设置好类别和颜色
                bg_objects[i]['class_sk'] = class_names_sk[max_indices]
                bg_objects[i]['inst_color'] = class_colors_sk[max_indices]
    elif cfg.class_methods == "llama":
        # 用作示范的prompt example
        caption_example1 = "a car parked on the street"
        caption_example2 = "a red and white sign"
        caption_example3 = "grass on the side of the road"
        caption_example4 = "a sign on a pole"
        DEFAULT_PROMPT = """
        You are a classifier that can categorize a caption phrase into one of the following categories based on a caption phrase.
        List of categories: [car, bicycle, motorcycle, truck, person, bicyclist, motorcyclist, road,
        parking, sidewalk, building, fence, vegetation, trunk, terrain, pole, traffic-sign].
        You only need to generate one category name which must be included in this list.
        The output format is 'Category name: [[your summarized category name itself]]'
        Emphasizing again: Do not provide words beyond the given list!!! Please test it yourself and regenerate it if it exceeds the list.
        """
        for i in trange(len(objects)):
            caption_obj = objects[i]["caption"]
            # 生成llama对话
            dialogs: List[Dialog] = [
                [{"role": "system",
                "content": DEFAULT_PROMPT}
                ,{"role": "user", "content": caption_example1}
                ,{"role": "assistant", "content": "Category name: [car]"}
                ,{"role": "user", "content": caption_example2}
                ,{"role": "assistant", "content": "Category name: [traffic-sign]"}
                ,{"role": "user", "content": caption_example3}
                ,{"role": "assistant", "content": "Category name: [terrain]"}
                ,{"role": "user", "content": caption_example4}
                ,{"role": "assistant", "content": "Category name: [traffic-sign]"}
                ,{"role": "user", "content": caption_obj}],
            ]
            # llama进行回答
            results = generator.chat_completion(
                dialogs,  # type: ignore
                max_gen_len= None,
                temperature=0.6,
                top_p=0.9,
            )
            # 读取llama回答结果中的generation content作为caption融合结果
            for dialog, result in zip(dialogs, results):
                input_text = result["generation"]["content"]
                pattern = r'\[([^]]+)\]'  # 匹配方括号中的内容
                match = re.search(pattern, input_text)
                extracted_content = []
                if match:
                    extracted_content = match.group(1)
            # 如果llama生成的特征没有在给定列表中，则使用sbert特征配准
            if extracted_content not in class_colors_sk_disk:
                extracted_content_ft = sbert_model.encode(extracted_content, convert_to_tensor=True)
                extracted_content_ft = extracted_content_ft / extracted_content_ft.norm(dim=-1, keepdim=True)
                extracted_content_ft = extracted_content_ft.squeeze()
                # 与class计算相似性
                similarities = F.cosine_similarity(class_name_fts, extracted_content_ft.unsqueeze(0), dim=-1)
                max_indices = torch.argmax(similarities)
                # 设置好类别和颜色
                objects[i]['class_sk'] = class_names_sk[max_indices]
                objects[i]['inst_color'] = class_colors_sk[max_indices]
            else:
                objects[i]["class_sk"] = extracted_content
                objects[i]['inst_color'] = np.array(class_colors_sk_disk[extracted_content])/255.0
        if bg_objects is not None:
            for i in trange(len(bg_objects)):
                caption_obj = bg_objects[i]["caption"]
                # 生成llama对话
                dialogs: List[Dialog] = [
                    [{"role": "system",
                    "content": DEFAULT_PROMPT}
                    ,{"role": "user", "content": caption_example1}
                    ,{"role": "assistant", "content": "Category name: [car]"}
                    ,{"role": "user", "content": caption_example2}
                    ,{"role": "assistant", "content": "Category name: [traffic-sign]"}
                    ,{"role": "user", "content": caption_example3}
                    ,{"role": "assistant", "content": "Category name: [terrain]"}
                    ,{"role": "user", "content": caption_example4}
                    ,{"role": "assistant", "content": "Category name: [traffic-sign]"}
                    ,{"role": "user", "content": caption_obj}],
                ]
                # llama进行回答
                results = generator.chat_completion(
                    dialogs,  # type: ignore
                    max_gen_len= None,
                    temperature=0.6,
                    top_p=0.9,
                )
                # 读取llama回答结果中的generation content作为caption融合结果
                for dialog, result in zip(dialogs, results):
                    input_text = result["generation"]["content"]
                    pattern = r'\[([^]]+)\]'  # 匹配方括号中的内容
                    match = re.search(pattern, input_text)
                    extracted_content = []
                    if match:
                        extracted_content = match.group(1)
                # 如果llama生成的特征没有在给定列表中，则使用sbert特征配准
                if extracted_content not in class_colors_sk_disk:
                    extracted_content_ft = sbert_model.encode(extracted_content, convert_to_tensor=True)
                    extracted_content_ft = extracted_content_ft / extracted_content_ft.norm(dim=-1, keepdim=True)
                    extracted_content_ft = extracted_content_ft.squeeze()
                    # 与class计算相似性
                    similarities = F.cosine_similarity(class_name_fts, extracted_content_ft.unsqueeze(0), dim=-1)
                    max_indices = torch.argmax(similarities)
                    # 设置好类别和颜色
                    bg_objects[i]['class_sk'] = class_names_sk[max_indices]
                    bg_objects[i]['inst_color'] = class_colors_sk[max_indices]
                else:
                    bg_objects[i]["class_sk"] = extracted_content
                    bg_objects[i]['inst_color'] = np.array(class_colors_sk_disk[extracted_content])/255.0
    elif cfg.class_methods == "chatgpt":
        print("Asking gpt for class")
        client = OpenAI()
        # openai.api_key = cfg.openai_key
        # openai.api_base = cfg.api_base
        TIMEOUT = 25  # timeout in seconds
        DEFAULT_PROMPT = """
        You are a classifier that can categorize a caption phrase into one of the following categories based on a caption phrase.
        List of categories: [car, bicycle, motorcycle, truck, person, bicyclist, motorcyclist, road,
        parking, sidewalk, building, fence, vegetation, trunk, terrain, pole, traffic-sign]
        . You only need to generate one category name which must be included in this list.
        The output format is 'Category name: [[your summarized category name itself]]'
        Note that I may enter all the captions at the same time, please output them in order, the number of your generated category name MUST be same as the number of captions!!!
        Here's an example for you.
        Input:
        'a car parked on the street'
        'a red and white sign'
        'grass on the side of the road'.
        You should output like this:
        'Category name: [car]
        Category name: [traffic-sign]
        Category name: [terrain]
        '
        Make sure that the number of category names you output matches the number of captions; otherwise, regenerate them.
        """
        caption_objects = objects.get_stacked_str_torch("caption")
        batch_size = cfg.gpt_max_num
        num_batches = len(caption_objects) // batch_size + (len(caption_objects) % batch_size > 0)

        for batch_idx in range(num_batches):
            #print("Batch num:", batch_idx,"/",num_batches)
            start_idx = batch_idx * batch_size
            end_idx = (batch_idx + 1) * batch_size
            current_caption_batch = caption_objects[start_idx:end_idx]
            caption_obj_batch = '\n'.join(current_caption_batch)

            chat_completion = client.chat.completions.create(
                    model="gpt-4o-mini",
                    messages=[{"role": "user", "content": DEFAULT_PROMPT + "\n\n" + caption_obj_batch}],
                    timeout=TIMEOUT,  # Timeout in seconds
                    )

            # chat_completion = openai.ChatCompletion.create(
            #     model="gpt-4",
            #     messages=[{"role": "user", "content": DEFAULT_PROMPT + "\n\n" + caption_obj_batch}],
            #     timeout=TIMEOUT,  # Timeout in seconds
            # )
            input_text_batch = chat_completion.choices[0].message.content #chat_completion["choices"][0]["message"]["content"]
            #print(f'current_caption_batch {current_caption_batch}, len {len(current_caption_batch)}')
            input_text_batch = input_text_batch.split('\n')
            extracted_contents_batch = [re.search(r'\[([^]]+)\]', result).group(1) for result in input_text_batch if re.search(r'\[([^]]+)\]', result)]
            #print(f'extracted_contents_batch {extracted_contents_batch}, len {len(extracted_contents_batch)}')

            regenerated_time = 0
            while len(extracted_contents_batch) != len(current_caption_batch):
                print(f"Missing captions, regenerate {regenerated_time} times")
                PROMPT = """Your generated category names do not match the number of captions. Please regenerate them again until their numbers are the same."""
                chat_completion = client.chat.completions.create(
                    model="gpt-4o-mini",
                    messages=[{"role": "user", "content": DEFAULT_PROMPT + "\n\n" + caption_obj_batch + "\n\n" + PROMPT}],
                    timeout=TIMEOUT,  # Timeout in seconds
                    )
                input_text_batch = chat_completion.choices[0].message.content
                input_text_batch = input_text_batch.split('\n')
                extracted_contents_batch = [re.search(r'\[([^]]+)\]', result).group(1) for result in input_text_batch if re.search(r'\[([^]]+)\]', result)]
                regenerated_time += 1

            for i in range(len(current_caption_batch)):
                # 如果gpt生成的特征没有在给定列表中，则使用sbert特征配准
                extracted_content = extracted_contents_batch[i]
                if extracted_content not in class_colors_sk_disk:
                    extracted_content_ft = sbert_model.encode(extracted_content, convert_to_tensor=True)
                    extracted_content_ft = extracted_content_ft / extracted_content_ft.norm(dim=-1, keepdim=True)
                    extracted_content_ft = extracted_content_ft.squeeze()
                    # 与class计算相似性
                    similarities = F.cosine_similarity(class_name_fts, extracted_content_ft.unsqueeze(0), dim=-1)
                    max_indices = torch.argmax(similarities)
                    # 设置好类别和颜色
                    objects[start_idx+i]['class_sk'] = class_names_sk[max_indices]
                    objects[start_idx+i]['inst_color'] = class_colors_sk[max_indices]
                else:
                    objects[start_idx+i]["class_sk"] = extracted_content
                    objects[start_idx+i]['inst_color'] = np.array(class_colors_sk_disk[extracted_content])/255.0
        if bg_objects is not None:
            caption_obj = bg_objects.get_stacked_str_torch("caption")
            caption_obj = '\n'.join(caption_obj)
            chat_completion = client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[{"role": "user", "content": DEFAULT_PROMPT + "\n\n" + caption_obj}],
                timeout=TIMEOUT,  # Timeout in seconds
            )
            # chat_completion = client.chat.completions.create(
            #         model="gpt-4o-mini",
            #         messages=[{"role": "user", "content": DEFAULT_PROMPT + "\n\n" + caption_obj}],
            #         timeout=TIMEOUT,  # Timeout in seconds
            #         )
            # print('HERE', chat_completion)
            input_text = chat_completion.choices[0].message.content #chat_completion["choices"][0]["message"]["content"]
            input_text = input_text.split('\n')
            extracted_contents = [re.search(r'\[([^]]+)\]', result).group(1) for result in input_text if re.search(r'\[([^]]+)\]', result)]
            for i in range(len(bg_objects)):
                bg_objects[i]["class_sk"] = extracted_contents[i]
                bg_objects[i]['inst_color'] = np.array(class_colors_sk_disk[extracted_contents[i]])/255.0
    else:
        raise NotImplementedError
    return objects, bg_objects




def show_captions(objects: MapObjectList, bg_objects: MapObjectList):
    '''
    展示objects所对应的caption以用于debug
    '''
    for i in range(len(objects)):
        caption_obj = objects[i]["caption"]
        class_obj = objects[i]["class_sk"]
        print(f"object id {i} capitons: {caption_obj} ******** class_name: {class_obj}")
    if bg_objects is not None:
        for i in range(len(bg_objects)):
            caption_obj = bg_objects[i]["caption"]
            class_obj = bg_objects[i]["class_sk"]
            print(f"bgobject id {i} capitons: {caption_obj} ******** class_name: {class_obj}")

def get_observation_by_window(observation):
    color = observation[-1][1]
    pointCloud = observation[-1][2]
    pose = observation[-1][3]
    timestamp = observation[-1][4]

    his_pointCloud, his_pose = [], []
    for i in range(len(observation)-1):
        his_index = len(observation)-i-1
        his_pointCloud.append(observation[his_index][2])
        his_pose.append(observation[his_index][3])

    return (
        color,
        pointCloud,
        pose,
        his_pointCloud,
        his_pose,
        timestamp
    )

def get_observation(stride, observation):
    '''
    retrieve the data from the dataset, including the image, point cloud, pose, and history
    '''
    color = observation[-1][1]
    pointCloud = observation[-1][2]
    pose = observation[-1][3]

    all_pc = observation
    all_poses = observation
    # Overlapping projection of historical frames

    his_pointCloud = []
    his_pose = []
    # if False:
    for i in range(stride-1):
        his_index = stride-i-1
        #print(f"pc len {len(all_pc)} and pose len {len(all_poses)}, and his_index {his_index}")
        his_pointCloud.append(all_pc[his_index][2])
        his_pose.append(all_poses[his_index][3])

    return (
        color,
        pointCloud,
        pose,
        his_pointCloud,
        his_pose
    )

def load_calib(calib_path):
    '''
    load calibration file
    '''
    calib = {}
    with open(calib_path, "r") as calib_file:
        calib_lines = calib_file.readlines()
        # 加载相机内参
        P_rect_line = calib_lines[2]
        P_rect_02 = np.array(list(map(float, P_rect_line.strip().split()[1:]))).reshape(3, 4)
        calib["P_rect_20"] = P_rect_02
        # 加载相机外参
        Tr_line = calib_lines[4]
        Tr = np.array(list(map(float, Tr_line.strip().split()[1:]))).reshape(3, 4)
        Tr = np.vstack([Tr, [0, 0, 0, 1]])
        calib['T_cam2_velo'] = Tr
    return calib

def from_intrinsics_matrix(K: np.ndarray) -> tuple[float, float, float, float]:
    '''
    Get fx, fy, cx, cy from the intrinsics matrix

    return 4 scalars
    '''
    fx = K[0, 0]
    fy = K[1, 1]
    cx = K[0, 2]
    cy = K[1, 2]
    return fx, fy, cx, cy

import numpy as np
import open3d as o3d

def create_object_pcd_with_extrinsics(
    depth_array,                  # H_d x W_d, 深度单位：米（或在入参前先换算好）
    mask,                         # 与 depth_array 同分辨率
    K_d,                          # 深度相机内参 [fx fy cx cy] 或 3x3
    image_rgb,                    # H_c x W_c x 3, uint8
    K_c,                          # 彩色相机内参 [fx fy cx cy] 或 3x3
    T_c_from_d,                   # 4x4 齐次矩阵，把点从深度相机坐标系变换到彩色相机坐标系
) -> o3d.geometry.PointCloud:
    fx_d, fy_d, cx_d, cy_d = K_d[0,0], K_d[1,1], K_d[0,2], K_d[1,2]
    fx_c, fy_c, cx_c, cy_c = K_c[0,0], K_c[1,1], K_c[0,2], K_c[1,2]

    H_d, W_d = depth_array.shape
    H_c, W_c, _ = image_rgb.shape

    # 有效深度掩码
    valid = np.isfinite(depth_array) & (depth_array > 0)
    if mask is not None:
        valid = np.logical_and(valid, mask)

    if valid.sum() == 0:
        return o3d.geometry.PointCloud()

    # 像素网格
    u = np.arange(W_d, dtype=np.float32) # + 0.5
    v = np.arange(H_d, dtype=np.float32) # + 0.5
    uu, vv = np.meshgrid(u, v)   # H_d x W_d

    z = depth_array[valid]                         # (N,)
    u_valid = uu[valid]
    v_valid = vv[valid]

    # 1) 用深度相机内参反投影到深度相机坐标系
    Xd = (u_valid - cx_d) * z / fx_d
    Yd = (v_valid - cy_d) * z / fy_d
    Zd = z
    Pd = np.stack([Xd, Yd, Zd, np.ones_like(Zd)], axis=0)  # 4 x N

    # 2) 用外参把点变到彩色相机坐标系
    Pc = T_c_from_d @ Pd                                   # 4 x N
    Xc, Yc, Zc = Pc[0, :], Pc[1, :], Pc[2, :]

    # 3) 透视投影到彩色图像平面（像素坐标）
    #    注意：只对 Zc>0 的点有效
    positive = Zc > 0
    Xc, Yc, Zc = Xc[positive], Yc[positive], Zc[positive]
    du, dv = -3.0, 0.0 # -3.0 0
    u_c = (fx_c * Xc / Zc) + cx_c + du
    v_c = (fy_c * Yc / Zc) + cy_c + dv
    # u_c = (fx_c * Xc / Zc) + cx_c
    # v_c = (fy_c * Yc / Zc) + cy_c

    # 4) 丢弃投影到彩色图像外的点
    u_round = np.rint(u_c).astype(np.int32)
    v_round = np.rint(v_c).astype(np.int32)
    in_img = (u_round >= 0) & (u_round < W_c) & (v_round >= 0) & (v_round < H_c)

    u_round = u_round[in_img]
    v_round = v_round[in_img]

    # 对应的三维点（仍然使用深度坐标或彩色坐标都行）
    # 这里返回到“彩色相机坐标系”的点云（也可以返回深度相机坐标系：用前面的 Xd/Yd/Zd 对应过滤）
    Xc, Yc, Zc = Xc[in_img], Yc[in_img], Zc[in_img]
    points = np.stack([Xc, Yc, Zc], axis=1)  # (M, 3)
    # points = points @ CAM_TO_EGO # To ego frame
    colors = image_rgb[v_round, u_round, :].astype(np.float32) / 255.0  # (M,3)

    # height_mask = points[:, 2] <= 4.0
    # points = points[height_mask]
    # colors = colors[height_mask]

    # 构建点云
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd

def visualize_depth_on_rgb_new(
    depth_m,           # H_d x W_d, 深度(米)，0或NaN为无效
    K_d,               # 深度相机内参 3x3
    image_rgb,         # H_c x W_c x 3, uint8 (按RGB处理)
    K_c,               # 彩色相机内参 3x3
    T_c_from_d,        # 4x4, depth坐标系 -> color坐标系
    depth_clip=(0.2, 5.0),
    alpha=0.6,
    delta_uv=(0.0, 0.0),         # <- 新增：常量像素偏移补偿 (Δu, Δv)
    scale_crop=None,             # <- 可选：(sx, sy, ox, oy) 处理resize/裁剪
    undistort_maps=None          # <- 可选：(map1, map2) 若想先去畸变
):
    import numpy as np, cv2

    # ---------- 0) 可选去畸变（强烈建议先对RGB/Depth分别去畸变后再用各自K） ----------
    if undistort_maps is not None:
        map1, map2 = undistort_maps
        image_rgb = cv2.remap(image_rgb, map1, map2, interpolation=cv2.INTER_LINEAR)

    # ---------- 1) 解析/校正内参 ----------
    # 若发生了 resize/裁剪，按 (sx, sy, ox, oy) 修正 K_c
    Kc = K_c.copy()
    if scale_crop is not None:
        sx, sy, ox, oy = scale_crop  # sx,sy:尺度因子; ox,oy:裁剪偏移(像素)
        Kc[0,0] *= sx
        Kc[1,1] *= sy
        Kc[0,2] = sx * Kc[0,2] + ox
        Kc[1,2] = sy * Kc[1,2] + oy

    fx_d, fy_d, cx_d, cy_d = float(K_d[0,0]), float(K_d[1,1]), float(K_d[0,2]), float(K_d[1,2])
    fx_c, fy_c, cx_c, cy_c = float(Kc[0,0]), float(Kc[1,1]), float(Kc[0,2]), float(Kc[1,2])

    H_d, W_d = depth_m.shape
    H_c, W_c = image_rgb.shape[:2]

    # ---------- 2) 有效深度 ----------
    valid = np.isfinite(depth_m) & (depth_m > 0)
    if not np.any(valid):
        return image_rgb.copy()

    # 使用像素中心 (+0.5) 可避免半像素系统性偏移
    u = np.arange(W_d, dtype=np.float32) # + 0.5
    v = np.arange(H_d, dtype=np.float32) # + 0.5
    uu, vv = np.meshgrid(u, v)  # H_d x W_d

    z = depth_m[valid].astype(np.float32)
    u_valid = uu[valid]
    v_valid = vv[valid]

    # ---------- 3) 反投影到深度相机坐标系 ----------
    Xd = (u_valid - cx_d) * z / fx_d
    Yd = (v_valid - cy_d) * z / fy_d
    Zd = z
    ones = np.ones_like(Zd)
    Pd = np.stack([Xd, Yd, Zd, ones], axis=0)  # 4 x N

    # ---------- 4) 外参变换到彩色相机坐标系 ----------
    Pc = T_c_from_d @ Pd
    Xc, Yc, Zc = Pc[0], Pc[1], Pc[2]

    front = Zc > 0
    if not np.any(front):
        return image_rgb.copy()
    Xc, Yc, Zc = Xc[front], Yc[front], Zc[front]

    # ---------- 5) 投影到彩色图像 + 常量像素补偿 ----------
    du, dv = float(delta_uv[0]), float(delta_uv[1])  # 常量偏移（像素）
    u_c = fx_c * (Xc / Zc) + cx_c + du
    v_c = fy_c * (Yc / Zc) + cy_c + dv

    # **注意：不要对齐到像素中心，这里是像素坐标系，后面四舍五入成整数索引**
    u_i = np.round(u_c).astype(np.int32)
    v_i = np.round(v_c).astype(np.int32)

    in_img = (u_i >= 0) & (u_i < W_c) & (v_i >= 0) & (v_i < H_c)
    if not np.any(in_img):
        return image_rgb.copy()
    u_i, v_i, Zc = u_i[in_img], v_i[in_img], Zc[in_img]

    # ---------- 6) 颜色映射 ----------
    z_vis = np.clip(Zc, depth_clip[0], depth_clip[1])
    z_norm = (1.0 - (z_vis - depth_clip[0]) / (depth_clip[1] - depth_clip[0]))
    z_norm = (np.clip(z_norm, 0, 1) * 255).astype(np.uint8)
    cmap_colors = cv2.applyColorMap(z_norm, cv2.COLORMAP_JET)       # BGR
    cmap_colors = cv2.cvtColor(cmap_colors, cv2.COLOR_BGR2RGB)      # RGB
    cmap_colors = cmap_colors.reshape(-1, 3)

    # ---------- 7) Z-buffer：最近点优先 ----------
    zbuffer = np.full((H_c, W_c), np.inf, dtype=np.float32)
    overlay = np.zeros((H_c, W_c, 3), dtype=np.uint8)

    lin = v_i * W_c + u_i
    np.minimum.at(zbuffer.ravel(), lin, Zc)
    keep = Zc == zbuffer.ravel()[lin]
    if not np.any(keep):
        return image_rgb.copy()

    overlay[v_i[keep], u_i[keep]] = cmap_colors[keep]

    # ---------- 8) 叠加 ----------
    out = (alpha * overlay.astype(np.float32) + (1 - alpha) * image_rgb.astype(np.float32)).astype(np.uint8)
    return out


def visualize_depth_on_rgb(
    depth_m,           # H_d x W_d, 深度(米)，0或NaN为无效
    K_d,               # 深度相机内参 [fx, fy, cx, cy] 或 3x3
    image_rgb,         # H_c x W_c x 3, uint8 (BGR或RGB都行，下面按RGB处理)
    K_c,               # 彩色相机内参 [fx, fy, cx, cy] 或 3x3
    T_c_from_d,        # 4x4，把点从深度相机坐标系 -> 彩色相机坐标系
    depth_clip=(0.2, 5.0),   # 可视化的深度范围(米)，用于颜色映射
    alpha=0.6               # 叠加透明度
):
    # --- 1) 解析内参 ---
    fx_d, fy_d, cx_d, cy_d = K_d[0,0], K_d[1,1], K_d[0,2], K_d[1,2]
    fx_c, fy_c, cx_c, cy_c = K_c[0,0], K_c[1,1], K_c[0,2], K_c[1,2]

    H_d, W_d = depth_m.shape
    H_c, W_c = image_rgb.shape[:2]

    # --- 2) 有效深度 ---
    valid = np.isfinite(depth_m) & (depth_m > 0)
    if not np.any(valid):
        return image_rgb.copy()  # 没有有效深度就原图返回

    u = np.arange(W_d, dtype=np.float32) # + 0.5
    v = np.arange(H_d, dtype=np.float32) # + 0.5
    uu, vv = np.meshgrid(u, v)            # H_d x W_d

    z = depth_m[valid].astype(np.float32)  # (N,)
    u_valid = uu[valid]
    v_valid = vv[valid]

    # --- 3) 反投影到深度相机坐标系 ---
    Xd = (u_valid - cx_d) * z / fx_d
    Yd = (v_valid - cy_d) * z / fy_d
    Zd = z
    ones = np.ones_like(Zd)
    Pd = np.stack([Xd, Yd, Zd, ones], axis=0)  # 4 x N

    # --- 4) 用外参变到彩色相机坐标系 ---
    Pc = T_c_from_d @ Pd
    Xc, Yc, Zc = Pc[0], Pc[1], Pc[2]

    # 只保留在相机前方的点
    front = Zc > 0
    if not np.any(front):
        return image_rgb.copy()
    Xc, Yc, Zc = Xc[front], Yc[front], Zc[front]

    # --- 5) 投影到彩色图像 ---
    du, dv = -3.0, 0 #float(delta_uv[0]), float(delta_uv[1])  # 常量偏移（像素）
    # u_c = fx_c * (Xc / Zc) + cx_c + du
    # v_c = fy_c * (Yc / Zc) + cy_c + dv

    u_c = fx_c * (Xc / Zc) + cx_c + du
    v_c = fy_c * (Yc / Zc) + cy_c + dv
    u_i = np.rint(u_c).astype(np.int32)
    v_i = np.rint(v_c).astype(np.int32)

    in_img = (u_i >= 0) & (u_i < W_c) & (v_i >= 0) & (v_i < H_c)
    if not np.any(in_img):
        return image_rgb.copy()

    u_i, v_i, Zc = u_i[in_img], v_i[in_img], Zc[in_img]

    # --- 6) 为可视化做深度归一化 + 颜色映射 ---
    z_vis = np.clip(Zc, depth_clip[0], depth_clip[1])
    z_norm = (z_vis - depth_clip[0]) / (depth_clip[1] - depth_clip[0])  # 0..1
    # z_norm = 1.0 - z_norm  # 近处更亮（可根据喜好调换）
    z_norm = (z_norm * 255).astype(np.uint8)

    # 使用 OpenCV 的 colormap（JET/Rainbow等）
    cmap_colors = cv2.applyColorMap(z_norm, cv2.COLORMAP_JET)  # BGR
    cmap_colors = cv2.cvtColor(cmap_colors, cv2.COLOR_BGR2RGB) # 变成RGB
    cmap_colors = cmap_colors.reshape(-1, 3)

    # --- 7) Z-buffer：同一像素保留最近点 ---
    # 初始化一个“最小深度图”，先全设为 +inf
    zbuffer = np.full((H_c, W_c), np.inf, dtype=np.float32)
    overlay = np.zeros((H_c, W_c, 3), dtype=np.uint8)

    # 线性索引方便做原子更新
    lin = v_i * W_c + u_i

    # 对每个位置取最小深度
    np.minimum.at(zbuffer.ravel(), lin, Zc)

    # 找出这些最小深度对应的像素（最近点）
    keep = Zc == zbuffer.ravel()[lin]
    if not np.any(keep):
        return image_rgb.copy()

    u_k, v_k = u_i[keep], v_i[keep]
    colors_k = cmap_colors[keep]  # (M, 3)

    overlay[v_k, u_k] = colors_k

    # --- 8) 与原图叠加 ---
    base = image_rgb.astype(np.float32)
    over = overlay.astype(np.float32)
    out = (alpha * over + (1 - alpha) * base).astype(np.uint8)
    return out


import numpy as np
import cv2

def undistort_fisheye_equidistant6(
    image,
    K_src,                 # 原鱼眼相机内参 3x3（用于把 θ_d 变成像素半径）
    k6,                    # [k0..k5]：θ → θ_d 的 6 系数（奇次多项式）
    output_size=None,      # (w_out, h_out)
    K_out=None,            # 输出（目标）相机内参；不传则按 fov_out 生成
    fov_out_deg=100.0,     # 目标视场（对角等效近似），只在 K_out=None 时生效
    interpolation=cv2.INTER_LINEAR,
    border_mode=cv2.BORDER_CONSTANT
):
    """
    把 6 参数 equidistant (f-θ) 鱼眼图像“拉直”为透视（rectilinear/pinhole）图像。
    """
    h_src, w_src = image.shape[:2]
    if output_size is None:
        w_out, h_out = w_src, h_src
    else:
        w_out, h_out = output_size

    K_src = np.asarray(K_src, dtype=np.float64).reshape(3, 3)
    fx_src, fy_src = K_src[0, 0], K_src[1, 1]
    cx_src, cy_src = K_src[0, 2], K_src[1, 2]

    k6 = np.asarray(k6, dtype=np.float64).ravel()
    if k6.size < 6:
        k6 = np.pad(k6, (0, 6 - k6.size))
    k0, k1, k2, k3, k4, k5 = k6[:6]

    # 生成输出相机内参（透视模型）
    if K_out is None:
        # 给定对角视场，按较小尺寸求近似焦距
        diag = np.sqrt(w_out**2 + h_out**2)
        f = (diag / 2.0) / np.tan(np.deg2rad(fov_out_deg) / 2.0)
        fx_out = fy_out = f
        cx_out, cy_out = (w_out - 1) / 2.0, (h_out - 1) / 2.0
        K_out = np.array([[fx_out, 0, cx_out],
                          [0, fy_out, cy_out],
                          [0,     0,     1]], dtype=np.float64)
    else:
        K_out = np.asarray(K_out, dtype=np.float64).reshape(3, 3)

    fx_out, fy_out = K_out[0, 0], K_out[1, 1]
    cx_out, cy_out = K_out[0, 2], K_out[1, 2]

    # --- 构建输出像素网格（逆向映射） ---
    u = np.arange(w_out, dtype=np.float64)
    v = np.arange(h_out, dtype=np.float64)
    uu, vv = np.meshgrid(u, v)  # 形状 (h_out, w_out)

    # 归一化透视坐标 -> 视线方向
    x = (uu - cx_out) / fx_out
    y = (vv - cy_out) / fy_out

    # 由透视模型得到入射角：theta = arctan(r)，r = sqrt(x^2 + y^2)
    r = np.sqrt(x * x + y * y)
    theta = np.arctan(r)  # 与光轴夹角

    # 方位角
    phi = np.arctan2(y, x)  # [-pi, pi]

    # --- 6 参数等距畸变：θ -> θ_d（奇次多项式） ---
    t2 = theta * theta
    theta_d = (k0 * theta +
               k1 * theta * t2 +
               k2 * theta * t2 * t2 +
               k3 * theta * t2 * t2 * t2 +
               k4 * theta * t2 * t2 * t2 * t2 +
               k5 * theta * t2 * t2 * t2 * t2 * t2)

    # 等距投影：r_d = f * theta_d
    rdx = fx_src * theta_d
    rdy = fy_src * theta_d

    cos_phi = np.cos(phi)
    sin_phi = np.sin(phi)

    # 源图像坐标（极坐标 -> 像素），用各向异性缩放吸收 fx, fy
    map_u = (cx_src + rdx * cos_phi).astype(np.float32)
    map_v = (cy_src + rdy * sin_phi).astype(np.float32)

    # --- remap 采样 ---
    undistorted = cv2.remap(image, map_u, map_v,
                            interpolation=interpolation,
                            borderMode=border_mode)
    return undistorted, K_out

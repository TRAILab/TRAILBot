"""
2024.11.01 
tools function file
"""
import sys
import os
# Add the parent directory to sys.path
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)
sys.path.append("/home/mfyuan/local_folder/OpenGraph")

import numpy as np
import cv2
import matplotlib.pyplot as plt
import torch
from pathlib import Path
from typing import List, Dict, Optional, Any

from tokenize_anything import model_registry
sys.path.append("/home/mfyuan/local_folder/Tag2Text")
sys.path.append("/home/mfyuan/local_folder/OpenGraph")
sys.path.append("/home/mfyuan/local_folder")
sys.path.append("/home/mfyuan/local_folder/GroundingDINO")
sys.path.append("/home/mfyuan/local_folder/OpenNav_v2/describe-anything")


from agentic_rag.star.some_class.amg_class_hardware import MyAutomaticMaskGenerator
from agentic_rag.star.some_class.map_calss import DetectionList
import open3d as o3d
from collections import Counter
from sentence_transformers import SentenceTransformer, util
import spacy
from agentic_rag.star.some_class.map_calss import MapObjectList
import torch.nn.functional as F
import json
from llama import Llama, Dialog
import faiss
import re
# import openai
from openai import OpenAI

from tqdm import trange
from PIL import Image, ImageDraw, ImageFont
import matplotlib.colors as mcolors

import argparse
import ast
from transformers import SamModel, SamProcessor
from dam import DescribeAnythingModel, disable_torch_init
import cv2
from remembr.utils.util import get_caption

try:
    from Tag2Text.models import tag2text
    import torchvision.transforms as TS
except ImportError as e:
    print("Tag2text sub-package not found. Please check your PATH. ")
    raise e

try: 
    from groundingdino.util.inference import Model
except ImportError as e:
    print("Import Error: Please install Grounded Segment Anything following the instructions in README.")
    raise e

from langchain_community.chat_models import ChatOllama
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage
import torch.nn.functional as F

# spacy分词时的一些先验词汇表
CONFUSED_NOUNS = ["metal", "back", "part", "row", "triangular","patch"]
INTEREST_NOUNS = ["van", "house"]
INTEREST_ADJS = ["grassy","white"]


def load_models(cfg):
    '''
    加载各个模型，传入三个大模型+SBERT
    '''
    # 加载一下ram模型
    TAG2TEXT_CHECKPOINT_PATH = cfg.tag2text_path
    delete_tag_index = []
    for i in range(3012, 3429):
        delete_tag_index.append(i)
    # load model
    tagging_model = tag2text.tag2text_caption(pretrained=TAG2TEXT_CHECKPOINT_PATH,
                                            image_size=384,
                                            vit='swin_b',
                                            delete_tag_index=delete_tag_index)
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
    tap_model = model_registry[model_type](checkpoint=checkpoint)
    concept_weights = cfg.tap_merge_path
    tap_model.concept_projector.reset_weights(concept_weights)
    tap_model.text_decoder.reset_cache(max_batch_size=1000)

    # SBERT文本编码器
    #Pay attention to the model path!!!
    #sbert_model = SentenceTransformer(cfg.sbert_path)
    sbert_model = SentenceTransformer('sentence-transformers/all-MiniLM-L6-v2')
    # 创建分割器 
    if cfg.use_dam:
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        sam_model = SamModel.from_pretrained("facebook/sam-vit-huge").to(device)
        sam_processor = SamProcessor.from_pretrained("facebook/sam-vit-huge")
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # Initialize DAM model
        disable_torch_init()
        prompt_modes = {
            "focal_prompt": "full+focal_crop",
        }

        dam = DescribeAnythingModel(
            model_path=cfg.dam_model_path,
            conv_mode=cfg.dam_conv_mode,
            prompt_mode=prompt_modes.get(cfg.prompt_mode, cfg.prompt_mode),
        ).to(device)
    else:
        dam = None
        sam_model = None
        sam_processor = None

    mask_generator = MyAutomaticMaskGenerator(tagging_model=tagging_model, grounding_dino_model=grounding_dino_model, tap_model=tap_model, sbert_model=sbert_model, dam_model=dam, sam_model=sam_model, sam_processor=sam_processor)

    print("Congratulations! All models are loaded successfully and ready to use!")
    return mask_generator

def project(points, image, calib, configs):
    '''
    Project the points to the image
    '''
    points_homo = np.insert(points, 3, 1, axis=1).T #N*3->4*N
    #be careful since our lidar with y as front!!!!
    front_axis = configs.front_axis
    if front_axis == 'x':
        idx = 0
    elif front_axis == 'y':
        idx = 1
    # 
    pointCloud = np.delete(points, np.where(points_homo[idx, :] < 0), axis=0)
    points_homo = np.delete(points_homo, np.where(points_homo[idx, :] < 0), axis=1)  

    proj_lidar = calib['P_rect_20'].dot(calib['T_cam2_velo']).dot(points_homo)  
    # delet points that 
    cam = proj_lidar#np.delete(proj_lidar, np.where(proj_lidar[2, :] < 0), axis=1)  
    pointCloud = pointCloud#np.delete(pointCloud, np.where(proj_lidar[2, :] < 0), axis=0)
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

    return points, pixels


def grid_based_sampling(points, num_samples):
    """
    Perform grid-based sampling to evenly distribute keypoints over the area.

    Args:
        points (ndarray): The input point cloud of shape (N, 3), where N is the number of points.
        num_samples (int): The number of keypoints to sample.

    Returns:
        sampled_indices (ndarray): Indices of the sampled points.
    """
    # Compute bounding box of the point cloud
    min_bounds = np.min(points, axis=0)
    max_bounds = np.max(points, axis=0)
    
    # Calculate grid size based on the number of samples
    grid_size = (max_bounds - min_bounds) / np.cbrt(num_samples)
    
    # Assign each point to a grid cell
    grid_indices = np.floor((points - min_bounds) / grid_size).astype(int)
    
    # Create a dictionary to store one point per grid cell
    cell_dict = {}
    for i, grid_idx in enumerate(grid_indices):
        grid_key = tuple(grid_idx)
        if grid_key not in cell_dict:
            cell_dict[grid_key] = i
    
    # Collect sampled indices from grid cells
    sampled_indices = list(cell_dict.values())
    
    # If there are more points than needed, randomly downsample
    if len(sampled_indices) > num_samples:
        sampled_indices = np.random.choice(sampled_indices, num_samples, replace=False)
    
    return sampled_indices

def create_object_pcd(image, pc, pixels, mask, obj_color=None) -> o3d.geometry.PointCloud:
    '''
    得到rgb的点云 
    '''
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    mask_for_pc = mask[pixels[:, 0].astype(int), pixels[:, 1].astype(int)]
    points = pc[mask_for_pc]
    pixels = pixels[mask_for_pc]
    # print("points shape", points.shape)
    # print("pixels shape", pixels.shape)
    colors = image[pixels[:,0].astype(int), pixels[:,1].astype(int)]/255.0
    # 对点稍加扰动以避免共线性，看不出来是雷达线条
    points += np.random.normal(0, 4e-3, points.shape)

    # 创建一个Open3D PointCloud对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd

def create_SoM_road(image, pc, pixels, mask, dots_size_w=10, dots_size_h=10, font_path='/home/mfyuan/local_folder/OpenGraph/config/arial.ttf'):
    '''
    Create a grid of dots and labels on the road surface, constrained by a mask.
    '''
    # Apply the mask to filter the point cloud and corresponding pixels
    mask_for_pc = mask[pixels[:, 0].astype(int), pixels[:, 1].astype(int)]
    points = pc[mask_for_pc]
    pixels = pixels[mask_for_pc]
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # Prepare the image for drawing
    img = Image.fromarray(image)
    draw = ImageDraw.Draw(img)

    width, height = img.size
    grid_size_w = dots_size_w + 1
    grid_size_h = dots_size_h + 1
    cell_width = width / grid_size_w
    cell_height = height / grid_size_h
    font = ImageFont.truetype(font_path, width // 80)  # Adjust font size if needed
    
    count = 0
    for j in range(1, grid_size_h):
        for i in range(1, grid_size_w):
            x = int(i * cell_width)
            y = int(j * cell_height)

            # Check if the (x, y) coordinate is within the mask
            if 0 <= x < width and 0 <= y < height and mask[y, x]:  # Note: mask indices are (row, col)

                pixel_color = img.getpixel((x, y))
                # Choose a contrasting color based on the pixel brightness
                # if sum(pixel_color) >= 255 * 3 / 2:
                #     opposite_color = (0, 0, 0)
                # else:
                #     opposite_color = (255, 255, 255)
                opposite_color =(0, 0, 255)
                circle_radius = width // 260  # Adjust dot size if needed

                # Draw the ellipse (dot) at the grid point
                draw.ellipse([(x - circle_radius, y - circle_radius), 
                              (x + circle_radius, y + circle_radius)], 
                             fill=opposite_color)

                # Draw the label next to the dot
                text_x, text_y = x + 3, y
                count_w = count // dots_size_w
                count_h = count % dots_size_w
                label_str = f"({count_w + 1},{count_h + 1})"

                draw.text((text_x, text_y), label_str, fill=opposite_color, font=font)
            count += 1

    modified_img = np.array(img)
    
    return modified_img

def create_SoM_keypts(image, pc, mask, pixels, font_path='/home/mfyuan/local_folder/OpenGraph/config/arial.ttf'):
    '''
    Create a grid of dots and labels on the road surface, constrained by a mask.
    '''
    # Apply the mask to filter the point cloud and corresponding pixels
    import time
    t1 = time.time()
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    # Prepare the image for drawing
    img = Image.fromarray(image)
    draw = ImageDraw.Draw(img)
    width, height = img.size

    key_points, key_pixels = project_and_sample(pc, img, pixels, num_samples=200)
    mask_for_pc = mask[key_pixels[:, 0].astype(int), key_pixels[:, 1].astype(int)]

    key_points = key_points[mask_for_pc]
    key_pixels = key_pixels[mask_for_pc]

    
    opposite_color =(0, 0, 255)
    circle_radius = width // 260  # Adjust dot size if needed
    font = ImageFont.truetype(font_path, width // 100)  # Adjust font size if needed
    
    for i in range(len(key_pixels)):
        x = key_pixels[i][1]
        y = key_pixels[i][0]
        # Draw the ellipse (dot) at the grid point
    
        # Draw the ellipse (dot) at the grid point
        draw.ellipse([(x - circle_radius, y - circle_radius), 
                        (x + circle_radius, y + circle_radius)], 
                        fill=opposite_color)

        # Draw the label next to the dot
        text_x, text_y = x + 5, y-10
        # keep 1 decimal
        point_x =round(key_points[i][0], 1)
        point_y = round(key_points[i][1], 1)
        point_z = round(key_points[i][2], 1)

        label_str = f"{(point_x, point_y, point_z)}"
        draw.text((text_x, text_y), label_str, fill=opposite_color, font=font)
    #print("time", time.time()-t1)
    modified_img = np.array(img)
    
    return modified_img

def project_and_sample(points, image, pixels, num_samples=200):
    '''
    Project the points to the image and create a uniformly distributed grid of pixels.
    
    Args:
        points (ndarray): The input point cloud of shape (N, 3).
        image (ndarray): The image onto which the points will be projected.
        calib (dict): Calibration matrix containing 'P_rect_20' and 'T_cam2_velo'.
        configs (object): Configurations containing the lidar front axis.
        num_samples (int): Number of keypoints to sample.

    Returns:
        sampled_points (ndarray): Points corresponding to the sampled pixels.
        sampled_pixels (ndarray): Uniformly sampled pixel coordinates.
    '''
    IMG_W, IMG_H = image.size
    # Step 2: Create a uniform grid over the image
    grid_size = int(np.sqrt(num_samples))
    grid_x = np.linspace(0, IMG_W - 1, grid_size, dtype=int)
    grid_y = np.linspace(0, IMG_H - 1, grid_size, dtype=int)
    grid_coords = np.array(np.meshgrid(grid_y, grid_x)).T.reshape(-1, 2)

    # Step 3: Find the closest projected point for each grid coordinate
    sampled_pixels = []
    sampled_points = []
    for grid_pixel in grid_coords:
        # Compute distances to all projected pixels
        distances = np.linalg.norm(pixels - grid_pixel, axis=1)
        nearest_idx = np.argmin(distances)
        sampled_pixels.append(pixels[nearest_idx])
        sampled_points.append(points[nearest_idx])

    return np.array(sampled_points), np.array(sampled_pixels)


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
    


# this function is to get the bounding box of the object?
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
    # 对每个mask处理
    SoM = None
    mask_dic = {}
    valid_mask_indices = []
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
                    #SoM = create_SoM_road(image, pc, pixels, mask, dots_size_w=10, dots_size_h=10, font_path='/home/mfyuan/local_folder/OpenGraph/config/arial.ttf')
                    SoM = create_SoM_keypts(image, pc, mask, pixels, font_path='/home/mfyuan/local_folder/OpenGraph/config/arial.ttf')
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
        else:
            detection_lists.append(detected_object)
        # detection_lists.append(detected_object)
    #print(f'---------------------above is for image {idx}---------------------')
    return detection_lists, bg_list, SoM, mask_dic, valid_mask_indices


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
    print("Before final map filtering:", len(objects))
    objects_to_keep = []
    for obj in objects:
        if len(obj['pcd'].points) >= cfg.obj_min_points and obj['num_detections'] >= cfg.obj_min_detections:
            objects_to_keep.append(obj)
    objects = MapObjectList(objects_to_keep)
    print("After final map filtering: ", len(objects))
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
    # only use xyz
    pc = pc[:,:3]
    his_pcs = [arr[:,:3] for arr in his_pcs]
    # Combines the current point cloud with historical ones, where the most recent data is at index 0.[9,8,7,...,0]
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
        stride = len(his_poses)
        #print("his_ poses stride: ", stride)
        for i, pcd in enumerate(list_his_pcs):
            from_pose = poses[i]
            to_pose = poses[-1]
            pcd = transform_point_cloud(pcd, from_pose, to_pose)
            time_index = i - stride + 1
            timestamp = round(time_index * 0.1, 3)
            list_his_pcs[i] = timestamp_tensor(pcd, timestamp)
        past_point_clouds = torch.cat(list_his_pcs, dim=0)
        past_point_clouds = past_point_clouds.to('cuda')
        past_point_clouds_list = []
        past_point_clouds_list.append(past_point_clouds)
        out = mos_model.forward(past_point_clouds_list)
        for step in range(stride):
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
            # print(f"Moving confidence: {moving_confidence}")
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

            # moving_mask = moving_confidence > cfg.moving_thre  # These are moving points
            # static_mask = ~moving_mask                         # Invert to get static
            # all_pcs[step] = all_pcs[step][static_mask]
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
    if cfg.class_methods == "sbert" or cfg.class_methods == "llama" or cfg.class_methods == "chatgpt" or cfg.class_methods == "ollama" or cfg.class_methods == "longest":
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
        for i in trange(len(objects)):
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
            for i in trange(len(bg_objects)):
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
    elif cfg.class_methods == "ollama" or cfg.class_methods == "longest":
        # Prompt examples
        caption_example1 = "A cluster of tall, slender palm trees with long, arching fronds. The fronds are green and fan out from the top of the trunks, creating a dense canopy. The trunks are straight and cylindrical, with some fronds extending horizontally., A cluster of tall, slender palm trees with long, arching fronds. The fronds are green and fan out from the top of the trunks, creating a dense canopy. The trunks are straight and cylindrical, with some fronds extending horizontally"
        caption_example2 = "The sidewalk is composed of smooth, light gray concrete with a slightly rough texture. It features a circular manhole cover near the center-right. The sidewalk is bordered by a narrow strip of darker material on the left side., The sidewalk is composed of smooth, light gray concrete with a slightly rough texture. It features a circular manhole cover near the center-right. The sidewalk is bordered by a narrow strip of darker material on the left side., , The sidewalk is composed of smooth, light gray concrete with a slightly rough texture. It features a circular manhole cover near the center-right. The sidewalk is bordered by a narrow, darker gray strip on the left side"
        caption_example3 = "The building features a two-tone facade with the upper portion painted in a light blue color and the lower portion in a muted green. The upper section has three rectangular windows with wooden frames, evenly spaced. The lower section includes a large, solid green panel with a small, rectangular door and a small, square window to the right of the door., The building features a two-tone facade with the upper portion painted in a light blue color and the lower portion in a muted green. The upper section has three rectangular windows with wooden frames, evenly spaced. The lower section includes a large, solid green panel with a small, rectangular door and a small, square window to the right of the door., , The building features a two-tone facade with the upper portion painted in a light blue color and the lower portion in a muted green. The upper section has three rectangular windows with wooden frames, each divided into smaller panes. The lower section includes a large, solid green panel with a small, rectangular door and a small, square window to the right of the door."
        caption_example4 = "A rectangular, beige-colored metal junction box with a flat roof. The front panel features a small, rectangular, transparent window near the bottom right corner. Above the window, there is a small, square, metallic vent. The left side of the front panel has a vertical handle and a small, rectangular, metallic latch. The top of the junction box has a small, circular, metallic fixture., A rectangular, beige-colored metal junction box with a flat roof. The front panel features a small, rectangular, transparent window near the bottom right corner. Above the window, there is a small, square, metallic vent. The left side of the front panel has a vertical handle and a small, rectangular, metallic latch. The top of the junction box has a small, circular, metallic fixture., , A rectangular, beige-colored metal junction box with a flat roof. The front panel features a small, rectangular, transparent window near the top right corner. Below the window, there is a small, rectangular, blue label with white text. The left side of the front panel has a vertical handle and a small, rectangular, black latch. The right side of the front panel has a protruding, rectangular, gray component with a small, square, black latch."

        DEFAULT_PROMPT = """
        You are a classifier that can categorize a caption phrase into one of the following categories based on a caption phrase.
        List of categories: [car, bicycle, motorcycle, truck, person, bicyclist, motorcyclist, road,
        parking, sidewalk, building, fence, vegetation, trunk, terrain, pole, traffic-sign].
        You only need to generate one category name which must be included in this list. 
        The output format is 'Category name: [[your summarized category name itself]]'
        Emphasizing again: Do not provide words beyond the given list!!! Please test it yourself and regenerate it if it exceeds the list.
        """

        def classify_caption(caption_obj):
            messages = [
                SystemMessage(content=DEFAULT_PROMPT),
                HumanMessage(content=caption_example1),
                AIMessage(content="Category name: [vegetation]"),
                HumanMessage(content=caption_example2),
                AIMessage(content="Category name: [sidewalk]"),
                HumanMessage(content=caption_example3),
                AIMessage(content="Category name: [building]"),
                HumanMessage(content=caption_example4),
                AIMessage(content="Category name: [building]"),
                HumanMessage(content=caption_obj),
            ]
            result = generator.invoke(messages)
            input_text = result.content
            match = re.search(r'\[([^]]+)\]', input_text)
            return match.group(1) if match else ""

        for i in trange(len(objects)):
            caption_obj = objects[i]["caption"]
            extracted_content = classify_caption(caption_obj)

            if extracted_content not in class_colors_sk_disk:
                extracted_content_ft = sbert_model.encode(extracted_content, convert_to_tensor=True)
                extracted_content_ft = extracted_content_ft / extracted_content_ft.norm(dim=-1, keepdim=True)
                extracted_content_ft = extracted_content_ft.squeeze()
                similarities = F.cosine_similarity(class_name_fts, extracted_content_ft.unsqueeze(0), dim=-1)
                max_indices = torch.argmax(similarities)
                objects[i]['class_sk'] = class_names_sk[max_indices]
                objects[i]['inst_color'] = class_colors_sk[max_indices]
            else:
                objects[i]["class_sk"] = extracted_content
                objects[i]['inst_color'] = np.array(class_colors_sk_disk[extracted_content]) / 255.0

        if bg_objects is not None:
            for i in trange(len(bg_objects)):
                caption_obj = bg_objects[i]["caption"]
                extracted_content = classify_caption(caption_obj)

                if extracted_content not in class_colors_sk_disk:
                    extracted_content_ft = sbert_model.encode(extracted_content, convert_to_tensor=True)
                    extracted_content_ft = extracted_content_ft / extracted_content_ft.norm(dim=-1, keepdim=True)
                    extracted_content_ft = extracted_content_ft.squeeze()
                    similarities = F.cosine_similarity(class_name_fts, extracted_content_ft.unsqueeze(0), dim=-1)
                    max_indices = torch.argmax(similarities)
                    bg_objects[i]['class_sk'] = class_names_sk[max_indices]
                    bg_objects[i]['inst_color'] = class_colors_sk[max_indices]
                else:
                    bg_objects[i]["class_sk"] = extracted_content
                    bg_objects[i]['inst_color'] = np.array(class_colors_sk_disk[extracted_content]) / 255.0
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
        . You only need to generate one category name. 
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
        Make sure that the number of category names you output matches the number of captions; otherwise, regenerate them again untill their numbers are the same.
        """

        caption_objects = objects.get_stacked_str_torch("caption")
        batch_size = cfg.gpt_max_num
        num_batches = len(caption_objects) // batch_size + (len(caption_objects) % batch_size > 0)
    
        for batch_idx in range(num_batches):
            print("Batch num:", batch_idx,"/",num_batches)
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
            # check if the number of captions and the number of generated captions are the same
            
            print(f'current_caption_batch {current_caption_batch}, len {len(current_caption_batch)}')
            input_text_batch = input_text_batch.split('\n')
            extracted_contents_batch = [re.search(r'\[([^]]+)\]', result).group(1) for result in input_text_batch if re.search(r'\[([^]]+)\]', result)]
            print(f'extracted_contents_batch {extracted_contents_batch}, len {len(extracted_contents_batch)}')
            # debug for missing captions
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
        print(f"object id {i} capitons: {get_caption(caption_obj, 'lastone')} ******** class_name: {class_obj}\n")
    # if bg_objects is not None:
    #     for i in range(len(bg_objects)):
    #         caption_obj = bg_objects[i]["caption"]
    #         class_obj = bg_objects[i]["class_sk"]
    #         print(f"bgobject id {i} capitons: {caption_obj} ******** class_name: {class_obj}")


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
    stride = min(stride, len(observation))
    print(f"stride {stride}")
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
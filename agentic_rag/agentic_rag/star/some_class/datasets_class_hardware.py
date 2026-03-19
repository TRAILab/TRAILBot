"""
2024.11.01 
data class for loading semantic data format
"""

import abc
import glob
import json
import os
from pathlib import Path
from typing import Dict, List, Optional, Union

import cv2
import imageio
import numpy as np
import torch
import torch.nn.functional as F
import yaml
from natsort import natsorted




class Dataformatting(torch.utils.data.Dataset):
    def __init__(
        self,
        basedir: Union[Path, str],
        sequence: Union[Path, str],
        stride: Optional[int] = 1,
        **kwargs,
    ):
        # 序列的基本文件
        self.input_folder = os.path.join(basedir, sequence)
        # 标定文件，直接加载
        self.calib_path = os.path.join(self.input_folder, "calib.txt")
        self.calib = self.load_calib()
        # 按照stride的间隔读取
        self.stride = stride
        print("\n Congratulations! The dataset is loaded and ready for any use! \n")
        super().__init__()
    
    '''def load_poses(self):
        import numpy as np
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        poses = []
        with open(self.pose_path, "r") as f:
            lines = f.readlines()
            poses = np.array([list(map(float, line.strip().split())) for line in lines])
            poses = poses.reshape(-1,3,4)
            ones_column = np.zeros((poses.shape[0], 1, 4))
            ones_column[:, :, -1] = 1.0
            poses = np.append(poses, ones_column, axis=1)
            # cam_pose = poses
            # 变换到雷达坐标系
            T_baselink_to_lidar = np.array([
                            [ 0, -1,  0,  0],  # Flip the X-axis
                            [ 1,  0,  0,  0],  # Flip the Y-axis
                            [ 0,  0,  1,  0],  # Z-axis remains the same
                            [ 0,  0,  0,  1]   # Homogeneous coordinate
                        ])
            T_lidar_to_baselink = np.linalg.inv(T_baselink_to_lidar)
            # print(self.calib['T_cam2_velo'])
            poses = poses @ T_lidar_to_baselink
            
        return poses
    '''
   

    def load_calib(self):
        '''
        加载标定文件
        '''
        calib = {}
        with open(self.calib_path, "r") as calib_file:
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
    
    '''def load_velo_scan(self, velo_filename):

        scan = np.fromfile(velo_filename, dtype=np.float32)
        scan = scan.reshape((-1, 4))
        return scan'''
    
    def __getitem__(self, observation):
        '''
        retrieve the data from the dataset, including the image, point cloud, pose, and history
        '''
        color = observation[1][-1]
        pointCloud = observation[2][-1]
        pose = observation[3][-1]

        all_pc = observation[2]
        all_poses = observation[3]
        # Overlapping projection of historical frames

        his_pointCloud = []
        his_pose = []
        
        for i in range(self.stride-1):
            his_index = self.stride-i-1
            his_pointCloud.append(all_pc[his_index])
            his_pose.append(all_poses[his_index])
        return (
            color,
            pointCloud,
            pose,
            his_pointCloud,
            his_pose
        )





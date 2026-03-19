"""
2024.01.18 
加载semantic数据格式的一个类
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

import cv2
import numpy as np

import cv2
import numpy as np
from PIL import Image as PILImage

class VideoCaptionScheduler:
    def __init__(self, frame_interval=10, caption_every_n_sec=3, fps=10, color_paths=None):
        self.frame_interval = frame_interval
        self.caption_every_n_sec = caption_every_n_sec
        self.fps = fps
        self.segment_frame_count = caption_every_n_sec * fps
        self.caption_frames_needed = 6
        self.color_paths = color_paths

        self.frame_idx_buffer = []
        self.last_caption_segment_idx = -1

    def add(self, idx: int, new_object_detected: bool):
        # 当前段对应的真实帧号区间：例如 idx=3 表示 [21, 22, ..., 30]
        start_frame = idx * self.fps - self.fps + 1
        end_frame = idx * self.fps
        new_frames = list(range(start_frame, end_frame + 1))

        # 添加这些帧号到缓存中
        self.frame_idx_buffer.extend(new_frames)

        # 限制缓存长度
        if len(self.frame_idx_buffer) > self.segment_frame_count:
            self.frame_idx_buffer = self.frame_idx_buffer[-self.segment_frame_count:]

        # 判断是否触发 caption
        if (
            new_object_detected and
            idx % self.caption_every_n_sec == 0 and
            idx != self.last_caption_segment_idx
        ):
            self.last_caption_segment_idx = idx

            if len(self.frame_idx_buffer) >= self.segment_frame_count:
                sampled_idxs = self._sample_evenly(self.frame_idx_buffer, self.caption_frames_needed)
                image_paths = [self.color_paths[i] for i in sampled_idxs]
                print(f"Triggered captioning at segment {idx}, frames {sampled_idxs}, paths {image_paths}")

                # ✅ 读取图像并转换为 PIL.Image 格式（RGB + uint8）
                images = [
                    PILImage.fromarray(
                        cv2.cvtColor(cv2.imread(p), cv2.COLOR_BGR2RGB).astype('uint8').copy(), 'RGB'
                    )
                    for p in image_paths
                ]

                return True, images, image_paths

        return False, None, None

    def _sample_evenly(self, buffer, num_samples):
        # 我们从 buffer 中获取真实帧编号范围
        min_frame = buffer[0]
        max_frame = buffer[-1]

        # 均匀采样帧编号（从 max_frame - duration + delta 开始，到 max_frame 结束）
        sampled_frames = np.linspace(
            max_frame - self.segment_frame_count + self.fps // 2,
            max_frame,
            num_samples
        ).astype(int)

        # 保证结果是 buffer 内部的帧（有些系统可能跳帧）
        sampled_frames = [f for f in sampled_frames if f in buffer]
        return sampled_frames



class IsaacDataset(torch.utils.data.Dataset):
    def __init__(
        self,
        basedir: Union[Path, str],
        sequence: Union[Path, str],
        stride: Optional[int] = 1,
        start: Optional[int] = 0,
        end: Optional[int] = -1,
        **kwargs,
    ):
        # 序列的基本文件
        self.input_folder = os.path.join(basedir, sequence)
        # 标定文件，直接加载
        self.calib_path = os.path.join(self.input_folder, "calib.txt")
        # self.calib_path = os.path.join(basedir, f"{sequence}/calib.txt")
        self.calib = self.load_calib()
        # pose文件，直接把pose全读出来
        self.pose_path = os.path.join(self.input_folder, "poses.txt")
        # self.pose_path = os.path.join(basedir, f"{sequence}/poses.txt")
        self.poses = self.load_poses()
        # 图像文件，读取image_2
        self.timestamp_path = os.path.join(self.input_folder, "time.txt")
        self.timestamp = self.load_timestamp()
        #
        self.color_paths = natsorted(glob.glob(f"{self.input_folder}/image_2/*.png"))
   
        self.orig_color_paths = self.color_paths
        # 点云文件，读取velodyne
        self.pc_paths = natsorted(glob.glob(f"{self.input_folder}/velodyne/*.bin"))

        # self.pc_paths = natsorted(glob.glob(f"{basedir}/3d_comp/os1/{sequence}/3d_comp_os1_{sequence}_*.bin"))
        # 开始id和结束id，没有则默认全部
        self.start = start
        self.end = end
        if start < 0:
            raise ValueError("start must be positive. Got {0}.".format(stride))
        if not (end == -1 or end > start):
            raise ValueError(
                "end ({0}) must be -1 (use all images) or greater than start ({1})".format(end, start)
            )
        # 看看是不是读对了
        if len(self.color_paths) != len(self.pc_paths):
            raise ValueError("Number of color and depth images must be the same.")
        self.num_imgs = len(self.color_paths)
        if self.end == -1:
            self.end = self.num_imgs
        # 保持所有的poses和pc_paths以多帧重叠投影
        self.all_pc_paths = self.pc_paths
        self.all_poses = self.poses
        # 按照stride的间隔读取
        self.stride = stride
    
        self.color_paths = self.color_paths[self.start : self.end : stride]
        #print(f"original image number: {len(self.orig_color_paths)}, after stride: {len(self.color_paths)}")
        self.pc_paths = self.pc_paths[self.start : self.end : stride]
        self.poses = self.poses[self.start : self.end : stride]
        # 此时的文件长度
        self.num_imgs = len(self.color_paths)
        print("\n Congratulations! The SemanticKITTI dataset is loaded and ready for any use! \n")
        super().__init__()


    def load_timestamp(self):
        '''
        Load timestamps
        '''
        timestamps = []
        with open(self.timestamp_path, "r") as f:
            lines = f.readlines()
            for line in lines:
                timestamp = float(line.strip())
                timestamps.append(timestamp)
        return np.array(timestamps)
    
    def load_poses(self):
        '''
        Load poses
        '''
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
            #print(T_lidar_to_baselink)
            # print(self.calib['T_cam2_velo'])
            #poses = poses @ T_lidar_to_baselink
            
            poses = poses# @ self.calib['T_cam2_velo']
            # cam_pose_new = cam_pose[:, :3, 3]    
            # positions = poses[:, :3, 3]
            # # Plot the poses in 3D
            # fig1 = plt.figure()
            # ax = fig1.add_subplot(111, projection='3d')

            # # Plot trajectory (x, y, z)
            # ax.plot(positions[:300, 0], positions[:300, 1], positions[:300, 2], label='Trajectory', marker = '.', color = 'r')
            # # ax.plot(cam_pose_new[:300, 0], cam_pose_new[:300, 1], cam_pose_new[:300, 2], label='Trajectory', marker='.', color='r')

            # # Set labels
            # ax.set_xlabel('X')
            # ax.set_ylabel('Y')
            # ax.set_zlabel('Z')

            # # Optional: Set axis limits for better visualization
            # ax.set_xlim([-100, 100])
            # ax.set_ylim([-100, 100])
            # ax.set_zlim([-100, 100])

            # # Show plot
            # plt.title('3D Pose Trajectory')
            # plt.legend()
            # plt.show()
        return poses
    
   

    def load_calib(self):
        '''
        Load calibration data
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
    
    def load_velo_scan(self, velo_filename):
        '''
        加载雷达数据
        '''
        scan = np.fromfile(velo_filename, dtype=np.float32)
        # scan = scan.reshape((-1, 4))
        scan = scan.reshape((-1, 3))
        return scan
    
    def __len__(self):
        '''
        得到数据集中数据的数量
        '''
        return self.num_imgs
    
    def __getitem__(self, index):
        '''
        索引，按照索引值数据类的一个数据，包括图像、点云、位姿
        '''
        color_path = self.color_paths[index]
        pc_path = self.pc_paths[index]
        time = self.timestamp[index]
        # cv图像格式
        color = cv2.imread(color_path)
        # np格式
        pointCloud = self.load_velo_scan(pc_path)  # 读取lidar原始数据
        pose = self.poses[index]
        # 历史多帧的重叠投影
        his_pointCloud = []
        his_pose = []
        if index > 0:
            for i in range(self.stride-1):
                his_index = self.start+index*self.stride-i-1
                his_pointCloud.append(self.load_velo_scan(self.all_pc_paths[his_index]))
                his_pose.append(self.all_poses[his_index])
        return (
            color,
            pointCloud,
            pose,
            his_pointCloud,
            his_pose,
            self.orig_color_paths,
            time,
        )

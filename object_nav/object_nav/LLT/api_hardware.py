import numpy as np
import sys
import torch
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import open3d as o3d
from scipy.interpolate import splprep, splev, CubicSpline
from scipy.spatial.transform import Rotation as R
from scipy.ndimage import binary_closing, binary_fill_holes, binary_dilation, center_of_mass

import heapq
from scipy.spatial import KDTree
from scipy.spatial.distance import cdist
from scipy.stats import mode
import tiktoken
from scipy.ndimage import gaussian_filter
import os
import time


from scipy.spatial.distance import euclidean
from scipy.ndimage import distance_transform_edt
from fastdtw import fastdtw
from scipy.spatial import distance

from shapely.geometry import Polygon, Point


# import config
# import models
# import utils
# from PIL import Image
# from prompts.success_detection_prompt import SUCCESS_DETECTION_PROMPT
# from config import OK, PROGRESS, FAIL, ENDC
# from config import CAPTURE_IMAGES, ADD_BOUNDING_CUBES, ADD_TRAJECTORY_POINTS, EXECUTE_TRAJECTORY, OPEN_GRIPPER, CLOSE_GRIPPER, TASK_COMPLETED, RESET_ENVIRONMENT

class TrajectoryDrawer:
    def __init__(self, occupancy_map, start, goal):
        self.occupancy_map = occupancy_map
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.path = [self.start.tolist()]  # 轨迹起点
        self.fig, self.ax = plt.subplots()
        self.ax.imshow(self.occupancy_map.T, cmap='gray', origin='lower')
        self.ax.plot(start[0], start[1], 'ro', label='Start')
        self.ax.plot(goal[0], goal[1], 'bo', label='Goal')
        self.ax.legend()
        self.cid = self.fig.canvas.mpl_connect('button_press_event', self.onclick)

    def onclick(self, event):
        """鼠标点击时记录轨迹点"""
        if event.xdata is not None and event.ydata is not None:
            self.path.append([event.xdata, event.ydata])
            self.ax.plot(event.xdata, event.ydata, 'go', markersize=3)  # 画出手绘点
            self.fig.canvas.draw()

    def finish_and_smooth(self):
        """完成轨迹绘制，进行B-spline平滑"""
        self.path.append(self.goal.tolist())  # 确保终点加入
        self.path = np.array(self.path)

        # B-spline 插值
        tck, u = splprep(self.path.T, s=3)  # s 控制平滑度
        u_new = np.linspace(0, 1, 100)  # 生成平滑曲线
        #smooth_path = np.array(splev(u_new, tck)).T  # 计算平滑轨迹
        smoothed_x, smoothed_y = splev(u_new, tck)

        # # 绘制平滑路径
        # self.ax.plot(self.path[:, 0], self.path[:, 1], 'go', markersize=3, label="Raw Points")
        # self.ax.plot(smooth_path[:, 0], smooth_path[:, 1], 'r-', label="Smoothed Path")
        # self.ax.legend()
        # self.fig.canvas.draw()
        # plt.show()

        return list(zip(smoothed_x, smoothed_y))

    
class API:

    def __init__(self, cfg, objects, bg_list, pose, idx, logger, sematic_info, drivable_indices, robot_state, semantic_map, resolution, map_origin, occupancy_map, terrain_map):#args, main_connection, logger, langsam_model, xmem_model, device

        # self.args = args
        # self.main_connection = main_connection
        self.logger = logger
        # self.langsam_model = langsam_model
        # self.xmem_model = xmem_model
        # self.device = device
        # self.segmentation_texts = []
        # self.segmentation_count = 0
        # self.trajectory_length = 0
        # self.attempted_task = False
        self.completed_task = False
        self.failed_task = False
        # self.head_camera_position = None
        # self.head_camera_orientation_q = None
        self.trajectory = None
        # self.wrist_camera_position = None
        # self.wrist_camera_orientation_q = None
        self.command = None
        self.cfg = cfg
        self.objects = objects
        self.bg_list = bg_list
        self.idx = idx
        self.drivable_area = bg_list
        self.pose = pose
        self.sematic_info = sematic_info
        self.drivable_indices = drivable_indices
        self.robot_state = robot_state
        self.semantic_map = semantic_map
        self.rows, self.cols = semantic_map.shape
        self.resolution = resolution
        self.map_origin = map_origin
        self.occupancy_map = occupancy_map
        self.terrain_map = terrain_map
        self.robot_radius = 1
        self.global_path_final = None
    def update_command(self, command):
        self.command = command

    def downsample_preserving_small_objects(self, original_res=0.1, new_res=0.5):
        factor = int(new_res / original_res)  # Downsampling factor

        new_shape = (self.semantic_map.shape[0] // factor, self.semantic_map.shape[1] // factor)

        downsampled_map = np.full(new_shape, -1, dtype=int)  # Initialize with -1

        for i in range(new_shape[0]):
            for j in range(new_shape[1]):
                block = self.semantic_map[i * factor:(i + 1) * factor, j * factor:(j + 1) * factor]

                # 统计所有非 -1 类别的数量
                unique, counts = np.unique(block[block != -1], return_counts=True)
                
                if len(unique) > 0:
                    # 选择出现次数最多的类别（原模式）
                    most_common = unique[np.argmax(counts)]
                    
                    # 如果类别中存在小物体（罕见类别），给予更高权重
                    if len(unique) > 1 and np.min(counts) <= (factor * factor * 0.2):  # 20% 的阈值
                        rare_class = unique[np.argmin(counts)]
                        downsampled_map[i, j] = rare_class  # 优先保留小物体类别
                    else:
                        downsampled_map[i, j] = most_common  # 采用原来的模式

        return downsampled_map

    def downsample_semantic_map(self, original_res=0.1, new_res=1):
        factor = int(new_res / original_res)  # Downsampling factor (5 in this case)
        new_shape = (min(320, self.semantic_map.shape[0]) // factor, min(320, self.semantic_map.shape[1]) // factor)

        downsampled_map = np.zeros(new_shape, dtype=int)  # Initialize new map

        for i in range(new_shape[0]):
            for j in range(new_shape[1]):
                block = self.semantic_map[i * factor:(i + 1) * factor, j * factor:(j + 1) * factor]
                downsampled_map[i, j] = mode(block, axis=None).mode[0]  # Assign most frequent label

        return downsampled_map

    def detect_object(self, segmentation_text = None):
        taget = segmentation_text
        for i in range(len(self.objects)):
            if self.objects[i]['image_idx'][-1] == self.idx:
                bbox = self.objects[i]['bbox']  # OrientedBoundingBox object
                img_bbx = self.objects[i]['img_bbox']
                rounded_center = np.round(bbox.center, 3)  # Center is a 3D point (x, y, z)
                rounded_extent = np.round(bbox.extent, 3)
                caption_obj = self.objects[i]["caption"]
                if ", " in caption_obj:
                    last_caption = caption_obj[caption_obj.rfind(', ')+2:]
                else:
                    last_caption = caption_obj
                    # # if too many observations, the string will be too long, then need to delete some observations in the front
                    # comma_count = caption_obj.count(', ')
                    # if comma_count > self.cfg.VLT_caption_num:
                    #     num_last_comma_index = 0
                    #     for j in range(self.cfg.VLT_caption_num):
                    #         num_last_comma_index = caption_obj.rfind(', ', 0, num_last_comma_index - 1)
                    #     # keep the last one
                    #     caption_obj = caption_obj[num_last_comma_index+1:]
                print('Caption'+str(i) +':'+last_caption+'; 2D bbox of the object in the image [xyxy]:'+str(img_bbx[-4:])+';', 'Center of 3D bbox:', rounded_center, ';', 'extent of 3D bbox:', rounded_extent)       
    

    def remove_empty_rows_and_columns(self, matrix):
        matrix = np.array(matrix)
        
        # 找到非全是 -1 的行和列
        valid_rows = np.any(matrix != -1, axis=1)
        valid_cols = np.any(matrix != -1, axis=0)
        
        # 获取索引映射
        row_map = np.where(valid_rows)[0]
        col_map = np.where(valid_cols)[0]
        
        # 生成精简后的矩阵
        reduced_matrix = matrix[np.ix_(valid_rows, valid_cols)]
        
        return reduced_matrix, row_map, col_map

    def get_original_position(self, reduced_i, reduced_j, row_map, col_map):
        """
        获取精简后矩阵索引 (reduced_i, reduced_j) 在原始矩阵中的索引 (original_i, original_j)
        """
        return row_map[reduced_i], col_map[reduced_j]
    
    def get_reduced_position(self, original_i, original_j, row_map, col_map):
        """
        获取原始矩阵索引 (original_i, original_j) 在精简后矩阵中的索引 (reduced_i, reduced_j)
        """
        # 查找原始索引在 row_map 和 col_map 中的位置
        reduced_i = np.where(row_map == original_i)[0]
        reduced_j = np.where(col_map == original_j)[0]

        # 如果原始索引在精简后矩阵中不存在，返回 None
        if reduced_i.size == 0 or reduced_j.size == 0:
            return None

        return reduced_i[0], reduced_j[0]

    def generate_map_prompt(self, path= []):
        """
        Generates a structured text representation of a semantic grid map.
        The origin (0,0) is assumed to be at the bottom-left corner.
        """
        original_res=0.1
        new_res=1.5

        # grid = self.downsample_semantic_map(original_res, new_res)
        grid = np.array(self.downsample_preserving_small_objects(original_res, new_res))
        x, y, z = np.round(self.robot_state["robot_state"][0], 2)
        heading_angle = np.round(self.robot_state["robot_state"][1], 2)
        # rad to degree
        heading_angle_deg = np.rad2deg(heading_angle)
        start = np.floor(([x, z] - self.map_origin) / new_res).astype(int)
        
        kdtree, valid_indices = self.build_kdtree_for_semantic_map(grid, self.drivable_indices)
        



        start = self.find_closest_point(kdtree, valid_indices, start)
        row_org, col_org = start[0], start[1]
        grid[row_org, col_org] = 99  # Set the start position to 0


        grid = grid.T
        grid = np.flip(grid, axis=0)  # Flip the map vertically
        # Ensure the grid is a list of lists (in case it's a NumPy array)
        # if isinstance(grid, np.ndarray):
        #     grid = grid.tolist()
        

        
        #print(f'row_org: {row_org}, col_org: {col_org}')
        

        reduced_grid, row_map, col_map = self.remove_empty_rows_and_columns(grid)
        # Get the dimensions of the reduced grid
        rows, cols = reduced_grid.shape
        #print(f'reduced_grid.shape: {reduced_grid.shape}')
        # rows = len(reduced_grid)
        # cols = len(reduced_grid[1]) if rows > 0 else 0

        # Generate the column index labels
        col_labels = "   " + " ".join(f"{i:2}" for i in range(cols))
        
        
        # Generate the map with row indices
        map_str = f"Semantic Grid Map:\n{col_labels}\n"
        # map_str += "2D_semantic matrix = \n"#f"Semantic Grid Map (Resolution: {new_res}m per cell):\n{col_labels}\n"

  
        # find where is 99 value located in the grid (numpt array)
        find_99 = np.where(np.array(reduced_grid) == 99)
        row, col = find_99[0], find_99[1]

        #original_position = self.get_original_position(2, 3, row_map, col_map)

        for r in range(rows):#rows - 
            row_index = f"{rows - r -1:2} "  # Row index (flipped so that 0 is at the bottom)
            #row_data = " ".join(f"{val:2}" for val in grid[r])
            if path != []:
                row_data = " ".join(
                "HH" if (r, c) in [(row, col) for row, col, _ in path] else f"{val:2}" 
                for c, val in enumerate(reduced_grid[r]))
            else:
                row_data = " ".join(
                "HH" if (r, c) == (row, col) else f"{val:2}" for c, val in enumerate(reduced_grid[r])
            )
            map_str += f"{row_index}{row_data}\n"
            # map_str += f"{row_data}\n"

            
                        # Define the legend for different values in the map

        info1 = f"\nEach cell represents a {new_res}m x {new_res}m area. your current position is located at {(rows-row[0]-1, col[0])} heading_angle: {int(heading_angle_deg[0])} degree\n\
        If the heading_angle is 0 degrees, it represents facing east. If heading_angle is 90 degrees, it represents facing north. similarly 180 degrees means facing west. 270 degrees represents facing south.\n\
        -1  -> Unknown Space\n"
        map_str += info1
        # save as txt file
        path_txt = '/home/mfyuan/local_folder/results/05/matrix_txt/semantic_map'+str(self.idx)+'.txt'
        with open(path_txt, 'w') as f:
            f.write(map_str)

        return map_str  + info1
    
    
    
    def count_tokens(self, text, model="gpt-4o"):
        """
        Counts the number of tokens in a given text input.
        
        Parameters:
        - text (str): The text whose token count is to be calculated.
        - model (str): The OpenAI model being used (default: "gpt-4o").
        
        Returns:
        - int: The token count.
        """
        encoding = tiktoken.encoding_for_model(model)
        tokens = encoding.encode(text)
        return len(tokens)

    def generate_bspline_trajectory_3d(self, trajectory_points: list, num_points: int) -> list:
        """
        Smooths a given trajectory using cubic spline interpolation.
        
        Parameters:
        - trajectory_points: list or np.array of shape (n, 4), where each point is (x, y, z, heading)
        - num_points: int, number of interpolated points for smoothing
        
        Returns:
        - smoothed_points: list of tuples [(x, y, z, heading)], the smoothed trajectory points
        """
        # Convert trajectory points to a NumPy array
        trajectory_points = np.array(trajectory_points)
        
        # Extract x, y, z, and heading
        x = trajectory_points[:, 0]
        # take the y value of the first point in trajectory_points as the y value for the rest of the points
        # Use the y value of the first point for all points
        y_fixed_value = trajectory_points[0, 1]
        y = np.full_like(x, y_fixed_value)
        #y = trajectory_points[:, 1]
        z = trajectory_points[:, 2]
        heading = trajectory_points[:, 3]
        
        # Define parameter t for interpolation (normalized)
        t = np.linspace(0, 1, len(trajectory_points))
        
        # Fine-grained parameter for smoothed trajectory
        t_smooth = np.linspace(0, 1, num_points)
        
        # Smooth each dimension using cubic spline
        x_smooth = CubicSpline(t, x)(t_smooth)
        y_smooth = CubicSpline(t, y)(t_smooth)
        z_smooth = CubicSpline(t, z)(t_smooth)
        heading_smooth = CubicSpline(t, heading)(t_smooth)
        
        # Combine smoothed dimensions and convert to list of tuples
        smoothed_points = list(zip(x_smooth, y_smooth, z_smooth, heading_smooth))
        
        return smoothed_points


    # def heuristic(self, point1, point2):
    #     """Heuristic function: Euclidean distance."""
    #     return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)
    
    def heuristic(self, a, b):
        """计算欧几里得距离启发函数"""
        return np.linalg.norm(np.array(a) - np.array(b))

    # def is_valid(self, point):
    #     """Check if a point is valid."""
    #     x, y = point
    #     return 0 <= x < self.rows and 0 <= y < self.cols and self.cost_map[x, y] < float('inf')
    # def is_valid(self, point):
    #     """Check if a point is valid considering robot's size."""
    #     x, y = point
    #     # Check if within bounds and no obstacle at the point itself
    #     if not (0 <= x < self.rows and 0 <= y < self.cols):
    #         return False
    #     if self.cost_map[x, y] == 10 or self.cost_map[x, y] == 20:  # obstacle check
    #         return False

    #     # # Check surrounding area based on robot's radius
    #     # for dx in range(-self.robot_radius, self.robot_radius+1):
    #     #     for dy in range(-self.robot_radius, self.robot_radius+1):
    #     #         nx, ny = x + dx, y + dy
    #     #         if 0 <= nx < self.rows and 0 <= ny < self.cols:
    #     #             if self.cost_map[nx, ny] == 10 or self.cost_map[nx, ny] == 20:  # obstacle in vicinity
    #     #                 return False
    #     return True
    def is_valid(self, point):
        """Check if a point is valid considering robot's size and a safety margin."""
        x, y = point
        
        # Check if within bounds
        if not (0 <= x < self.rows and 0 <= y < self.cols):
            return False
        
        # Add a safety margin to the obstacle check
        safety_margin = int(np.ceil(self.robot_radius))  # Robot radius or buffer zone in grid cells

        # Check if the point itself or surrounding area is an obstacle
        if self.cost_map[x, y] in [10, 20]:  # Check for obstacle values (adjust as necessary)
            return False

        # Check surrounding area based on robot's size (radius + margin)
        for dx in range(-safety_margin, safety_margin+ 1):
            for dy in range(-safety_margin, safety_margin + 1):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.rows and 0 <= ny < self.cols:
                    if self.cost_map[nx, ny] in [10, 20]:  # Obstacle in vicinity
                        return False
                    
        return True
    def get_neighbors(self, point):
        """Return valid neighbors of a point."""
        x, y = point
        neighbors = [
            (x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1),  # cardinal directions
            (x - 1, y - 1), (x - 1, y + 1), (x + 1, y - 1), (x + 1, y + 1)  # diagonals
        ]
        return [n for n in neighbors if self.is_valid(n)]

    def reconstruct_path(self, came_from, current):
        """Reconstruct the path from start to goal."""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        global_path = np.array(path) * self.resolution + self.map_origin
        updated_path = np.zeros((global_path.shape[0], 3))
        updated_path[:, 0] = global_path[:, 0]
        updated_path[:, 1] = self.robot_state["robot_state"][0][1]
        updated_path[:, 2] = global_path[:, 1]
        return path, updated_path
    
    def build_kdtree_for_semantic_map(self):#semantic_map, valid_classes
        """
        Build a KDTree for the valid points in the semantic map.

        Args:
            semantic_map (np.ndarray): 2D semantic map where each value represents a class.
            valid_classes (list): List of classes to include in the KDTree.

        Returns:
            KDTree: KDTree built from the valid points.
            np.ndarray: Array of valid point indices (row, col) used to build the KDTree.
        """
        # Find indices of valid points
        # check if the valid_classes is a dictionary or list
        # if isinstance(valid_classes, dict):
        #     valid_class_keys = list(valid_classes.keys())
        # elif isinstance(valid_classes, list):
        #     valid_class_keys = valid_classes
        # else:
        #     print("Invalid valid_classes type. Please provide a list or dictionary.")
        # print(f'valid_class_keys: {valid_class_keys}')  
        # check free space indices from occupancy map
        valid_indices = np.argwhere(self.occupancy_map == 0)
        #valid_indices = #np.argwhere(np.isin(semantic_map, valid_class_keys))
        # Build KDTree using the valid indices
        kdtree = KDTree(valid_indices)
        return kdtree, valid_indices

    def find_closest_point(self, current_location):#kdtree, valid_indices, current_location):
        """
        Find the closest valid point to the robot's current location using the KDTree.

        Args:
            kdtree (KDTree): KDTree built from the valid points.
            valid_indices (np.ndarray): Array of valid point indices used to build the KDTree.
            current_location (tuple): Current location of the robot (row, col).

        Returns:
            tuple: Closest point indices (row, col) in the semantic map.
        """
        valid_indices = np.argwhere(self.occupancy_map == 0)
        #valid_indices = #np.argwhere(np.isin(semantic_map, valid_class_keys))
        # Build KDTree using the valid indices
        kdtree = KDTree(valid_indices)
        # Query the KDTree for the closest point
        distance, index = kdtree.query(current_location)
        # Get the closest point's indices
        closest_point = tuple(valid_indices[index])
        return closest_point


    def is_valid_area_for_road(self, target, radius):
        x, y = target

        for dx in range(-radius, radius):
            for dy in range(-radius, radius):
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.rows and 0 <= ny < self.cols:
                    if self.cost_map[nx, ny] in [10, 20]:  
                        return False
        return True

    def move_edge_point_to_road(self, edge_point, radius):
        x1, y1 = edge_point  # 当前的边缘点位置

        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                x2, y2 = x1 + dx, y1 + dy
                if 0 <= x2 < self.rows and 0 <= y2 < self.cols:
                    if self.cost_map[x2, y2] not in [10, 20]:  # 检查目标点是否是障碍物

                        if self.is_valid_area_for_road((x2, y2), radius):
                            # 返回符合条件的目标位置
                            print(f'{x2, y2}')
                            return (x2, y2)
        
        # 如果没有找到合适的位置，返回 None 或者其他提示
        return None

    def update_original_cost_map(self, cost_map, path, radius=0.6):
        """
        Updates the original cost map by assigning a lower cost to regions corresponding to the path indices 
        from the downsampled grid.

        Parameters:
        - cost_map (np.ndarray): The high-resolution original cost map.
        - path (np.ndarray): List of global (x, z) waypoints.
        - radius (float): Radius in meters within which to lower costs.

        Returns:
        - np.ndarray: Updated original cost map.
        """
        if not isinstance(path, np.ndarray):
            path = np.array(path)  # Ensure NumPy array
        
        if path.ndim != 2 or path.shape[1] < 3:
            raise ValueError("Error: 'path' must be an (N, 3) array representing (x, y, z) coordinates.")

        # Convert global path coordinates to map indices (only use x and z)
        path_indices = np.floor((path[:, [0, 1]] - self.map_origin) / self.resolution).astype(int)

        # Clip indices to ensure they are within the map bounds
        path_indices = np.clip(path_indices, [0, 0], np.array(cost_map.shape) - 1)

        # Build KD-Tree for fast nearest neighbor search
        traversable_points = np.column_stack(np.where(cost_map == 3))  # Get all traversable (cost=1) points
        if traversable_points.shape[0] > 0:
            # Convert grid indices to global coordinates
            global_traversable_points = traversable_points * self.resolution + self.map_origin

            # Build KDTree
            tree = KDTree(global_traversable_points)  
            
            # Find traversable points within the specified radius of the path
            neighbors = tree.query_ball_point(path[:, [0, 1]], r=radius)  

            for i, indices in enumerate(neighbors):
                for idx in indices:
                    map_x, map_y = traversable_points[idx]  # Convert back to map indices
                    cost_map[map_x, map_y] = 0  # Reduce cost for nearby points

        # Set exact path points to cost = 0
        for px, py in path_indices:
            cost_map[px, py] = 0

        return cost_map, path_indices


    def a_star(self, cost_map, start, goal):
        """
        高效 A* 搜索算法
        """
        rows, cols = cost_map.shape
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]  # 8方向

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                
                return path[::-1]  # 返回路径

            for dr, dc in directions:
                neighbor = (current[0] + dr, current[1] + dc)

                if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and cost_map[neighbor] != 10:
                    tentative_g_score = g_score[current] + self.heuristic(current, neighbor) + cost_map[neighbor]

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # 没找到路径

    # def inflate_cost_map(self, cost_map, robot_radius, inflation_factor=20):
    #     """
    #     使用高斯滤波对障碍物进行膨胀 (inflation)，提高计算效率
    #     """
    #     inflation_radius = int(np.ceil(robot_radius * inflation_factor))  # 计算膨胀范围
    #     obstacle_mask = (cost_map == 10).astype(float)  # 创建障碍物掩码

    #     # 使用高斯滤波扩展障碍物
    #     inflated_map = gaussian_filter(obstacle_mask, sigma=inflation_radius, mode='nearest')


    #     # 归一化到 (0, 10)，靠近障碍物的区域更危险
    #     inflated_map = inflated_map / np.max(inflated_map) * 10

    #     inflated_map[cost_map == 10] = 10  # 确保原始障碍物保持10
    #     inflated_map[self.occupancy_map == -1] = 8  # 确保原始障碍物保持20

    #     return inflated_map
    
    def inflate_cost_map_old(self, cost_map, robot_radius, inflation_factor=30, unknown_cost=8, terrain_cost=5):
        """
        Expand obstacles (10) and unknown areas (-1)
        Use Gaussian filtering for smooth expansion
        unknown_cost sets the default cost for unknown areas (lower than obstacles)
        """
        inflation_radius = int(np.ceil(robot_radius * inflation_factor))  # 计算膨胀范围
        processed_map = cost_map.copy()

        # # 将未知区域 (-1) 设为 high cost（但比障碍物低）
        # processed_map[processed_map == -1] = unknown_cost

        # 生成障碍物 & 未知区域掩码
        obstacle_mask = (processed_map == 10).astype(float)
        unknown_mask = (processed_map == unknown_cost).astype(float)
        terrain_mask = (processed_map == terrain_cost).astype(float)

        # 对障碍物进行膨胀
        inflated_obstacles = gaussian_filter(obstacle_mask, sigma=inflation_radius, mode='nearest')
        inflated_unknowns = gaussian_filter(unknown_mask, sigma=inflation_radius, mode='nearest')
        inflacted_terrain = gaussian_filter(terrain_mask, sigma=inflation_radius, mode='nearest')

        # 归一化到 (0, 10)
        inflated_obstacles = inflated_obstacles / np.max(inflated_obstacles) * 10
        inflated_unknowns = inflated_unknowns / np.max(inflated_unknowns) * unknown_cost
        inflacted_terrain = inflacted_terrain / np.max(inflated_obstacles) * terrain_cost

        # 取最大值，保证障碍物和未知区域都能膨胀
        inflated_map = np.maximum(processed_map, inflated_obstacles)
        inflated_map = np.maximum(inflated_map, inflated_unknowns)
        inflated_map = np.maximum(inflated_map, inflacted_terrain)

        return inflated_map
    

    def inflate_cost_map(self, cost_map, robot_radius, inflation_factor=12, unknown_cost=8, terrain_cost=5):
        """
        Inflate cost map around obstacles, unknowns, and terrain using distance transform.

        Args:
            cost_map (np.ndarray): 2D array where:
                                - 0 = free
                                - 10 = obstacle
                                - unknown_cost = unknown (e.g., 8)
                                - terrain_cost = terrain (e.g., 5)
            robot_radius (float): Robot's physical radius in meters
            resolution (float): Map resolution in meters per cell
            inflation_factor (float): Multiplier to increase inflation zone (e.g., 1.0 = robot radius)
            unknown_cost (float): Cost assigned to unknown areas (default 8)
            terrain_cost (float): Cost assigned to terrain areas (default 5)

        Returns:
            np.ndarray: Inflated cost map
        """

        inflation_radius_cells = int(np.ceil(robot_radius * inflation_factor / self.resolution))
        inflated_map = cost_map.copy()

        # Masks
        obstacle_mask = (cost_map >= 9.5)
        unknown_mask = np.isclose(cost_map, unknown_cost, atol=0.5)
        terrain_mask = np.isclose(cost_map, terrain_cost, atol=0.5)

        # Invert masks: distance transform measures distance to closest FALSE
        obstacle_dist = distance_transform_edt(~obstacle_mask)
        unknown_dist = distance_transform_edt(~unknown_mask)
        terrain_dist = distance_transform_edt(~terrain_mask)

        # Linearly decaying inflation (closer = higher cost)
        inflated_obstacles = np.clip((inflation_radius_cells - obstacle_dist), 0, inflation_radius_cells) / inflation_radius_cells * 10
        inflated_unknowns  = np.clip((inflation_radius_cells - unknown_dist), 0, inflation_radius_cells) / inflation_radius_cells * unknown_cost
        inflated_terrain   = np.clip((inflation_radius_cells - terrain_dist), 0, inflation_radius_cells) / inflation_radius_cells * terrain_cost

        # Merge layers
        inflated_map = np.maximum.reduce([inflated_map, inflated_obstacles, inflated_unknowns, inflated_terrain])
        return inflated_map


    def inflate_cost_map_use(self, cost_map, robot_radius, inflation_factor=12, unknown_cost=8, terrain_cost=5):
        """
        Inflate high-cost regions (obstacles, unknown, terrain) using Gaussian filtering.
        This smooths cost transitions and keeps robot away from edges and danger zones.
        """
        inflation_radius = int(np.ceil(robot_radius * inflation_factor))
        processed_map = cost_map.copy()

        # Create masks using tolerant matching
        obstacle_mask = (processed_map >= 9.5).astype(float)
        unknown_mask = np.isclose(processed_map, unknown_cost, atol=0.5).astype(float)
        terrain_mask = np.isclose(processed_map, terrain_cost, atol=0.5).astype(float)

        # Gaussian filtering (soft inflation)
        inflated_obstacles = gaussian_filter(obstacle_mask, sigma=inflation_radius, mode='nearest')
        inflated_unknowns = gaussian_filter(unknown_mask, sigma=inflation_radius, mode='nearest')
        inflated_terrain = gaussian_filter(terrain_mask, sigma=inflation_radius, mode='nearest')

        # Normalize each layer safely
        if np.max(inflated_obstacles) > 0:
            inflated_obstacles = inflated_obstacles / np.max(inflated_obstacles) * 10
        if np.max(inflated_unknowns) > 0:
            inflated_unknowns = inflated_unknowns / np.max(inflated_unknowns) * unknown_cost
        if np.max(inflated_terrain) > 0:
            inflated_terrain = inflated_terrain / np.max(inflated_terrain) * terrain_cost

        # Merge inflated layers with original map
        inflated_map = np.maximum(processed_map, inflated_obstacles)
        inflated_map = np.maximum(inflated_map, inflated_unknowns)
        inflated_map = np.maximum(inflated_map, inflated_terrain)

        return inflated_map


    def visualize_path(self, cost_map, path, start, goal):
        """
        可视化代价地图和路径，采用彩色热度图
        """
        fig = plt.figure(figsize=(10, 8))
        plt.imshow(cost_map.T, cmap="jet", origin="lower")  # 使用热度图
        # plt.colorbar(label="Cost")
        cbar = plt.colorbar(label="Cost", orientation='horizontal', pad=0.08)
        cbar.ax.tick_params(labelsize=10)
        labels = ["OpenNav", "A*", "User Path", "VLT-Code"]
        colors = ['r-', 'g-', 'orange', 'lime', 'y-']


        if len(path):
        #     path_x, path_y = zip(*path)
        #     plt.plot(path_y, path_x, 'w-', linewidth=2, label="Planned Path")  # 用白色表示路径
            for index, point in enumerate(path):
                if index == 2:
                    continue
                path_x, path_y = zip(*point)
                plt.plot(path_x, path_y, colors[index], linewidth=3, label=f"{labels[index]}")  # 用白色表示路径
            plt.plot(start[0], start[1], 'ro', markersize=10, label="Start")  # 起点
            plt.plot(goal[0], goal[1], 'go', markersize=10, label="Goal")  # 终点
        
        plt.legend()
        plt.title("Path")
        # plt.show()
        # plt.savefig('/home/mfyuan/local_folder/results/05/heatmap/heatmap'+str(self.idx)+'.png')
        return fig
        # save the figure
        # plt.savefig('/home/mfyuan/local_folder/results/05/heatmap/heatmap'+str(self.idx)+'.png')

    def smooth_path_catmull_rom(self, path, num_points=100):
        """
        使用 Catmull-Rom 样条对路径进行平滑
        """
        path = np.array(path)
        t = np.arange(len(path))  # 生成参数
        cs_x = CubicSpline(t, path[:, 0])  # x 方向插值
        cs_y = CubicSpline(t, path[:, 1])  # y 方向插值
        t_new = np.linspace(0, len(path) - 1, num_points)  # 生成新的采样点

        smoothed_x = cs_x(t_new)
        smoothed_y = cs_y(t_new)

        return list(zip(smoothed_x, smoothed_y))
    

    def euclidean_distance(self, traj1, traj2):
        """
        计算两条轨迹的平均欧几里得距离（需要轨迹点数相同）
        """
        traj1, traj2 = np.array(traj1), np.array(traj2)
        min_len = min(len(traj1), len(traj2))
        traj1, traj2 = traj1[:min_len], traj2[:min_len]  # 对齐长度
        distances = np.linalg.norm(traj1 - traj2, axis=1)  # 逐点计算欧氏距离
        return np.mean(distances)

    def frechet_distance(self, P, Q):
        """
        计算 Frechet Distance，用于衡量轨迹形状相似度。
        """
        dp = np.full((len(P), len(Q)), -1.0)

        def c(i, j):
            if dp[i, j] > -1:
                return dp[i, j]
            elif i == 0 and j == 0:
                dp[i, j] = distance.euclidean(P[0], Q[0])
            elif i > 0 and j == 0:
                dp[i, j] = max(c(i - 1, 0), distance.euclidean(P[i], Q[0]))
            elif i == 0 and j > 0:
                dp[i, j] = max(c(0, j - 1), distance.euclidean(P[0], Q[j]))
            else:
                dp[i, j] = max(
                    min(c(i - 1, j), c(i - 1, j - 1), c(i, j - 1)), 
                    distance.euclidean(P[i], Q[j])
                )
            return dp[i, j]

        return c(len(P) - 1, len(Q) - 1)

    # def dtw_distance(self, traj1, traj2):
    #     """
    #     计算 DTW (Dynamic Time Warping) 距离
    #     """
    #     traj1, traj2 = np.array(traj1), np.array(traj2)
    #     distance, _ = fastdtw(traj1, traj2, dist=euclidean)
    #     return distance
    def path_length(self, traj):
        """
        Compute the accumulated Euclidean distance (total path length) for a trajectory.
        """
        traj = np.array(traj)
        return np.sum(np.linalg.norm(traj[1:] - traj[:-1], axis=1))  # Sum of segment distances
    
    def ndtw(self, traj1, traj2, delta=3.0):
        """
        Compute Normalized DTW (NDTW) using the equation:
        NDTW = exp(-DTW(P, R) / (L * delta))

        where:
        - P = traj1 (the navigation path)
        - R = traj2 (the best/reference path)
        - L = accumulated Euclidean distance of R
        - delta = threshold for distance normalization (default: 1.0)
        """
        traj1, traj2 = np.array(traj1), np.array(traj2)

        # Compute DTW distance
        dtw_dist, _ = fastdtw(traj1, traj2, dist=euclidean)

        # Compute accumulated Euclidean distance (total path length) of the reference path
        L = self.path_length(traj2)  

        # Avoid division by zero if the reference path is too short
        if L == 0:
            return 0.0  

        # Compute NDTW using the updated formula
        ndtw_score = np.exp(-dtw_dist / (L * delta))

        return ndtw_score

    def get_unique_filename(self, filepath):
        base, extension = os.path.splitext(filepath)
        counter = 1
        new_filepath = filepath
        
        while os.path.exists(new_filepath):
            new_filepath = f"{base}_{counter}{extension}"
            counter += 1
        
        return new_filepath

    def compare_trajectories(self, smoothed_path_final, map_path_org, smooth_user_path, VLT_path):
        """
        比较 A*、算法生成的轨迹、手绘轨迹，找出最接近手绘轨迹的
        """
        # 确保轨迹格式为 (N, 2) 形状的 NumPy 数组
        smoothed_path_final = np.array(list(zip(*smoothed_path_final))).T  # 转换为 [(x, y), ...] 格式
        map_path_org = np.array(list(zip(*map_path_org))).T
        smooth_user_path = np.array(list(zip(*smooth_user_path))).T
        VLT_path = np.array(list(zip(*VLT_path))).T

        # 计算欧几里得距离
        d_final = self.euclidean_distance(smoothed_path_final, smooth_user_path)
        d_orig = self.euclidean_distance(map_path_org, smooth_user_path)
        d_VLT = self.euclidean_distance(VLT_path, smooth_user_path)

        # calculate Frechet distance
        f_final = self.frechet_distance(smoothed_path_final, smooth_user_path)
        f_orig = self.frechet_distance(map_path_org, smooth_user_path)
        f_VLT = self.frechet_distance(VLT_path, smooth_user_path)

        # calculate NDTW distance
        # The threshold for distance normalization
        delta = 3.0
        ndtw_final = self.ndtw(smoothed_path_final, smooth_user_path, delta)
        ndtw_orig = self.ndtw(map_path_org, smooth_user_path, delta)
        ndtw_VLT = self.ndtw(VLT_path, smooth_user_path, delta)

        # dtw_astar = self.dtw_distance(map_path_astar, smooth_user_path)
        # dtw_algo = self.dtw_distance(map_path_algo, smooth_user_path)
        
        if not os.path.exists(self.cfg.alignment_result_path):
                os.makedirs(self.cfg.alignment_result_path)
        output_file = os.path.join(self.cfg.alignment_result_path, f"path_comparison_results{self.idx*10+self.cfg.start}.txt")
        unique_output_file = self.get_unique_filename(output_file)
        # output_file = f"path_comparison_results{self.idx*10+self.cfg.start}.txt"
        with open(unique_output_file, "w") as file:
            file.write(self.command + "\n")
            file.write("Path Comparison Results\n")
            file.write("="*50 + "\n")

            # Write Euclidean distances
            file.write(f"Euclidean Distance:\n")
            file.write(f"  - Our Smoothed Path: {d_final:.4f}\n")
            file.write(f"  - A* Path: {d_orig:.4f}\n")
            file.write(f"  - VLT Path: {d_VLT:.4f}\n")
            file.write("\n")

            # Write Frechet distances
            file.write(f"Frechet Distance:\n")
            file.write(f"  - Our Smoothed Path: {f_final:.4f}\n")
            file.write(f"  - A* Path: {f_orig:.4f}\n")
            file.write(f"  - VLT Path: {f_VLT:.4f}\n")
            file.write("\n")

            # Write Normalized DTW distances
            file.write(f"Normalized DTW (NDTW) with delta={delta}:\n")
            file.write(f"  - Our Smoothed Path: {ndtw_final:.4f}\n")
            file.write(f"  - A* Path: {ndtw_orig:.4f}\n")
            file.write(f"  - VLT Path: {ndtw_VLT:.4f}\n")
            file.write("\n")

            file.write("="*50 + "\n")

       
        # print(f"Euclidean distance: our: {d_final}, A*: {d_orig}, VLT: {d_VLT}")
        # print(f"Frechet distance: our: {f_final}, A*: {f_orig}, VLT: {f_VLT}")
        # return f_final, f_orig, f_VLT

        # print(f"A* 轨迹 vs 手绘轨迹 - 欧氏距离: {d_astar}, Frechet 距离: {f_astar}, DTW 距离: {dtw_astar}")
        # print(f"算法轨迹 vs 手绘轨迹 - 欧氏距离: {d_algo}, Frechet 距离: {f_algo}, DTW 距离: {dtw_algo}")

        # 选择最小的
        # scores = {
        #     "A*": d_astar + f_astar + dtw_astar,
        #     "算法轨迹": d_algo + f_algo + dtw_algo
        # }
        # scores = {
        #     "A*": d_astar + f_astar,
        #     "our": d_algo + f_algo
        # }
        # scores_1 = {
        #     "our": d_final

        # best_match = min(scores, key=scores.get)
        # print(f"The best aligned path is: {best_match}")

        # return best_match


    def generate_trajectory(self, start, end, num_points=100):
        """
        Generate a linear trajectory from start to end.
        
        Parameters:
        - start: Starting position (x, y, z, orientation)
        - end: Ending position (x, y, z, orientation)
        - num_points: Number of points in the trajectory
        
        Returns:
        - trajectory: List of trajectory points
        """
        trajectory = []
        for t in np.linspace(0, 1, num_points):
            x = (1 - t) * start[0] + t * end[0]
            y = (1 - t) * start[1] + t * end[1]
            z = (1 - t) * start[2] + t * end[2]
            orientation = (1 - t) * start[3] + t * end[3]
            trajectory.append((x, y, z, orientation))
        return trajectory

    def smooth_path_bspline(self, path, num_points=100):
        """
        使用 B 样条对路径进行平滑
        """
        path = np.array(path)
        tck, u = splprep([path[:, 0], path[:, 1]], s=3)  # s 控制平滑度
        u_new = np.linspace(0, 1, num_points)  # 生成新的采样点
        smoothed_x, smoothed_y = splev(u_new, tck)

        return list(zip(smoothed_x, smoothed_y))

    def plan_path(self, start, goal, user_preference=None, path=None):#, user_preference=-2, path=None
        """Plan a path from start to goal using A*."""
        #only task x and z
        start = (start[0], start[1])
        goal = (goal[0], goal[1])

        start = np.floor((start - self.map_origin) / self.resolution).astype(int)
        goal = np.floor((goal - self.map_origin) / self.resolution).astype(int)
        
        valid_indices = np.argwhere(self.occupancy_map == 0)
        kdtree = KDTree(valid_indices)

        # kdtree, valid_indices = self.build_kdtree_for_semantic_map(self.semantic_map, user_preference)
        _, start_index = kdtree.query(start)
        _, goal_index = kdtree.query(goal)
        start = tuple(valid_indices[start_index])
        goal = tuple(valid_indices[goal_index])
        # visualize the start and goal points in occupancy map
        user_traj = False
        if user_traj:
            drawer = TrajectoryDrawer(self.occupancy_map, start, goal)
            plt.show()  # 等待用户绘制路径
            smooth_user_path = drawer.finish_and_smooth()
            

            # plt.imshow(self.occupancy_map.T, cmap='gray', origin='lower')
            # plt.plot(start[0], start[1], 'ro', label='Start')
            # plt.plot(goal[0], goal[1], 'bo', label='Goal')
            # plt.legend()
            # plt.show()


        original_cost_map, self.cost_map = self.create_affordance_map(user_preference, path)#user_preference, path)
        inflated_cost_map = self.inflate_cost_map(self.cost_map, 0.5)
        inflated_org_cost_map = self.inflate_cost_map(original_cost_map, 0.5)
        map_path = self.a_star(inflated_cost_map, start, goal)
        map_path_org = self.a_star(inflated_org_cost_map, start, goal)
        smoothed_org_a_star = self.smooth_path_catmull_rom(map_path_org, num_points=100)
        smoothed_path_final = self.smooth_path_bspline(map_path, num_points=100)
        VLT_path = [tuple(pt) for pt in self.VLT_path_indices]
        # self.compare_trajectories(smoothed_path_final, map_path_org, VLT_path)
        # f_scores = [f_final, f_orig, f_VLT]
        #smoothed_path1 = self.smooth_path_catmull_rom(path, num_points=100)
        # our path
        
        global_path_final = np.array([[row * self.resolution + self.map_origin[0], col * self.resolution + self.map_origin[1], self.robot_state["robot_state"][0][1]] for row, col in smoothed_path_final])
        global_path_A = np.array([[row * self.resolution + self.map_origin[0], col * self.resolution + self.map_origin[1], self.robot_state["robot_state"][0][1]] for row, col in smoothed_org_a_star])
        # global_user_path = np.array([[row * self.resolution + self.map_origin[0], self.robot_state["robot_state"][0][1], col * self.resolution + self.map_origin[1]] for row, col in smooth_user_path])
        
        global_VLT_path = np.array([[row * self.resolution + self.map_origin[0], col * self.resolution + self.map_origin[1], self.robot_state["robot_state"][0][1]] for row, col in VLT_path])
        
        
        smoothed_list = [smoothed_path_final, smoothed_org_a_star, VLT_path]
        fig  = self.visualize_path(inflated_cost_map, smoothed_list, start, goal)
        if self.cfg.cost_map:
            # check if the path exists
            if not os.path.exists(self.cfg.cost_map_path):
                os.makedirs(self.cfg.cost_map_path)
            # cost_map_path = os.path.join(self.cfg.cost_map_path, f"cost_map_{self.idx*10 + self.cfg.start}.png")
            # Unique_cost_map_path = self.get_unique_filename(cost_map_path)
            fig.savefig(os.path.join(self.cfg.cost_map_path, f"cost_map_{self.idx*10 + self.cfg.start}.png"), format='png', dpi=500)
        
        self.global_path_final = global_path_final
        return map_path, global_path_final, global_path_A, global_VLT_path
    
    def plan_path_old(self, start, goal, user_preference=None, path=None):#, user_preference=-2, path=None
        """Plan a path from start to goal using A*."""
        #only task x and z
        start = (start[0], start[1])
        goal = (goal[0], goal[1])

        start = np.floor((start - self.map_origin) / self.resolution).astype(int)
        goal = np.floor((goal - self.map_origin) / self.resolution).astype(int)
        
        valid_indices = np.argwhere(self.occupancy_map == 0)
        kdtree = KDTree(valid_indices)

        # kdtree, valid_indices = self.build_kdtree_for_semantic_map(self.semantic_map, user_preference)
        _, start_index = kdtree.query(start)
        _, goal_index = kdtree.query(goal)
        start = tuple(valid_indices[start_index])
        goal = tuple(valid_indices[goal_index])
        # visualize the start and goal points in occupancy map
        user_traj = False
        if user_traj:
            drawer = TrajectoryDrawer(self.occupancy_map, start, goal)
            plt.show()  # 等待用户绘制路径
            smooth_user_path = drawer.finish_and_smooth()
            

            # plt.imshow(self.occupancy_map.T, cmap='gray', origin='lower')
            # plt.plot(start[0], start[1], 'ro', label='Start')
            # plt.plot(goal[0], goal[1], 'bo', label='Goal')
            # plt.legend()
            # plt.show()


        original_cost_map, self.cost_map = self.create_affordance_map(user_preference, path)#user_preference, path)
        inflated_cost_map = self.inflate_cost_map(self.cost_map, 0.4)
        inflated_org_cost_map = self.inflate_cost_map(original_cost_map, 0.4)
        map_path = self.a_star(inflated_cost_map, start, goal)
        map_path_org = self.a_star(inflated_org_cost_map, start, goal)
        if map_path_org is not None and len(map_path_org) > 3:
            smoothed_org_a_star = self.smooth_path_catmull_rom(map_path_org, num_points=100)
        else:
            smoothed_org_a_star = map_path_org
        smoothed_path_final = self.smooth_path_bspline(map_path, num_points=100) #map_path #
        VLT_path = [tuple(pt) for pt in self.VLT_path_indices]
        #self.compare_trajectories(smoothed_path_final, map_path_org, smooth_user_path, VLT_path)
        # f_scores = [f_final, f_orig, f_VLT]
        #smoothed_path1 = self.smooth_path_catmull_rom(path, num_points=100)
        # our path
        
        global_path_final = np.array([[row * self.resolution + self.map_origin[0], col * self.resolution + self.map_origin[1], self.robot_state["robot_state"][0][1]] for row, col in smoothed_path_final])
        global_path_A = np.array([[row * self.resolution + self.map_origin[0], col * self.resolution + self.map_origin[1], self.robot_state["robot_state"][0][1]] for row, col in smoothed_org_a_star])
        #global_user_path = np.array([[row * self.resolution + self.map_origin[0], self.robot_state["robot_state"][0][1], col * self.resolution + self.map_origin[1]] for row, col in smooth_user_path])
        
        global_VLT_path = np.array([[row * self.resolution + self.map_origin[0], col * self.resolution + self.map_origin[1], self.robot_state["robot_state"][0][1]] for row, col in VLT_path])
        
        
        smoothed_list = [smoothed_path_final, smoothed_org_a_star, VLT_path] #[smoothed_path_final, smoothed_org_a_star, smooth_user_path, VLT_path]
        fig  = self.visualize_path(inflated_cost_map, smoothed_list, start, goal)
        if self.cfg.cost_map:
            # check if the path exists
            if not os.path.exists(self.cfg.cost_map_path):
                os.makedirs(self.cfg.cost_map_path)
            # cost_map_path = os.path.join(self.cfg.cost_map_path, f"cost_map_{self.idx*10 + self.cfg.start}.png")
            # Unique_cost_map_path = self.get_unique_filename(cost_map_path)
            
            # fig.savefig(cost_map_path, format='png', dpi=500)
            fig.savefig(os.path.join(self.cfg.cost_map_path, f"cost_map_{self.idx*10 + self.cfg.start}.png"), format='png', dpi=500)
        
        #return map_path, global_path_final, global_path_A, global_user_path, global_VLT_path
        self.global_path_final = global_path_final
        return map_path, global_path_final, global_path_A, global_VLT_path

 

    def create_affordance_map(self, user_preference, path=None):
        """
        Combine semantic map, occupancy map, and terrain map into a single cost map.
        
        :param semantic_map: Semantic map where each value corresponds to a specific region type.
        :param occupancy_map: Binary map where 1 indicates obstacles and 0 indicates free space.
        :param terrain_map: Map of terrain heights or difficulties.
        :param user_preference: Dictionary of user-preferred regions with weights.
        :return: Combined cost map.
        """
        cost_map = np.full_like(self.occupancy_map, 10, dtype=float)

        # Assign costs based on semantic regions and user preferences
        # To do
        # for region, cost in user_preference.items():
        #     cost_map[self.semantic_map == region] = cost
        # cost_map = self.update_original_cost_map(cost_map, path, original_res=0.1, new_res=1.0, cost_factor=0.3)
        # Inflate costs for obstacles in occupancy map

        cost_map[self.occupancy_map == 1] = 10
        cost_map[self.occupancy_map == -1] = 8
        cost_map[self.occupancy_map == 0] = 3
        if user_preference:
            for reg in user_preference:
                cost_map[self.semantic_map == reg] = 10
        original_cost_map = cost_map.copy()
        if len(path):
            cost_map, self.VLT_path_indices = self.update_original_cost_map(cost_map, path)
        #self.cost_map 
        
        # Add terrain costs (e.g., penalties for height differences)
        #cost_map += self.terrain_map 
        
        return original_cost_map, cost_map

    def visualize_map_and_path(self, path=None):
        """Visualize the cost map and the planned path."""
        plt.figure(figsize=(10, 10))
        cost_map_display = np.copy(self.cost_map)
        plt.imshow(cost_map_display.T, cmap='magma_r', origin='lower', interpolation='bilinear')
        if path:
            path_x, path_y = zip(*path)
            plt.plot(path_x, path_y, color='blue', linewidth=2, label='Path')
        cbar = plt.colorbar()
        cbar.set_label("Cost Value", rotation=270, labelpad=15)
        plt.title("Path Planning on Affordance Map")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.legend()
        plt.show()

    # def generate_bspline_trajectory_3d(self, coordinates: list, num_points: int) -> list:
    #     """
    #     Generate a smooth 3D trajectory using B-spline interpolation based on a list of 3D coordinates.
    #     This function returns a trajectory as a list of 3D points (tuples), which can be directly passed to api.execute_trajectory.

    #     Args:
    #         coordinates (list): A list of (x, y, z) coordinates representing the waypoints.
    #         num_points (int): The number of points to generate in the smooth trajectory.

    #     Returns:
    #         list: A list of 3D points (tuples) representing the smooth trajectory.
    #     """
    #     # Convert the list of coordinates to NumPy arrays
    #     coordinates = np.array(coordinates)
    #     x, y, z = coordinates[:, 0], coordinates[:, 1], coordinates[:, 2]
        
    #     # Generate a periodic B-spline for 3D (closed loop)
    #     tck, _ = splprep([x, y, z], s=0.1, per=True)  # 'per=True' ensures a closed trajectory
    #     u_new = np.linspace(0, 1, num_points)
    #     x_smooth, y_smooth, z_smooth = splev(u_new, tck)
    #     # Combine the smooth x, y, z into a list of 3D points
    #     trajectory = [(x_smooth[i], y_smooth[i], z_smooth[i]) for i in range(num_points)]
        
    #     return trajectory
    
    def prepare_question(self):
        markers = {}
        theta = np.round(self.robot_state["robot_state"][1], 2)
        for i, info in enumerate(self.sematic_info):
            if info["img_bbox"] in self.drivable_indices:
                markers.update(info["SoM_label"])
        formatted_questions = self.generate_mcq([markers], theta[0])
        # convert the formatted_questions to a string
        formatted_questions_str = '\n'.join(formatted_questions)
        return formatted_questions_str
    
    def print_grid_map(self):
        print(self.generate_map_prompt(original_res=0.1, new_res=1.0))

    def find_nearest_region_point(self, point, target_label, hight):

        # 获取所有区域 6 的坐标
        point  = np.floor((point - self.map_origin) / self.resolution).astype(int)
        target_points = np.column_stack(np.where(self.semantic_map == target_label))

        if len(target_points) == 0:
            raise ValueError(f"Map does not have area with {target_label}！")

        # 使用 KD-Tree 快速查找最近点
        tree = KDTree(target_points)
        dist, idx = tree.query(point)  # 计算 point 到最近区域6点的距离和索引
        nearest_point = tuple(target_points[idx])
        global_point = np.array(nearest_point) * self.resolution + self.map_origin
        # keep 2 decimal
        global_point = np.round(global_point, 1)
        global_point = (global_point[0], np.round(hight), global_point[1])

        return global_point

    def find_nearest_point_to_robot(self, available_points: dict, robot_xy: np.ndarray):
        
        min_dist = float('inf')
        nearest_point = None

        for road_id, points in available_points.items():
            if points is None:
                continue

            if isinstance(points[0], (list, tuple, np.ndarray)):  # 多个点
                for pt in points:
                    pt_xy = np.array(pt[:2])
                    dist = np.linalg.norm(robot_xy - pt_xy)
                    if dist < min_dist:
                        min_dist = dist
                        nearest_point = pt
            else:  
                pt_xy = np.array(points[:2])
                dist = np.linalg.norm(robot_xy - pt_xy)
                if dist < min_dist:
                    min_dist = dist
                    nearest_point = points

        return nearest_point
    


    def world_to_map(self, x, y, origin, resolution):
        return int(np.floor((x - origin[0]) / resolution)), int(np.floor((y - origin[1]) / resolution))

    def map_to_world(self, mx, my, origin, resolution):
        return mx * resolution + origin[0], my * resolution + origin[1]

    def find_nearest_free_point_near_object(self, four_corners, occupancy_map, map_origin, resolution, robot_position):
        polygon = Polygon(four_corners)
        height, width = occupancy_map.shape
        free_points_world = []

        # Step 1: try to find free space near the object
        for mx in range(height):
            for my in range(width):
                if occupancy_map[mx, my] != 0:
                    continue

                wx, wy = self.map_to_world(mx, my, map_origin, resolution)
                point = Point(wx, wy)

                # Nearby (within 0.5 m) but not inside the object
                if not polygon.contains(point) and polygon.exterior.distance(point) < 0.4:
                    free_points_world.append((wx, wy))

        # Step 2: fallback to entire map if nothing found nearby
        if not free_points_world:
            print("⚠️ No free space near object, fallback to full map search.")
            cx, cy = polygon.centroid.x, polygon.centroid.y
            for mx in range(height):
                for my in range(width):
                    if occupancy_map[mx, my] != 0:
                        continue
                    wx, wy = self.map_to_world(mx, my, map_origin, resolution)
                    free_points_world.append((wx, wy))

            if not free_points_world:
                raise RuntimeError("No free space in the entire map.")

            # Find closest free point to object center
            closest = min(free_points_world, key=lambda p: (p[0] - cx)**2 + (p[1] - cy)**2)
            return (round(closest[0], 1), round(closest[1], 1), round(robot_position[2], 1))

        # Step 3: if nearby free points exist, find closest to robot
        robot_x, robot_y, robot_z = robot_position
        closest = min(free_points_world, key=lambda p: (p[0] - robot_x)**2 + (p[1] - robot_y)**2)
        return (round(closest[0], 1), round(closest[1], 1), round(robot_z, 1))




    def detect_object_label(self, activate = True):
        #from LLT.prompts.example import Example
        #print(Example)
        #self.robot_state1 = self.matrix_to_xyz_yaw(self.pose)
        # x= self.robot_state['robot_state'][0][0]
        # y= self.robot_state['robot_state'][0][1]
        # z= self.robot_state['robot_state'][0][2]
        x, y, z = np.round(self.robot_state["robot_state"][0], 2)
        #y = y - 1.0
        theta = np.round(self.robot_state["robot_state"][1], 2)
        #print(f'Current position of Husky robot: {(x, y, z, theta[0])}, Husky A200 physical dimensions: length: 1.0m, width: 0.7m, height: 1.0 m\n')
        label_text = []
        label_gradio = []
        label_text_num=[]
        self.object_dimention = {}
        save_txt = False
        location_info = f'Current position of Husky robot: {(x, y, z)} orientation rpy: {theta} rad, Husky A200 physical dimensions: length: 1.0m, width: 0.7m, height: 1.0 m\n'
        print(location_info)
        label_text.append(location_info)
        label_gradio.append(location_info)
        label_text_num.append(location_info)    
        for obj in (self.objects+self.bg_list):
            # check if obj belongs to self.bg_list
            
            #if self.objects[i]['image_idx'][-1] == self.idx:
            # if the bbox is available
            bbox = obj['bbox']  # OrientedBoundingBox object
            rounded_center = np.round(bbox.center, 1)  # Center is a 3D point (x, y, z)
            rounded_extent = np.round(bbox.extent, 1)
            rotation_matrix = np.array(bbox.R, copy=True)
            # euler_angles = R.from_matrix(rotation_matrix).as_euler('zyx', degrees=False)
            euler_angles = R.from_matrix(rotation_matrix).as_euler('xyz', degrees=False)
            rounded_orientation = np.round(euler_angles, 1)

            # corners = np.round(np.asarray(bbox.get_box_points())[:, [0, 1]], 1) 
             
            # min_xz = np.min(corners, axis=0)
            # max_xz = np.max(corners, axis=0)
            # four_corners = ([min_xz[0], min_xz[1]], [max_xz[0], min_xz[1]], [max_xz[0], max_xz[1]], [min_xz[0], max_xz[1]])
            corners = np.round(np.asarray(bbox.get_box_points())[:, [0, 1]], 3)

            min_xz = np.min(corners, axis=0)
            max_xz = np.max(corners, axis=0)

            # Ensure box has non-zero area
            eps = 0.05  # Small buffer (e.g., 5 cm)
            if abs(max_xz[0] - min_xz[0]) < 1e-3:
                min_xz[0] -= eps
                max_xz[0] += eps
            if abs(max_xz[1] - min_xz[1]) < 1e-3:
                min_xz[1] -= eps
                max_xz[1] += eps

            four_corners = [
                [min_xz[0], min_xz[1]],
                [max_xz[0], min_xz[1]],
                [max_xz[0], max_xz[1]],
                [min_xz[0], max_xz[1]]
            ]

            caption_obj = obj["caption"]
            if ", " in caption_obj:
                last_caption = caption_obj[caption_obj.rfind(', ')+2:]
            else:
                last_caption = caption_obj
                # # if too many observations, the string will be too long, then need to delete some observations in the front
                # comma_count = caption_obj.count(', ')
                # if comma_count > self.cfg.VLT_caption_num:
                #     num_last_comma_index = 0
                #     for j in range(self.cfg.VLT_caption_num):
                #         num_last_comma_index = caption_obj.rfind(', ', 0, num_last_comma_index - 1)
                #     # keep the last one
                #     caption_obj = caption_obj[num_last_comma_index+1:]
                
            #print('label-'+str(self.objects[i]['img_bbox']) +':'+last_caption+';', 'Center of 3D bbox:', rounded_center, ';')   
        
            output_txt0 =f'label-{str(obj["img_bbox"])}; 3D bbox info(center: {rounded_center}(meters), extent (length, width and height): {rounded_extent}(meters), orientation: {rounded_orientation[0]}(rad))'
            output_txt =f'label-{str(obj["img_bbox"])}: {last_caption}; 3D bbox info(center: {rounded_center}(meters), extent (length, width and height): {rounded_extent}(meters), orientation: {rounded_orientation[0]}(rad))'
            output_txt_gradio = f'label-{str(obj["img_bbox"])}: {last_caption}; Center:{rounded_center}(m))'
            # output_txt = f'label-{str(obj["img_bbox"])}: {last_caption};'
            label_text.append(output_txt)
            label_text_num.append(output_txt0)
            
            self.object_dimention[obj["img_bbox"]] = {'center': rounded_center, 'extent': rounded_extent, 'orientation': rounded_orientation}
            bg_ids = {bg["img_bbox"] for bg in self.bg_list}
            if obj["img_bbox"] in bg_ids:
            # if obj in self.bg_list:
                # output_txt =f'label-{str(obj["img_bbox"])}: {last_caption}; 3D bbox info(center: {rounded_center}(meters), extent (length, width and height): {rounded_extent}(meters), orientation: {rounded_orientation}(rad))'
                output_txt =f'label-{str(obj["img_bbox"])}: {last_caption}; 3D bbox info(center: {rounded_center}(meters), extent (length, width and height): {rounded_extent}(meters)'
                output_txt_gradio = f'label-{str(obj["img_bbox"])}: {last_caption}; Center:{rounded_center})'
            else:
                # available_points = {}
                # for road in self.drivable_indices:
                #     available_point = self.check_points_in_semantic_map(obj["img_bbox"], road, y)
                #     if available_point != []:
                #         available_points[road] = tuple(available_point)
                #     else:
                #         available_points[road] = self.find_nearest_region_point((rounded_center[0], rounded_center[1]), road, z)
                robot_xy = np.array([x, y, z])
                goal = self.find_nearest_free_point_near_object(
                        four_corners=four_corners,
                        occupancy_map=self.occupancy_map,
                        map_origin= self.map_origin,
                        resolution= self.resolution,
                        robot_position=(x, y, z)
                    )
                # print("Goal:", goal)
                nearest_point = goal
                # nearest_point = self.find_nearest_point_to_robot(available_points, robot_xy)
                output_txt =f'label-{str(obj["img_bbox"])}: {last_caption}; 3D bbox info(center: {rounded_center}(meters), extent (length, width and height): {rounded_extent}(meters), {nearest_point}' #available_points
                output_txt_gradio = f'label-{str(obj["img_bbox"])}: {last_caption}; Center:{rounded_center}, POI: {nearest_point}'
                f'label-{str(obj["img_bbox"])}: {last_caption}; 3D bbox info(center: {rounded_center}(meters), extent (length, width and height): {rounded_extent}(meters), {nearest_point}'
                # output_txt =f'label-{str(obj["img_bbox"])} 3D bbox info(center: {rounded_center}(meters), extent (length, width and height): {rounded_extent}(meters), {available_points}'
                # output_txt = f'label-{str(obj["img_bbox"])}: {last_caption};  The nearest point in each ground region to the label-{str(obj["img_bbox"])} {available_points}'
                # output_txt = f'label-{str(obj["img_bbox"])};  {str(obj["img_bbox"])} {available_points}'
                #self.available_points = available_points
            if activate:
                print(output_txt)
            label_gradio.append(output_txt_gradio)
        # Save labels to file to sync with GUI
        label_txt_path = os.path.join(self.cfg.label_txt_path, f"labels_{self.idx*10+self.cfg.start}.txt")
        os.makedirs(os.path.dirname(label_txt_path), exist_ok=True)
        with open(label_txt_path, "w") as f:
            f.write("\n".join(label_gradio))
        if save_txt:
            path_txt = '/home/mfyuan/local_folder/results/05/matrix_txt/semantic_map_label'+str(self.idx)+'.txt'
            with open(path_txt, "w") as file:
                file.write("\n".join(label_text))  # 以换行符拼接所有行，并写入文件
            path_txt_num = '/home/mfyuan/local_folder/results/05/matrix_txt/semantic_map_label_num'+str(self.idx)+'.txt'
            with open(path_txt_num, "w") as file:
                file.write("\n".join(label_text_num))

        # for i, info in enumerate(self.sematic_info):
        #     #if info["cap"] == "driveable area":
        #     if info["img_bbox"] in self.drivable_indices:
        #         output = ', '.join(f"{key}: ({value[0]:.2f}, {value[1]:.2f}, {value[2]:.2f})" for key, value in info["SoM_label"].items())
        #         markers.append(output)
            #if len(info["SoM_label"]) > 0:
        
        # road_boundary = self.drivable_area[0]['bbox']
        # corners = np.round(np.asarray(road_boundary.get_box_points())[:, [0, 2]], 1)
        # min_xz = np.min(corners, axis=0)
        # max_xz = np.max(corners, axis=0)
        # rounded_center = np.round(road_boundary.center, 1)  # Center is a 3D point (x, y, z)
        # four_corners = ([min_xz[0], min_xz[1]], [max_xz[0], min_xz[1]], [max_xz[0], max_xz[1]], [min_xz[0], max_xz[1]])
        #print('label-'+str(i+2) +':'+'Driveable road area;', 'Center of 3D bbox:', rounded_center, ';', 'Driveable area', four_corners)   
    
    def generate_mcq(self, data, theta):
        questions = []
        if math.pi/4 <= theta < 3*math.pi/4:
            # Group points by the second letter of their label
            grouped_points = {}
            for label, coords in data[0].items():
                second_letter = label[1]
                if second_letter not in grouped_points:
                    grouped_points[second_letter] = []
                grouped_points[second_letter].append((label, coords))

            # Sort by the second letter of the label
            sorted_second_letters = sorted(grouped_points.keys())
            

            for idx, second_letter in enumerate(sorted_second_letters, start=1):
                options = sorted(grouped_points[second_letter], key=lambda x: x[0][0])
                question = f"{idx}. " + " ".join(
                    [f"{chr(65 + i)}: {label} {coords}" for i, (label, coords) in enumerate(options)]
                ) + f" {chr(65 + len(options))}: skip this point"
                questions.append(question)
        elif -math.pi / 4 <= theta < math.pi / 4:
            # Group points by the first letter of their label
            grouped_points = {}
            for label, coords in data[0].items():
                first_letter = label[0]
                if first_letter not in grouped_points:
                    grouped_points[first_letter] = []
                grouped_points[first_letter].append((label, coords))

            # Sort by the first letter of the label
            sorted_first_letters = sorted(grouped_points.keys())
            
            for idx, first_letter in enumerate(sorted_first_letters, start=1):
                options = sorted(grouped_points[first_letter], key=lambda x: x[0][1])
                question = f"{idx}. " + " ".join(
                    [f"{chr(65 + i)}: {label} {coords}" for i, (label, coords) in enumerate(options)]
                ) + f" {chr(65 + len(options))}: skip this point"
                questions.append(question)
        return questions

    def execute_trajectory(self, trajectory=None):
        """
        Execute the generated trajectory.

        Args:
            trajectory (list): A list of [x, y, z, theta] points.
        """
        if trajectory is None:
            print('Trajectory is None. Please generate a trajectory first.')
            return
        self.trajectory = trajectory
        #print('Executing trajectory:', trajectory)

    def smooth_3d_points(self, trajectory, window_size=10, poly_order=3):
        """
        Smooths a 3D trajectory while keeping the same input format.

        :param points: NumPy array of shape (N, 3), where each row is (x, y, z).
        :param window_size: Window size for smoothing (must be odd and <= number of points).
        :param poly_order: Polynomial order for the filter.
        :return: Smoothed NumPy array of shape (N, 3), same as input format.
        """
        from scipy.signal import savgol_filter
        if len(trajectory[0]) == 3:
            points = np.array([[x, y, z] for x, y, z in trajectory])
        else:
            points = np.array([[x, y, z] for x, y, z, theta in trajectory])
        points = np.asarray(points)  # Ensure input is a NumPy array
        if len(points) < window_size:  # Ensure window size is valid
            window_size = len(points) // 2 * 2 + 1  # Adjust to the largest valid odd number
        
        # Apply Savitzky-Golay filter to each coordinate (x, y, z)
        x_smooth = savgol_filter(points[:, 0], window_size, poly_order)
        y_smooth = savgol_filter(points[:, 1], window_size, poly_order)
        z_smooth = savgol_filter(points[:, 2], window_size, poly_order)

        # Return smoothed points in the same format
        return np.column_stack((x_smooth, y_smooth, z_smooth))



    def visualize_trajectory(self, point_clouds, trajectory, target_obj_pose=None, undriveable=None):
        """
        Visualize a trajectory using Open3D's draw_geometries.
        
        Args:
            point_clouds (list): A list of PointCloud objects.
            trajectory (list): A list of [x, y, z, theta] points.
        """
        if trajectory is None:
            print('Trajectory is None. Please generate a trajectory first.')
            return

        # Extract [x, y, z] from trajectory (ignoring theta for visualization)
        if len(trajectory[0]) == 3:
            points = np.array([[x, y, z] for x, y, z in trajectory])
        else:
            points = np.array([[x, y, z] for x, y, z, _ in trajectory])

        # Plan the path
        map_path, global_path_final, global_path_A, global_VLT_path = self.plan_path(points[0], points[-1], user_preference=undriveable, path=points)

        # Convert global path to Open3D format
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(global_path_final)

        # Create a LineSet object to visualize connections between points
        lines = [[i, i + 1] for i in range(len(global_path_final) - 1)]
        line_set = o3d.geometry.LineSet()
        line_set.points = o3d.utility.Vector3dVector(global_path_final)
        line_set.lines = o3d.utility.Vector2iVector(lines)
        line_set.paint_uniform_color([0, 1, 0])  # Green lines

        # Create a red sphere at the **start point**
        start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
        start_sphere.translate(global_path_final[0])  # Move to the start point
        start_sphere.paint_uniform_color([1, 0, 0])  # Red color

        if target_obj_pose is not None:
            end_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
            # end_sphere.translate(target_obj_pose)  # Move to the end point
            end_sphere.translate(np.array(target_obj_pose[:3]))  # Move to the end point

            end_sphere.paint_uniform_color([0, 0, 1])  # Blue color

        # Create green spheres at intermediate points
        middle_spheres = []
        for i in range(1, len(global_path_final) - 1):  # Skip start and end points
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)  # Smaller radius
            sphere.translate(global_path_final[i])  # Move to the trajectory point
            sphere.paint_uniform_color([0, 1, 0])  # Green color
            middle_spheres.append(sphere)
        # Visualize the trajectory
       
        if self.cfg.history_3D:
            # Extend point cloud list with new objects
            if target_obj_pose is not None:
                point_clouds.extend([point_cloud, line_set, start_sphere, end_sphere] + middle_spheres)
            else:
                point_clouds.extend([point_cloud, line_set, start_sphere] + middle_spheres)
            vis = o3d.visualization.draw_geometries(point_clouds)
        else:
            if target_obj_pose is not None:
                visualization_objects = point_clouds + [point_cloud, line_set, start_sphere, end_sphere]
            else:
                visualization_objects = point_clouds + [point_cloud, line_set, start_sphere]

            vis = o3d.visualization.VisualizerWithKeyCallback()
            vis.create_window()

            for geom in visualization_objects:
                vis.add_geometry(geom)

            view_control = vis.get_view_control()

            # === 设置初始视角（用于后续旋转） ===
            lookat_point = np.array(global_path_final[0])
            radius = 0.2
            angle = [0.0]
            paused = [False]
            should_exit = [False]

            # 设置初始摄像机位置（平视）
            cam_x = lookat_point[0] + radius * math.cos(angle[0])
            cam_y = lookat_point[1] + radius * math.sin(angle[0])
            cam_z = lookat_point[2] + 1.0  # ✅ 与 lookat 在同一高度，平视
            cam_pos = np.array([cam_x, cam_y, cam_z])
            front = (lookat_point - cam_pos)
            front /= np.linalg.norm(front)

            view_control.set_lookat(lookat_point)
            view_control.set_front(front)
            view_control.set_up(np.array([0, 0, 1]))  # ✅ Z轴朝上
            view_control.set_zoom(0.45)

         
            def toggle_pause(vis):
                paused[0] = not paused[0]
                print("Paused" if paused[0] else "Resumed")

            def close_window(vis):
                print("Exiting visualization...")
                should_exit[0] = True
                vis.destroy_window()

            vis.register_key_callback(32, toggle_pause)
            vis.register_key_callback(113, close_window)  # Q退出

            print("📍 Showing trajectory step-by-step...")

            # === 逐个显示 middle_spheres 和更新线条 ===
            points = list(global_path_final[:1])  # 起点
            lines = []

            for i in range(1, len(global_path_final)-1):
                while paused[0]:

                    vis.poll_events()
                    vis.update_renderer()
                    time.sleep(0.1)

                # 添加当前点与连线
                points.append(global_path_final[i])
                lines.append([i - 1, i])

                # 添加中间绿色球体
                vis.add_geometry(middle_spheres[i - 1])

                # 更新 line_set 和 point_cloud
                line_set.points = o3d.utility.Vector3dVector(points)
                line_set.lines = o3d.utility.Vector2iVector(lines)
                point_cloud.points = o3d.utility.Vector3dVector(points)

                vis.update_geometry(point_cloud)
                vis.update_geometry(line_set)
                vis.poll_events()
                vis.update_renderer()
                time.sleep(0.2)

            print("✅ Trajectory fully visualized. Starting horizontal rotation...")
            while True:
                if paused[0]:
                    break
                vis.poll_events()
                vis.update_renderer()
                time.sleep(0.1)
            swing_range = math.radians(20)  # 最大±20度
            swing_speed = 0.005  # 摆动速度


            while False:
            # while not vis.was_stopped() and not should_exit[0]:
                vis.poll_events()
                vis.update_renderer()

                if not paused[0]:
                    angle[0] += swing_speed
                    offset_angle = math.sin(angle[0]) * swing_range

                    # 在 XY 平面左右摆动，Z 坐标保持不变（平视）
                    cam_x = lookat_point[0] + radius * math.cos(offset_angle)
                    cam_y = lookat_point[1] + radius * math.sin(offset_angle)
                    cam_z = lookat_point[2]  # ✅ 不改变高度

                    cam_pos = np.array([cam_x, cam_y, cam_z])
                    front = lookat_point - cam_pos
                    front /= np.linalg.norm(front)

                    view_control.set_lookat(lookat_point)
                    view_control.set_front(front)
                    view_control.set_up(np.array([0, 0, 1]))  # ✅ 保持Z向上
                    view_control.set_zoom(0.45)

                time.sleep(0.01)
            '''if target_obj_pose is not None:
                visualization_objects = point_clouds + [point_cloud, line_set, start_sphere, end_sphere]
            else:
                visualization_objects = point_clouds + [point_cloud, line_set, start_sphere]
            vis = o3d.visualization.VisualizerWithKeyCallback()
            vis.create_window()

            for geom in visualization_objects:
                vis.add_geometry(geom)

            view_control = vis.get_view_control()

            # === 设置初始视角（用于后续旋转） ===
            lookat_point = np.array(global_path_final[0])
            radius = 0.2
            angle = [0.0]
            paused = [False]

            # 设置到最终旋转视角（但暂时不转动）
            cam_x = lookat_point[0] + radius * math.cos(angle[0])
            cam_z = lookat_point[2] + radius * math.sin(angle[0])
            cam_y = lookat_point[1] + 1.0
            cam_pos = np.array([cam_x, cam_y, cam_z])
            front = (lookat_point - cam_pos)
            front /= np.linalg.norm(front)
            view_control.set_lookat(lookat_point)
            view_control.set_front(front)
            view_control.set_up(np.array([0, -1, 0]))
            view_control.set_zoom(0.45)

            # 空格暂停
            def toggle_pause(vis):
                paused[0] = not paused[0]
                print("Paused" if paused[0] else "Resumed")

            vis.register_key_callback(32, toggle_pause)

            print("📍 Showing trajectory step-by-step...")

            # === 逐个显示 middle_spheres 和更新线条 ===
            points = list(global_path_final[:1])  # 起点
            lines = []

            for i in range(1, len(global_path_final)-1):
                while paused[0]:
                    vis.poll_events()
                    vis.update_renderer()
                    time.sleep(0.1)

                # 添加当前点与连线
                points.append(global_path_final[i])
                lines.append([i - 1, i])

                # 添加中间绿色球体
                vis.add_geometry(middle_spheres[i - 1])

                # 更新 line_set 和 point_cloud
                line_set.points = o3d.utility.Vector3dVector(points)
                line_set.lines = o3d.utility.Vector2iVector(lines)
                point_cloud.points = o3d.utility.Vector3dVector(points)

                vis.update_geometry(point_cloud)
                vis.update_geometry(line_set)
                vis.poll_events()
                vis.update_renderer()
                time.sleep(0.2)

            print("✅ Trajectory fully visualized. Starting rotation...")
            swing_range = math.radians(20)  # 最大±20度
            swing_speed = 0.02  # 摆动速度

            while True:
                vis.poll_events()
                vis.update_renderer()

                if not paused[0]:
                    angle[0] += swing_speed
                    offset_angle = math.sin(angle[0]) * swing_range

                    # 在 XY 平面轻微左右摆动相机位置，Z 保持不变
                    cam_x = lookat_point[0] + radius * math.cos(offset_angle)
                    cam_y = lookat_point[1] + radius * math.sin(offset_angle)
                    cam_z = lookat_point[2] + 3.0  # 俯视高度

                    cam_pos = np.array([cam_x, cam_y, cam_z])
                    front = lookat_point - cam_pos
                    front /= np.linalg.norm(front)

                    view_control.set_lookat(lookat_point)
                    view_control.set_front(front)
                    view_control.set_up(np.array([0, 1, 0]))  # Z轴为上
                    view_control.set_zoom(0.45)

                time.sleep(0.01)'''

            # === 轨迹完成后开始旋转视角 ===
            # while True:
            #     vis.poll_events()
            #     vis.update_renderer()

            #     if not paused[0]:
            #         angle[0] += 0.01
            #         cam_x = lookat_point[0] + radius * math.cos(angle[0])
            #         cam_y = lookat_point[1] + radius * math.sin(angle[0])
            #         cam_z = lookat_point[2] + 1.0

            #         cam_pos = np.array([cam_x, cam_y, cam_z])
            #         front = (lookat_point - cam_pos)
            #         front /= np.linalg.norm(front)

            #         view_control.set_lookat(lookat_point)
            #         view_control.set_front(front)
            #         view_control.set_up(np.array([0, 0, -1]))
            #         view_control.set_zoom(0.45)

            #     time.sleep(0.01)
        # else:
        # # Store only visualization objects in a new list (not modifying point_clouds)
            # visualization_objects = point_clouds + [point_cloud, line_set, start_sphere] + middle_spheres
            # # vis = o3d.visualization.draw_geometries(visualization_objects)
            # vis = o3d.visualization.VisualizerWithKeyCallback()
            # vis.create_window()

            # for geom in visualization_objects:
            #     vis.add_geometry(geom)

            # view_control = vis.get_view_control()

            # # 初始视角位置和方向
            # lookat_point = np.array(global_path_final[0])  # 初始观察目标
            # radius = 2.0  # 旋转半径（摄像机距目标点的距离）
            # angle = [0.0]  # 初始角度（用 list 以便在闭包中修改）

            # paused = [False]

            # def toggle_pause(vis):
            #     paused[0] = not paused[0]
            #     print("Paused" if paused[0] else "Resumed")

            # vis.register_key_callback(32, toggle_pause)  # 空格键暂停/继续

            # print("Press [SPACE] to pause/resume visualization. Press [Q] to quit.")

            # # 主可视化循环
            # while True:
            #     if not paused[0]:
            #         # 计算新的相机位置（绕着 lookat_point 旋转）
            #         angle[0] += 0.05  # 每次旋转一点
            #         cam_x = lookat_point[0] + radius * math.cos(angle[0])
            #         cam_z = lookat_point[2] + radius * math.sin(angle[0])
            #         cam_y = lookat_point[1] + 1.0  # 保持高度稍高

            #         cam_pos = np.array([cam_x, cam_y, cam_z])
            #         front = (lookat_point - cam_pos)
            #         front /= np.linalg.norm(front)

            #         view_control.set_lookat(lookat_point)
            #         view_control.set_front(front)
            #         view_control.set_up(np.array([0, -1, 0]))  # Y朝下符合Open3D默认
            #         view_control.set_zoom(0.45)

            #         vis.poll_events()
            #         vis.update_renderer()
            #         time.sleep(0.05)
            #     else:
            #         vis.poll_events()
            #         vis.update_renderer()
            #         time.sleep(0.1)

            
    def visualize_3D_map(self, point_clouds):
        """
        Visualize a trajectory using Open3D's draw_geometries.
        
        Args:
            point_clouds (list): A list of PointCloud objects.
            trajectory (list): A list of [x, y, z, theta] points.
        """
        def create_start_arrow(position, theta, length=1.0):
            """
            Create a 3D arrow representing the robot's heading.
            
            Args:
                position: (x, y, z) numpy array
                theta: heading angle in radians (around Y-axis)
                length: length of the arrow
            Returns:
                o3d.geometry.TriangleMesh arrow
            """
            # 创建一个箭头模型
            arrow = o3d.geometry.TriangleMesh.create_arrow(
                cylinder_radius=0.05,
                cone_radius=0.1,
                cylinder_height=length * 0.8,
                cone_height=length * 0.2
            )
            arrow.paint_uniform_color([0, 0, 1])  # 蓝色表示朝向

            # 旋转（绕Y轴）
            R = arrow.get_rotation_matrix_from_axis_angle([0, -theta, 0])  # 注意方向
            arrow.rotate(R, center=(0, 0, 0))

            # 平移到起始点
            arrow.translate(position)

            return arrow
  
        x, y, z = np.round(self.robot_state["robot_state"][0], 2)
        start_point = np.array([x, y+0.8, z])   
        theta = np.round(self.robot_state["robot_state"][1], 2)[0]
        
        #Create a red sphere at the **start point**
        start_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
        start_sphere.translate(start_point)  # Move to the start point
        start_sphere.paint_uniform_color([1, 0, 0])  # Red color
        heading_arrow = create_start_arrow(start_point, theta, length=0.3)

        visualization_objects = point_clouds# + [start_sphere]
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window()

        for geom in visualization_objects:
            vis.add_geometry(geom)

        view_control = vis.get_view_control()
        lookat_point = start_point  # 初始观察目标
        radius = 0.01
        angle = [theta]
        paused = [False]

        # 空格暂停
        def toggle_pause(vis):
            paused[0] = not paused[0]
            print("Paused" if paused[0] else "Resumed")

        vis.register_key_callback(32, toggle_pause)
        print("📍 Showing trajectory step-by-step...")

        # === 轨迹完成后开始旋转视角 ===
        while True:
            vis.poll_events()
            vis.update_renderer()

            if not paused[0]:
                angle[0] += 0.001
                cam_x = lookat_point[0] + radius * math.cos(angle[0])
                cam_z = lookat_point[2] + radius * math.sin(angle[0])
                cam_y = lookat_point[1] #+ 0.5

                cam_pos = np.array([cam_x, cam_y, cam_z])
                front = (lookat_point - cam_pos)
                front /= np.linalg.norm(front)

                view_control.set_lookat(lookat_point)
                view_control.set_front(front)
                view_control.set_up(np.array([0, -1, 0]))
                view_control.set_zoom(0.05)

            time.sleep(0.001)
    


    def matrix_to_xyz_yaw(self):
        x, y, z = np.round(self.robot_state["robot_state"][0], 2)
        theta = np.round(self.robot_state["robot_state"][1], 2)#[0]
        return [x, y, z, theta]
    
    def matrix_to_xyz_yaw_old(self, T):
        """
        Convert a 4x4 transformation matrix to x, y, z, and yaw.

        Args:
            T (np.ndarray): 4x4 transformation matrix.

        Returns:
            tuple: (x, y, z, yaw), where yaw is in radians.
        """
        # Extract translation
        x = T[0, 3]  # Right
        y = T[1, 3]  # Downward
        z = T[2, 3]  # Forward

        # Extract yaw
        yaw = np.arctan2(T[2, 0], T[2, 2])  # atan2(r31, r33)
        print('Current position of Husky robot:', np.round([x, y, z, yaw], 3), 'Husky A200 physical dimensions: length: 1.0m, width: 0.7m, height: 1.0 m\n')
        return np.round([x, y, z, yaw], 1)

    def task_completed(self):
        print('Task completed. completed_task is set to True.')
        self.completed_task = True
    

    # def detect_object(self, segmentation_text):

    #     self.logger.info(PROGRESS + "Capturing head and wrist camera images..." + ENDC)
    #     self.main_connection.send([CAPTURE_IMAGES])
    #     [head_camera_position, head_camera_orientation_q, wrist_camera_position, wrist_camera_orientation_q, env_connection_message] = self.main_connection.recv()
    #     self.logger.info(env_connection_message)

    #     self.head_camera_position = head_camera_position
    #     self.head_camera_orientation_q = head_camera_orientation_q
    #     self.wrist_camera_position = wrist_camera_position
    #     self.wrist_camera_orientation_q = wrist_camera_orientation_q

    #     rgb_image_head = Image.open(config.rgb_image_head_path).convert("RGB")
    #     depth_image_head = Image.open(config.depth_image_head_path).convert("L")
    #     depth_array = np.array(depth_image_head) / 255.

    #     if self.segmentation_count == 0:
    #         xmem_image = Image.fromarray(np.zeros_like(depth_array)).convert("L")
    #         xmem_image.save(config.xmem_input_path)

    #     segmentation_texts = [segmentation_text]

    #     self.logger.info(PROGRESS + "Segmenting head camera image..." + ENDC)
    #     model_predictions, boxes, segmentation_texts = models.get_langsam_output(rgb_image_head, self.langsam_model, segmentation_texts, self.segmentation_count)
    #     self.logger.info(OK + "Finished segmenting head camera image!" + ENDC)

    #     masks = utils.get_segmentation_mask(model_predictions, config.segmentation_threshold)

    #     bounding_cubes_world_coordinates, bounding_cubes_orientations = utils.get_bounding_cube_from_point_cloud(rgb_image_head, masks, depth_array, self.head_camera_position, self.head_camera_orientation_q, self.segmentation_count)

    #     utils.save_xmem_image(masks)

    #     self.segmentation_texts.extend(segmentation_texts)

    #     self.logger.info(PROGRESS + "Adding bounding cubes to the environment..." + ENDC)
    #     self.main_connection.send([ADD_BOUNDING_CUBES, bounding_cubes_world_coordinates])
    #     [env_connection_message] = self.main_connection.recv()
    #     self.logger.info(env_connection_message)

    #     for i, bounding_cube_world_coordinates in enumerate(bounding_cubes_world_coordinates):

    #         bounding_cube_world_coordinates[4][2] -= config.depth_offset

    #         object_width = np.around(np.linalg.norm(bounding_cube_world_coordinates[1] - bounding_cube_world_coordinates[0]), 3)
    #         object_length = np.around(np.linalg.norm(bounding_cube_world_coordinates[2] - bounding_cube_world_coordinates[1]), 3)
    #         object_height = np.around(np.linalg.norm(bounding_cube_world_coordinates[5] - bounding_cube_world_coordinates[0]), 3)

    #         print("Position of " + segmentation_texts[i] + ":", list(np.around(bounding_cube_world_coordinates[4], 3)))

    #         print("Dimensions:")
    #         print("Width:", object_width)
    #         print("Length:", object_length)
    #         print("Height:", object_height)

    #         if object_width < object_length:
    #             print("Orientation along shorter side (width):", np.around(bounding_cubes_orientations[i][0], 3))
    #             print("Orientation along longer side (length):", np.around(bounding_cubes_orientations[i][1], 3), "\n")
    #         else:
    #             print("Orientation along shorter side (length):", np.around(bounding_cubes_orientations[i][1], 3))
    #             print("Orientation along longer side (width):", np.around(bounding_cubes_orientations[i][0], 3), "\n")

    #     self.segmentation_count += 1



    # def execute_trajectory(self, trajectory):

    #     self.logger.info(PROGRESS + "Adding trajectory points to the environment..." + ENDC)
    #     self.main_connection.send([ADD_TRAJECTORY_POINTS, trajectory])

    #     self.logger.info(PROGRESS + "Executing generated trajectory..." + ENDC)
    #     self.main_connection.send([EXECUTE_TRAJECTORY, trajectory])

    #     self.trajectory_length += len(trajectory)



    # def open_gripper(self):

    #     self.logger.info(PROGRESS + "Opening gripper..." + ENDC)
    #     self.main_connection.send([OPEN_GRIPPER])



    # def close_gripper(self):

    #     self.logger.info(PROGRESS + "Closing gripper..." + ENDC)
    #     self.main_connection.send([CLOSE_GRIPPER])



    # def task_completed(self):

    #     if self.attempted_task:

    #         self.completed_task = True

    #     else:

    #         self.logger.info(PROGRESS + "Waiting to execute all generated trajectories..." + ENDC)
    #         self.main_connection.send([TASK_COMPLETED])
    #         [env_connection_message] = self.main_connection.recv()
    #         self.logger.info(env_connection_message)

    #         self.logger.info(PROGRESS + "Generating XMem output..." + ENDC)
    #         masks = models.get_xmem_output(self.xmem_model, self.device, self.trajectory_length)
    #         self.logger.info(OK + "Finished generating XMem output!" + ENDC)

    #         num_objects = len(np.unique(masks[0])) - 1

    #         new_prompt = SUCCESS_DETECTION_PROMPT.replace("[INSERT TASK]", self.command)
    #         new_prompt += "\n"

    #         self.logger.info(PROGRESS + "Calculating object bounding cubes..." + ENDC)

    #         for object in range(1, num_objects + 1):

    #             object_positions = []
    #             object_orientations = []

    #             idx_offset = 0

    #             for i, mask in enumerate(masks):

    #                 rgb_image = Image.open(config.rgb_image_trajectory_path.format(step=i * config.xmem_output_every)).convert("RGB")
    #                 depth_image = Image.open(config.depth_image_trajectory_path.format(step=i * config.xmem_output_every)).convert("L")
    #                 depth_array = np.array(depth_image) / 255.

    #                 object_mask = mask.copy()
    #                 object_mask[object_mask != object] = False
    #                 object_mask[object_mask == object] = True
    #                 object_mask = torch.Tensor(object_mask)

    #                 bounding_cubes, orientations = utils.get_bounding_cube_from_point_cloud(rgb_image, [object_mask], depth_array, self.head_camera_position, self.head_camera_orientation_q, object - 1)

    #                 if len(bounding_cubes) == 0:

    #                     self.logger.info("No bounding cube found: removed.")
    #                     idx_offset += 1

    #                 else:

    #                     [bounding_cube] = bounding_cubes
    #                     [orientation] = orientations
    #                     position = bounding_cube[4]
    #                     orientation = orientation[0]
    #                     orientation = np.mod(orientation + math.pi, 2 * math.pi) - math.pi

    #                     object_positions.append(position)

    #                     if i == 0:

    #                         object_orientations.append(orientation)

    #                     else:

    #                         previous_orientation = object_orientations[i - 1 - idx_offset]
    #                         possible_orientations = np.array([np.mod(orientation + i * math.pi / 2 + math.pi, 2 * math.pi) - math.pi for i in range(4)])
    #                         circular_difference = np.minimum(np.abs(possible_orientations - previous_orientation), 2 * math.pi - np.abs(possible_orientations - previous_orientation))
    #                         min_index = np.argmin(circular_difference)
    #                         orientation = possible_orientations[min_index]
    #                         object_orientations.append(orientation)

    #             new_prompt += self.segmentation_texts[object - 1] + " trajectory positions and orientations:\n"
    #             new_prompt += "Positions:\n"
    #             new_prompt += str(np.around([position for p, position in enumerate(object_positions) if p % config.xmem_lm_input_every == 0], 3)) + "\n"
    #             new_prompt += "Orientations:\n"
    #             new_prompt += str(np.around([orientation for o, orientation in enumerate(object_orientations) if o % config.xmem_lm_input_every == 0], 3)) + "\n"
    #             new_prompt += "\n"

    #         self.logger.info(OK + "Finished calculating object bounding cubes!" + ENDC)

    #         self.attempted_task = True

    #         messages = []

    #         self.logger.info(PROGRESS + "Generating ChatGPT output..." + ENDC)
    #         messages = models.get_chatgpt_output(self.args.language_model, new_prompt, messages, "system", file=sys.stderr)
    #         self.logger.info(OK + "Finished generating ChatGPT output!" + ENDC)

    #         code_block = messages[-1]["content"].split("```python")

    #         task_completed = self.task_completed
    #         task_failed = self.task_failed

    #         for block in code_block:
    #             if len(block.split("```")) > 1:
    #                 code = block.split("```")[0]
    #                 exec(code)



    # def task_failed(self):

    #     self.failed_task = True

    #     self.logger.info(PROGRESS + "Resetting environment..." + ENDC)
    #     self.main_connection.send([RESET_ENVIRONMENT])
    #     [env_connection_message] = self.main_connection.recv()
    #     self.logger.info(env_connection_message)

    #     self.segmentation_count = 0
    #     self.trajectory_length = 0
    #     self.segmentation_texts = []
    #     self.attempted_task = False



    def check_points_in_semantic_map(self, object_idx, target_value, hight, extend_distance=0.2, include_midpoints=True):
        """Checks if given points exist in a specific region of the semantic map."""
        center = self.object_dimention[object_idx]['center']
        extent = self.object_dimention[object_idx]['extent'][[0, 1]]
        rotation_angle = self.object_dimention[object_idx]['orientation'][0]
        # print(center, extent, rotation_angle)

        #center_target = self.object_dimention[target_value]['center']

        bbox_vertices, extended_vertices, extended_midpoints = self.generate_extended_points(
            center, extent, rotation_angle, extend_distance, include_midpoints
        )
        # Combine extended_vertices and extended_midpoints
        all_points = np.vstack((extended_midpoints)) #all_points = np.vstack((extended_vertices, extended_midpoints))
        
        valid_points = []
        
        for i in range(all_points.shape[0]):
            point = all_points[i]
            indices = np.floor((point- self.map_origin) / self.resolution).astype(int)
            row, col = indices
            
            if 0 <= row < self.semantic_map.shape[0] and 0 <= col < self.semantic_map.shape[1]:
                if self.semantic_map[row, col] == target_value:
                    # keep 2 decimal points
                    point = np.round(point, 1)
                    point = (point[0], np.round(hight, 1), point[1])
                    valid_points.append(point) # Store the global coordinate
        
        return valid_points
    
    def backward_projection_to_rgb(self):
        if len(self.road_pcd_list) >0:
            road_pcd_homogeneous = np.hstack((self.road_pcd_list, np.ones((len(self.road_pcd_list), 1)))).T
            #object_pcd_homogeneous = np.hstack((object_pcd_list, np.ones((len(object_pcd_list), 1)))).T
        
            trans_pose_inv  = self.invert_transformation_matrix(self.pose)
            road_pcd_homo_inv = trans_pose_inv @ road_pcd_homogeneous
            #object_pcd_homo_inv = trans_pose_inv @ object_pcd_homogeneous
        
            # road_lidar_frame = road_pcd_homo_inv[:3, :].T
            # object_lidar_frame = object_pcd_homo_inv[:3, :].T

            road_cam = self.cfg['P_rect_20'].dot(self.cfg['T_cam2_velo']).dot(road_pcd_homo_inv) 
            #object_cam = calib['P_rect_20'].dot(calib['T_cam2_velo']).dot(object_pcd_homo_inv)
            road_cam[:2, :] /= road_cam[2, :]  
            #object_cam[:2, :] /= object_cam[2, :]  
            u,v,z  = road_cam
            pixels = np.dstack((v,u)).squeeze()
        else:
            pixels = np.array([])
        return pixels

    def invert_transformation_matrix(self, matrix):
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

    def rotate_point(self, point, center, angle):
        """Rotates a point around an origin by a given angle in degrees."""
        angle_radians = np.radians(angle)
        cos_theta, sin_theta = np.cos(angle_radians), np.sin(angle_radians)
        x, y = point
        cx, cz, cy = center
        x_rot = cx + (x - cx) * cos_theta - (y - cy) * sin_theta
        y_rot = cy + (x - cx) * sin_theta + (y - cy) * cos_theta
        return x_rot, y_rot

    def get_bounding_box_vertices(self, center, extent, angle=0):
        """Gets the four vertices of a bounding box, with optional rotation."""
        cx, cz, cy = center
        w, h = extent
        corners = [
            (cx - w/2, cy - h/2), (cx + w/2, cy - h/2),
            (cx + w/2, cy + h/2), (cx - w/2, cy + h/2)
        ]
        if angle != 0:
            corners = [self.rotate_point(p, center, angle) for p in corners]
        return np.array(corners)

    def get_midpoints(self, vertices):
        """Calculates the midpoints of bounding box edges."""
        midpoints = []
        for i in range(len(vertices)):
            next_i = (i + 1) % len(vertices)
            midpoints.append(((vertices[i][0] + vertices[next_i][0]) / 2,
                            (vertices[i][1] + vertices[next_i][1]) / 2))
        return np.array(midpoints)

    def extend_points_from_center(self, center, vertices, extend_distance):
        """Calculates extended points from the bounding box center to its vertices."""
        cx, cz, cy = center
        extended_vertices = []
        for vx, vy in vertices:
            direction = np.array([vx - cx, vy - cy])
            direction = direction / np.linalg.norm(direction)  # Normalize
            extended_point = np.array([vx, vy]) + extend_distance * direction
            extended_vertices.append(extended_point)
        return np.array(extended_vertices)

    def generate_extended_points(self, center, extent, angle, extend_distance, include_midpoints=True):
        """Generates extended points for bounding box vertices and optionally midpoints."""
        bbox_vertices = self.get_bounding_box_vertices(center, extent, angle)
        extended_vertices = self.extend_points_from_center(center, bbox_vertices, extend_distance)
        
        extended_midpoints = []
        if include_midpoints:
            midpoints = self.get_midpoints(bbox_vertices)
            extended_midpoints = self.extend_points_from_center(center, midpoints, extend_distance)
        
        return bbox_vertices, extended_vertices, extended_midpoints

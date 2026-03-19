import sys
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt
import torch
from pathlib import Path
from typing import List, Dict, Optional, Any
from matplotlib.colors import ListedColormap, BoundaryNorm, to_rgba
from matplotlib.ticker import MultipleLocator, AutoMinorLocator
import matplotlib.colors as mcolors
import copy
import string
from scipy.ndimage import binary_closing, binary_fill_holes, binary_dilation, center_of_mass
from scipy.spatial.distance import cdist
from matplotlib.patches import FancyArrow
import json


def visualize_map(map, map_origin, resolution, idx, robot_state, map_type="occ", sematic_info= None, zoom =False, drivable_indices=None, interval = 20): # sematic
    """
    Visualize the 2D occupancy map on the Z-X plane.

    Args:
        occupancy_map (np.ndarray): The 2D occupancy map.
        map_origin (np.ndarray): Origin of the map in world coordinates.
        resolution (float): Resolution of the map.
    """

    def get_semantic_indices(semantic_info):
        indices = []
        for info in semantic_info:
            indices.append(info["img_bbox"])
        return sorted(indices)
    
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
    
    def is_overlapping(new_point, existing_points, min_distance=10):
        """
        Check if the new_point is too close to any of the existing points.
        If so, return True; otherwise, return False.
        """
        if len(existing_points) == 0:
            return False
        distances = cdist([new_point], existing_points)
        return np.any(distances < min_distance)

    if map_type == "occ":
        # Define the colors for each value
        colors = ['gray', 'white', 'black']  # -1: gray (unknown), 0: white (free), 1: black (occupied)
        cmap = ListedColormap(colors)
        bounds = [-1.5, -0.5, 0.5, 1.5]  # Boundaries for values (-1, 0, 1)
        norm = BoundaryNorm(bounds, cmap.N)

        # Create grid axes
        z_extent = map_origin[0] + np.array([0, map.shape[0]]) * resolution
        x_extent = map_origin[1] + np.array([0, map.shape[1]]) * resolution

        # Plot the occupancy map
        fig = plt.figure(figsize=(10, 6))
        plt.imshow(
            map.T,
            origin="lower",
            #extent=[z_extent[0], z_extent[1], x_extent[0], x_extent[1]],
            cmap=cmap,
            norm=norm
        )
        # cbar = plt.colorbar(ticks=[-1, 0, 1], label="Occupancy")  # Tick labels for legend
        # cbar.set_label("Object Index", fontsize=10)  # Set the colorbar label
        cbar = fig.colorbar(plt.cm.ScalarMappable(norm=norm, cmap=cmap),
                    ax=plt.gca(), orientation='horizontal', pad=0.08, ticks=[-1, 0, 1])

        cbar.ax.tick_params(labelsize=10)  # Set the colorbar tick label size
        plt.title("2D Occupancy Map")
        plt.xlabel("X-axis")
        plt.ylabel("Z-axis")
        
        plt.grid(True, linestyle="--", alpha=0.5)
        # plt.show()
    if map_type == "sematic":
        # Plot with colorbar
        # Generate a large colormap with distinct colors
        # Create grid axes
        z_extent = map_origin[0] + np.array([0, map.shape[0]]) * resolution
        x_extent = map_origin[1] + np.array([0, map.shape[1]]) * resolution


        # road_index = find_semantic_index(sematic_info, "driveable area")
        num_classes = len(sematic_info)         
        semantic_indices = get_semantic_indices(sematic_info)
        semantic_indices = [-1] + semantic_indices
        base_colormap = plt.cm.get_cmap("tab20c", num_classes-1)  # Use "tab20c" or "viridis", etc.
        
        #custom_colors = base_colormap(np.linspace(0, 1, num_classes))  # Extract discrete colors
        custom_colors = base_colormap(np.linspace(0, 1, len(semantic_indices) - 1))

        # Add a specific color for -1 (e.g., gray for "Unknown Area")
        custom_colors = np.vstack(([1.0, 1.0, 1.0, 1], custom_colors))  # Add gray at the start
        # TODO: Delete the for loop if do not want to highlight the drivable area

        road_color  = [(0.75, 0.85, 0.85, 1), (0.85, 0.85, 0.85, 1), (0.95, 0.85, 0.85, 1), (0.85, 0.75, 0.85, 1), (0.85, 0.95, 0.85, 1), (0.85, 0.85, 0.95, 1)]
        for idx, road_index in enumerate(drivable_indices):
            # if idx large than the number of road color, staart from the beginning
            # if idx >= len(road_color):
            idx = idx % len(road_color)
            #print(f"drivable_indices: {drivable_indices}; Road index {road_index}; idx: {idx}")
        # for road_index in drivable_indices:
            custom_colors[semantic_indices.index(road_index)] = to_rgba(road_color[idx])  # Change the color of the drivable area
        
        # custom_colors[road_index] = to_rgba('orange')  
        cmap = mcolors.ListedColormap(custom_colors)
        #print(cmap.N)

        # boundaries = np.arange(num_classes + 1) - 0.5
        #boundaries = np.array(semantic_indices + [semantic_indices[-1] + 1]) #+ 0.5
        boundaries = np.array(semantic_indices + [max(semantic_indices) + 1])# - 0.5
        
        norm = mcolors.BoundaryNorm(boundaries, cmap.N)

        # colors = generate_random_colors(len(semantic_indices))
        # cmap = ListedColormap(colors)
        # boundaries = np.asarray(semantic_indices + [max(semantic_indices) + 1]) - 0.5
        # norm = mcolors.BoundaryNorm(boundaries, len(semantic_indices))
        map_x_min, map_x_max = 0, map.shape[0]
        map_z_min, map_z_max = 0, map.shape[1]
        x_min, x_max = max(-5, map_x_min), min(320, map_x_max)
        z_min, z_max = max(-5, map_z_min), min(320, map_z_max)
        
        # Plot with colorbar
        fig, ax = plt.subplots(figsize=(10, 6))

        # Plot the semantic map
        im = ax.imshow(map.T, 
                       origin="lower", 
                       #extent=[z_extent[0], z_extent[1], x_extent[0], x_extent[1]],
                       cmap=cmap, 
                       norm=norm)
        

        # Add a colorbar with custom labels
        # cbar = plt.colorbar(im, ax=ax, ticks=np.asarray(semantic_indices))
        cbar = fig.colorbar(im, ax=ax, orientation='horizontal', pad=0.08, ticks=np.asarray(semantic_indices))
        cbar.ax.tick_params(labelsize=10)

        sorted_sematic_info = sorted(sematic_info, key=lambda x: int(x['img_bbox']))

        road_grid_points = np.array([])
        # sampled_points = get_contour_sampling_points(occ, road_index)
        for free_index in drivable_indices:
            grid_points = get_semantic_area_grid_points(map, free_index, interval)
            road_grid_points = np.vstack((road_grid_points, grid_points)) if road_grid_points.size else grid_points
        # print("Grid points", len(road_grid_points))

        dot_flag = False
        if dot_flag:
            marks = 'dot'  # 'dot' or 'pad'
            labels_string =list(string.ascii_lowercase)
            for i, point in enumerate(road_grid_points):
                # label_x = (point[0]-interval)//(interval*2)
                # label_y = point[1]//(interval*2)
                label_x = point[0]//interval
                label_y = point[1]//interval
                if marks =='dot':
                    plt.text(point[0], point[1]-3, f'{labels_string[label_x]+labels_string[label_y]}', fontsize=10, weight="bold", color='red', ha='center', va='top')
                else:
                    plt.text(point[0], point[1], f'{labels_string[label_x]+labels_string[label_y]}',
                fontsize=10, color='white', ha='center', va='center',
                bbox=dict(facecolor='black', edgecolor='none', boxstyle='round,pad=0.15'))
            if marks == 'dot':
                plt.scatter(road_grid_points[:, 0], road_grid_points[:, 1], color='red', s=6, marker='o')      
        used_positions = []
        for label in sorted_sematic_info:  # Loop through each object class
            object_mask = (map == label["img_bbox"])  # Create a mask for the current label
            if np.any(object_mask):  # If the object exists in the map
                # Compute the centroid of the object
                centroid = center_of_mass(object_mask)
                x= centroid[0]
                y = centroid[1]
                if is_overlapping(centroid, used_positions, min_distance=8):
                    x += 8  # Offset x-coordinate
                    y += 8  # Offset y-coordinate
                # Add the numerical label at the centroid
                used_positions.append([x,y])
                if zoom:
                    if centroid[0] < 300 and centroid[1] < 300:
                        ax.text(
                            x,  # X-coordinate (column)
                            y,  # Y-coordinate (row)
                            f"{label['img_bbox']}",  # Label text
                            color="black", ha="center", va="center", fontsize=8, weight="bold"
                        )
                        plt.xlim(x_min, x_max)
                        plt.ylim(z_min, z_max)
                        
                else:
                    ax.text(
                            x,  # X-coordinate (column)
                            y,  # Y-coordinate (row)
                            f"{label['img_bbox']}",  # Label text
                            color="black", ha="center", va="center", fontsize=8, weight="bold"
                        )
        sorted_sematic_info = [{"cap": "Unknown Area", "img_bbox": -1}] + sorted(sematic_info, key=lambda x: int(x['img_bbox']))
        # Set custom labels matching colors
        # labels = [f"{info['img_bbox']} - {info['cap']}" for info in sorted_sematic_info]
        labels = [f"{info['img_bbox']}" for info in sorted_sematic_info]
        
        # cbar.ax.set_yticklabels(labels)  # Set the custom labels
        cbar.set_label("Object Index", fontsize=10)  # Set the colorbar label
        cbar.ax.tick_params(labelsize=10)  # Set the colorbar tick label size

 
        #plt.scatter(sampled_points[:, 1], sampled_points[:, 0], color='blue', label='Key Points', s=25, marker='.')
        # Add labels and title
        plt.title("2D Semantic Map")
        plt.xlabel(f"X-axis\n drivable area: {drivable_indices}")
        plt.ylabel("Z-axis")

        plt.gca().xaxis.set_major_locator(MultipleLocator(50)) # main locator
        plt.gca().yaxis.set_major_locator(MultipleLocator(100))
        plt.gca().xaxis.set_minor_locator(AutoMinorLocator(5)) # minor locator
        plt.gca().yaxis.set_minor_locator(AutoMinorLocator(10))

        plt.gca().xaxis.set_tick_params(which='minor', labelbottom=False)
        plt.gca().yaxis.set_tick_params(which='minor', labelleft=False)

        # plt.xticks(np.arange(0, map.shape[0], 5))
        # plt.yticks(np.arange(0, map.shape[1], 10))
        plt.grid(True, which='major', linestyle="--", alpha=0.5)
        plt.grid(True, which='minor', linestyle="--", alpha=0.25)
    
        arrow_length = 8
        circle_radius = 10
        robot_indices = robot_state["robot_indices"]
        robot_theta = robot_state["robot_state"][1]
        # dx = arrow_length * np.cos(robot_theta[0])
        # dz = arrow_length * np.sin(robot_theta[0])
        dx = arrow_length * np.cos(robot_theta)
        dz = arrow_length * np.sin(robot_theta)

        # Plot the robot position and heading
        plot_robot = True
        if plot_robot:
        # plt.Circle((robot_indices[0], robot_indices[1]), circle_radius, color='red', fill='black', linewidth = 2, label='Robot Position')
            plt.plot(robot_indices[0], robot_indices[1], 'o', color='red', markersize=circle_radius, label='Robot Position', zorder=1)
            offset_x = 0.5  # Adjust the horizontal offset
            offset_y = 0.5  # Adjust the vertical offset
            plt.text(
                robot_indices[0]+5,  # X-coordinate with offset
                robot_indices[1]+5,  # Y-coordinate with offset
                'Robot',  # The text label
                fontsize=10, weight = "bold", color='black', ha='left', va='bottom'
            )
            #plt.arrow(robot_indices[0], robot_indices[1], dx, dz, head_width=4, head_length=4, fc='black', ec='black', label='_nolegend_', linewidth=2, zorder=2)
        # Create custom arrow for legend
        arrow_legend = FancyArrow(0, 0, 0.5, 0, width=0.2, length_includes_head=True, head_width=0.4, head_length=0.3, color='black')

        # Add the legend with custom arrow
        # plt.legend(
        #     handles=[
        #         plt.Line2D([0], [0], marker='*', color='w', markerfacecolor='black', markersize=10, label='Robot Position'),
        #         arrow_legend
        #     ],
        #     labels=['Robot Position', 'Robot Heading'],
        #     loc='lower left',
        #     fontsize=8
        # )
        #plt.legend(loc='lower left', fontsize=6, handler_map={robot_plot: HandlerLine2D(marker_pad=0.3, numpoints=1, markersize=5)})
        
        # plt.xlim(-5, 300)
        # plt.ylim(-5, 300)
    if map_type == "terrain":
        fig, ax = plt.subplots(figsize=(10, 6))
        im = ax.imshow(map.T, cmap='terrain', origin='lower')
        plt.colorbar(im, label='Height Variation')
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.title("2D Terrain Map")
        plt.show()
    # plt.xlim(left=-5)
    # plt.ylim(bottom=-5)
    #plt.tight_layout()
    
        # output_path = f"semantic_map_{idx}.png"  # Change the filename and format as needed
        # plt.savefig(output_path, format='png', dpi=300, bbox_inches='tight')
        # plt.show()
    return fig


def update_flag(path=None, trigger=False):
    """
    Resets the 'trigger' flag to False in update.txt.
    """
    if os.path.exists(path):
        with open(path, "w") as f:
            json.dump({"trigger": trigger}, f)


def read_update_flag(path=None):
    """
    Reads the 'trigger' flag from update.txt.
    Returns True if triggered, False otherwise.
    """
    if not os.path.exists(path):
        print("⚠️ update.txt not found.")
        return False

    try:
        with open(path, "r") as f:
            data = json.load(f)
        return data.get("trigger", False)
    except Exception as e:
        print(f"❌ Error reading update.txt: {e}")
        return False

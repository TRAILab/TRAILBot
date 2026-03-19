import os
import re
import shutil
import numpy as np
import open3d as o3d

set_number = 1

base_dir = '/home/trailbot/action4/20260227/outdoor_standard'

select_pc_dir = os.path.join(base_dir, 'selected_pc')
select_camera_dir = os.path.join(base_dir, 'selected_camera')
match_file = os.path.join(base_dir, 'match.txt')

pcd_out_dir = os.path.join(base_dir, 'pcd')
cam_out_dir = os.path.join(base_dir, 'extrinsic_camera')

os.makedirs(pcd_out_dir, exist_ok=True)
os.makedirs(cam_out_dir, exist_ok=True)

# ---------- 1. parse match.txt (lidar -> camera) ----------
lidar_to_camera = {}

with open(match_file, 'r') as f:
    for line in f:
        m = re.search(r'point cloud\s+(\d+)\s*:\s*(\d+)', line)
        if m:
            lidar_idx = int(m.group(1))
            camera_idx = int(m.group(2))
            lidar_to_camera[lidar_idx] = camera_idx

# ---------- 2. iterate selected_pc ----------
files = sorted(
    [f for f in os.listdir(select_pc_dir) if f.startswith('points_')],
    key=lambda x: int(re.search(r'points_(\d+)', x).group(1))
)

for fname in files:
    filename = os.path.join(select_pc_dir, fname)

    lidar_idx = int(re.search(r'points_(\d+)\.txt', fname).group(1))

    if lidar_idx not in lidar_to_camera:
        print(f'[WARN] No camera match for lidar {lidar_idx}, skip.')
        continue

    camera_idx = lidar_to_camera[lidar_idx]

    # ---------- read & crop lidar ----------
    points = []
    with open(filename, 'r') as f:
        f.readline()  # skip header
        for line in f:
            if not line.strip():
                continue
            if line[0] == 'T':
                continue

            x, y, z = map(float, line.strip().split())

            if (x < 5 and x > -5 and
                y < 10 and y > 0 and
                z < 1.5 and z > -1):
                points.append([x, y, z])

    if len(points) == 0:
        print(f'[WARN] No points kept in {fname}, skip.')
        continue

    # ---------- save PCD ----------
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np.asarray(points))

    pcd_path = os.path.join(pcd_out_dir, f'set_{set_number}.pcd')
    o3d.io.write_point_cloud(pcd_path, pcd)

    # ---------- save camera image ----------
    cam_src = os.path.join(
    select_camera_dir,
    f'frame_{camera_idx:04d}.jpg'
)
    cam_dst = os.path.join(cam_out_dir, f'set_{set_number}.jpg')

    if os.path.exists(cam_src):
        shutil.copy(cam_src, cam_dst)
    else:
        print(f'[WARN] Missing camera image: frame_{camera_idx}.jpg')

    print(
        f'Saved set_{set_number}: '
        f'points_{lidar_idx}.txt ↔ frame_{camera_idx}.jpg '
        f'({len(points)} pts)'
    )

    set_number += 1
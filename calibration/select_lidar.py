import os
import re
import shutil

match_file = '/home/trailbot/action4/20260227/outdoor_standard/match.txt'
select_camera_dir = '/home/trailbot/action4/20260227/outdoor_standard/top100_images'
lidar_src_dir = '/home/trailbot/action4/20260227/outdoor_standard/lidar_data/pc'
lidar_dst_dir = '/home/trailbot/action4/20260227/outdoor_standard/selected_pc'

# --- 1. collect selected camera indices ---
camera_indices = set()
for fname in os.listdir(select_camera_dir):
    m = re.search(r'(\d+)', fname)
    if m:
        camera_indices.add(int(m.group(1)))

print('Selected camera indices:')
print(sorted(camera_indices))

# --- 2. parse match.txt (lidar : camera) ---
selected_lidar_indices = []

with open(match_file, 'r') as f:
    for line in f:
        line = line.strip()
        if not line:
            continue

        m = re.search(r'point cloud\s+(\d+)\s*:\s*(\d+)', line)
        if not m:
            continue

        lidar_idx = int(m.group(1))
        camera_idx = int(m.group(2))

        if camera_idx in camera_indices:
            selected_lidar_indices.append(lidar_idx)

print('Matched lidar indices:')
print(selected_lidar_indices)

# --- 3. copy lidar txt ---
os.makedirs(lidar_dst_dir, exist_ok=True)

missing = []

for idx in selected_lidar_indices:
    src = os.path.join(lidar_src_dir, f'points_{idx}.txt')
    dst = os.path.join(lidar_dst_dir, f'points_{idx}.txt')

    if not os.path.exists(src):
        missing.append(idx)
        continue

    shutil.copy(src, dst)

print(f'Copied {len(selected_lidar_indices) - len(missing)} lidar files to {lidar_dst_dir}')

if missing:
    print('Missing lidar files:')
    print(missing)
import numpy as np
import matplotlib.pyplot as plt
import tqdm
import os
import shutil



def parse_time_from_txt(line):
    nums = []
    cur = ""
    for c in line:
        if c.isdigit():
            cur += c
        elif cur:
            nums.append(int(cur))
            cur = ""
    if cur:
        nums.append(int(cur))
    return nums[0], nums[1]  # sec, nanosec


# -------- load image timestamps --------
img_times = []
for img_idx in tqdm.tqdm(range(541, 8417), desc="Loading image timestamps"):
    imgFile = f'/home/trailbot/action4/20260227/outdoor_ultra/camera_data/output_{img_idx}.txt'
    with open(imgFile, 'r') as f:
        sec, nsec = parse_time_from_txt(f.readline())
        t_ns = sec * 1_000_000_000 + nsec
        img_times.append((img_idx, t_ns))

img_times = np.array(img_times, dtype=object)
img_t = np.array(img_times[:, 1], dtype=np.int64)


# -------- nearest-neighbor matching --------
nearest_matches = []   # (pt_idx, img_idx, dt_ms)

for pt_idx in tqdm.tqdm(range(1, 5631), desc="Matching lidar to nearest image"):
    ptFile = f'/home/trailbot/action4/20260227/outdoor_ultra/lidar_data/points_{pt_idx}.txt'
    with open(ptFile, 'r') as f:
        line = f.readline()
        line = line.replace(
            "Timestamp: builtin_interfaces.msg.Time(", ""
        ).replace(")", "")
        parts = line.split(", ")
        sec = int(parts[0].split("=")[1])
        nsec = int(parts[1].split("=")[1])
        t_pc = sec * 1_000_000_000 + nsec

    # nearest neighbor in time
    idx = np.argmin(np.abs(img_t - t_pc))
    dt_ms = (img_t[idx] - t_pc) / 1e6
    img_idx = img_times[idx][0]

    nearest_matches.append((pt_idx, img_idx, dt_ms))


# -------- analyze nearest neighbors --------
dt_all = np.array([m[2] for m in nearest_matches], dtype=float)

# -------- take TOP-K most nearest --------
nearest_sorted = sorted(nearest_matches, key=lambda x: abs(x[2]))

TOP_K = 300
best_matches = nearest_sorted[:TOP_K]

dt_best = np.array([m[2] for m in best_matches], dtype=float)


txtname = "/home/trailbot/action4/20260227/outdoor_ultra/match.txt"
best_matches_sorted = sorted(best_matches, key=lambda x: x[0])

with open(txtname, "w") as output_file:
    for pt_idx, img_idx, dt_ms in best_matches_sorted:
        output_file.write(f"point cloud {pt_idx}: {img_idx}\n")

print(f"match.txt written to {txtname}")


dst_dir = "/home/trailbot/action4/20260227/outdoor_ultra/top300_images"
os.makedirs(dst_dir, exist_ok=True)
src_dir = "/home/trailbot/action4/20260227/outdoor_ultra/camera_data"
img_indices = [m[1] for m in best_matches]

print(f"Copying {len(img_indices)} images...")

for img_idx in img_indices:
    src_file = os.path.join(src_dir, f"frame_{img_idx:04d}.jpg")
    dst_file = os.path.join(dst_dir, f"frame_{img_idx:04d}.jpg")

    if not os.path.exists(src_file):
        print(f"[WARN] missing file: {src_file}")
        continue

    shutil.copy2(src_file, dst_file)

print("Done.")
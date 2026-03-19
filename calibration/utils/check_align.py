import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from mpl_toolkits.mplot3d import Axes3D
import glob, os

IMG_DIR = "/home/trailbot/action4/20260227/outdoor_ultra/extrinsic_camera"
PCD_DIR = "/home/trailbot/action4/20260227/outdoor_ultra/pcd"

import re
def natural_sort_key(s):
    return [int(t) if t.isdigit() else t for t in re.split(r'(\d+)', s)]

img_files = sorted(glob.glob(os.path.join(IMG_DIR, "*.jpg")), key=natural_sort_key)
pcd_files = sorted(glob.glob(os.path.join(PCD_DIR, "*.pcd")), key=natural_sort_key)
n_frames   = min(len(img_files), len(pcd_files))
print(f"Total frames: {n_frames}")

# ── 预扫所有帧，计算全局 xyz 范围 ────────────────────────────────
print("Scanning all frames for global axis limits...")
all_min = np.array([np.inf, np.inf, np.inf])
all_max = np.array([-np.inf, -np.inf, -np.inf])
for f in pcd_files[:n_frames]:
    pts = np.asarray(o3d.io.read_point_cloud(f).points)
    if len(pts) > 0:
        all_min = np.minimum(all_min, pts.min(axis=0))
        all_max = np.maximum(all_max, pts.max(axis=0))
print(f"  X: [{all_min[0]:.2f}, {all_max[0]:.2f}]  "
      f"Y: [{all_min[1]:.2f}, {all_max[1]:.2f}]  "
      f"Z: [{all_min[2]:.2f}, {all_max[2]:.2f}]")

idx = [0]   # mutable so callbacks can modify it

fig = plt.figure(figsize=(14, 6))
ax_img = fig.add_subplot(1, 2, 1)
ax_pcd = fig.add_subplot(1, 2, 2, projection='3d')

def draw(i):
    # 保存当前视角
    elev = ax_pcd.elev
    azim = ax_pcd.azim
    ax_img.cla()
    ax_pcd.cla()

    # ── 图像 ──────────────────────────────────────────────────────
    img = mpimg.imread(img_files[i])
    ax_img.imshow(img)
    ax_img.set_title(os.path.basename(img_files[i]), fontsize=9)
    ax_img.axis('off')

    # ── 点云 ──────────────────────────────────────────────────────
    pcd = o3d.io.read_point_cloud(pcd_files[i])
    pts = np.asarray(pcd.points)

    if len(pts) > 0:
        # 降采样加速渲染（最多 20000 点）
        if len(pts) > 20000:
            chosen = np.random.choice(len(pts), 20000, replace=False)
            pts = pts[chosen]

        dist = np.linalg.norm(pts, axis=1)
        norm = (dist - dist.min()) / (dist.max() - dist.min() + 1e-6)

        ax_pcd.scatter(
            pts[:, 0], pts[:, 1], pts[:, 2],
            c=norm, cmap='jet', s=0.5, linewidths=0
        )

    ax_pcd.view_init(elev=elev, azim=azim)
    ax_pcd.set_xlim(all_min[0], all_max[0])
    ax_pcd.set_ylim(all_min[1], all_max[1])
    ax_pcd.set_zlim(all_min[2], all_max[2])
    ax_pcd.set_xlabel('X'); ax_pcd.set_ylabel('Y'); ax_pcd.set_zlabel('Z')
    ax_pcd.set_title(f"Point Cloud  frame {i+1}/{n_frames}", fontsize=9)

    fig.suptitle("← → 切换帧   q 退出", fontsize=10, y=0.02)
    fig.canvas.draw_idle()

def on_key(event):
    if event.key == 'right':
        idx[0] = (idx[0] + 1) % n_frames
        draw(idx[0])
    elif event.key == 'left':
        idx[0] = (idx[0] - 1) % n_frames
        draw(idx[0])
    elif event.key == 'q':
        plt.close()

fig.canvas.mpl_connect('key_press_event', on_key)
draw(0)
plt.tight_layout()
plt.show()
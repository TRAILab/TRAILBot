import os
import numpy as np
import matplotlib.pyplot as plt

lidar_dir = '/home/trailbot/action4/20260227/outdoor_standard/selected_pc'
files = sorted([f for f in os.listdir(lidar_dir) if f.startswith('points_')])

for fname in files:
    points = np.loadtxt(os.path.join(lidar_dir, fname), skiprows=1)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(points[:,0], points[:,1], points[:,2],
               s=2, c=points[:,2], cmap='viridis')

    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim(-5, 5)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_box_aspect([1,1,1])
    ax.set_title(fname)

    plt.show()
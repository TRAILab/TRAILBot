import numpy as np
import open3d as o3d

# change set_number value so that it matches with your set number
set_number = 1
# in square bracket enter the indices that are selected for extrinsic calibration
for i in []:
    # change the filelocation here
    filename = f'realCali/calif/points/points_{i}.txt'
    points = []
    with open(filename, 'r') as f:
        for line in f.readlines():
            if line[0] == 'T':
                continue
            x, y, z = map(float, line.strip().split())
            # change the min max value for xy here
            if x < 6 and x > 1 and y > -2 and y < 2:
                points.append([x, y, z])
    
    npArrayPoints = np.asarray(points)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    o3d.io.write_point_cloud(f"set_{set_number}.pcd", pcd)
    set_number = set_number + 1
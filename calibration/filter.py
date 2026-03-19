import numpy as np
import open3d as o3d

# change set_number value so that it matches with your set number
set_number = 1
# in square bracket enter the indices that are selected for extrinsic calibration
# for i in [39, 54, 64, 72, 85, 96, 109, 128, 145, 163, 174, 186, 206, 214, 223, 236, 252]:
for i in [383, 405, 437, 439, 441, 690, 730, 748, 750, 764, 772, 796, 1071, 1125, 1129, 1133, 1368, 1424, 1440, 1703,\
          1735, 1775, 1992, 2108,  2427, 2447, 2455, 2461, 2467, 2706, 2734, 2752, 2766, 2780, 3049, 3067, 3450, 3721, 3733,\
            3741, 4401, 4706, 4756]:
# for i in [158]:
    # change the filelocation here
    filename = f'/home/trailbot/action4/20260227/outdoor_ultra/lidar_data/pc/points_{i}.txt'
    points = []
    with open(filename, 'r') as f:
        f.readline()
        for line in f.readlines():
            if line[0] == 'T':
                continue
            x, y, z = map(float, line.strip().split())
            # change the min max value for xy here
            if  x < 4 and x > -4 and y < 4 and y > 0  and z <1.5 and z>-0.8:
                points.append([x, y, z])
    #     # if  x < 0.755489 and x > -0.507062 and y < 2.94305 and y > 2.32642 

    npArrayPoints = np.asarray(points)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    o3d.io.write_point_cloud(f"/home/trailbot/action4/20260227/outdoor_ultra//pcd/set_{set_number}.pcd", pcd)
    set_number = set_number + 1
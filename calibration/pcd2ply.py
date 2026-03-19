import open3d as o3d

# Load the PCD file
pcd = o3d.io.read_point_cloud("/workspace/trail_ws/calibration_2025/outdoor_0605/out/pcd/set_12.pcd")

# Save it as a PLY file
o3d.io.write_point_cloud("/workspace/trail_ws/calibration_2025/outdoor_0605/out/show_pcd/set_12.ply", pcd)

print("Successfully converted PCD to PLY.")

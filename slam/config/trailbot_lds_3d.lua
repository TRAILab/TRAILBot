include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "os_imu",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

--ALL mentioned original values are from the files at /opt/ros/humble/share/cartographer/configuration_files
--SOME non-documented changes were made before as this was adapted from the backpack_3d.lua file

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1 --originally 1
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10 --originally 5
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1e2 --originally 400

--These resolutions made the map a lot more clear 
-- TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05 --originally 0.1 
-- TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.2 --originally 0.45
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.1 --originally 0.1 
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.5 --originally 0.45

MAP_BUILDER.use_trajectory_builder_3d = true --originally false
-- MAP_BUILDER.num_background_threads = 4 --originally 4
MAP_BUILDER.num_background_threads = 16 --originally 4

-- POSE_GRAPH.optimization_problem.huber_scale = 5e2 --originally 100 
POSE_GRAPH.optimization_problem.huber_scale = 100
POSE_GRAPH.optimize_every_n_nodes = 50 --originally 90
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3 --originally 0.3

POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 200 --oroiginally 200

POSE_GRAPH.constraint_builder.min_score = 0.62 --originally 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66 --originally 0.6
-- POSE_GRAPH.constraint_builder.min_score = 0.2 --originally 0.55
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.2 --originally 0.6

return options

-- trailbot_lds_3d_mahan.lua  (example baseline for Ouster -> /filtered_points)

-- include "map_builder.lua"
-- include "trajectory_builder.lua"

-- options = {
--   map_builder = MAP_BUILDER,
--   trajectory_builder = TRAJECTORY_BUILDER,
--   map_frame = "map",
--   tracking_frame = "os_imu",          -- IMU frame; ensure TF base_link <-> os_imu exists
--   published_frame = "base_link",
--   odom_frame = "odom",
--   provide_odom_frame = true,          -- Cartographer provides odom (OK)
--   publish_frame_projected_to_2d = false,
--   use_odometry = false,               -- start without wheel odom; enable later if good quality
--   use_nav_sat = false,
--   use_landmarks = false,
--   num_laser_scans = 0,
--   num_multi_echo_laser_scans = 0,
--   num_subdivisions_per_laser_scan = 1,
--   num_point_clouds = 1,               -- using /filtered_points
--   lookup_transform_timeout_sec = 0.2,
--   submap_publish_period_sec = 0.3,
--   pose_publish_period_sec = 5e-3,
--   trajectory_publish_period_sec = 30e-3,
--   rangefinder_sampling_ratio = 1.,
--   odometry_sampling_ratio = 1.,
--   fixed_frame_pose_sampling_ratio = 1.,
--   imu_sampling_ratio = 1.,
--   landmarks_sampling_ratio = 1.,
-- }

-- MAP_BUILDER.use_trajectory_builder_3d = true
-- MAP_BUILDER.num_background_threads = 16

-- -- ===== 3D builder tuning (point cloud) =====
-- -- Ranges: pick for your space. Indoors often 0.5..30m. Outdoors can go larger.
-- TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
-- TRAJECTORY_BUILDER_3D.min_range = 0.5           -- discard very near points
-- TRAJECTORY_BUILDER_3D.max_range = 30.0          -- cap for indoor mapping (increase if needed)
-- TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.05  -- base downsampling (meters)
-- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 2.0
-- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 150
-- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 15.0
-- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 4.0
-- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 200
-- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 60.0

-- -- Keep only height band of interest to remove ceiling/ground clutter if needed:
-- TRAJECTORY_BUILDER_3D.min_z = -0.6              -- tune to your lidar mounting height
-- TRAJECTORY_BUILDER_3D.max_z =  0.5

-- -- Submaps: resolutions and fusion
-- TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.10
-- TRAJECTORY_BUILDER_3D.submaps.low_resolution  = 0.50
-- --TRAJECTORY_BUILDER_3D.submaps.num_range_data  = 160  -- higher -> smoother submaps, slower update

-- -- Scan matcher (your weights were fine; keep modest)
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.0
-- TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight    = 100.0
-- --TRAJECTORY_BUILDER_3D.ceres_scan_matcher.only_optimize_yaw  = false

-- -- Useful: motion filter so we don’t over-insert redundant clouds
-- TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
-- TRAJECTORY_BUILDER_3D.motion_filter.max_distance_m   = 0.1
-- TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians= 0.003

-- -- IMU tuning
-- TRAJECTORY_BUILDER_3D.imu_gravity_time_constant = 10.0

-- -- Pose graph / loop closure
-- POSE_GRAPH.optimize_every_n_nodes = 50
-- POSE_GRAPH.optimization_problem.huber_scale = 100.0
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 200
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
-- POSE_GRAPH.constraint_builder.min_score = 0.62
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

-- return options

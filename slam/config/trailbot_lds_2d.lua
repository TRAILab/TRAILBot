include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link", --originally 'odom'
  odom_frame = "odom",
  provide_odom_frame = true, --originally 'false'
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
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

MAP_BUILDER.use_trajectory_builder_2d = true

-- bringing changes over from the 3D file
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10 --originally 10
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40 --originally 40

TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 12
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.use_imu_data = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

--These resolutions made the map a lot more clear 
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.05 --normally 0.1 
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.2 --normally 0.45

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

POSE_GRAPH.optimize_every_n_nodes = 90  --originally 90, turn to 0 when wanting to only use local SLAM

return options
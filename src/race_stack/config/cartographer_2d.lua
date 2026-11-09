-- 2D mapping: /scan + gyro-corrected /odom_imu.
-- imu_odom owns odom -> base_link; Cartographer owns map -> odom.
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_pose_extrapolator = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.02,
  trajectory_publish_period_sec = 0.1,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 2
-- Raw VESC IMU is deg/s and g. Its yaw is already used by imu_odom.
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.min_range = 0.1
-- ★ 2026-11-09: 20.0 -> 5.0 (낮은 벽 너머를 보지 않게 자른다)
--
--   우리 트랙은 벽이 낮아 라이다 평면이 벽 높이를 스친다. 20m 까지 열어두면
--   벽 너머 공간(사람·의자·짐)이 그대로 스캔에 들어오는데, 그쪽은 정적이지도
--   않고 **차가 1도만 피치해도 넘어가는 빔이 통째로 바뀐다** (10m 지점에서
--   높이 17cm 이동 = 벽에 맞던 빔이 벽을 넘어간다).
--
--   2026-11-09 주행 실측이 정확히 그 모양이었다:
--       근거리(<2m) 잔차   5~11cm   ← 트랙 벽. 항상 거기 있음
--       원거리    잔차  87~97cm   ← 벽 너머. 매번 다름
--   위치는 가까운 벽이 잡아주는데 **자세가 먼 쓰레기 데이터에 끌려갔다.**
--   원거리 점은 지렛대가 길어서, 10m 점이 1m 어긋나면 매처가 자세를 5.7도
--   돌려버린다. map_controller 가 "AMCL 보정을 제한" 경고를 53회 냈다.
--
--   스캔 매칭은 파티클 필터보다 이 문제에 약하다 — 가설이 하나뿐이라 틀린
--   정합으로 그대로 끌려간다. AMCL 은 laser_likelihood_max_dist 2.0 으로
--   사실상 근거리만 쓰고, slam_toolbox 도 12m 다. Cartographer 만 20m 로
--   열려 있었다.
--
--   값 근거: 1109-2_clean 트랙은 원점 기준 최대 반경 4.7m 다 (x -3.15~+4.45,
--   y -0.88~+3.77). 5m 면 트랙 벽은 전부 쓰고 벽 너머는 전부 버린다.
--   ★ 트랙이 더 커지면 이 값도 같이 키울 것. 트랙보다 작으면 벽을 못 본다.
--
--   ⚠ 이 값을 바꾸면 **맵을 다시 따야 한다.** 맵핑 때와 주행 때 범위가
--     다르면 서로 다른 스캔을 비교하게 된다. cartographer_driving.lua 가
--     이 파일을 include 하므로 맵핑·주행 양쪽에 같이 적용된다.
TRAJECTORY_BUILDER_2D.max_range = 5.
-- max_range 와 맞춘다 (빔이 아무것도 못 맞췄을 때 '여기까지는 비어 있다' 로
-- 기록하는 길이. max_range 보다 길면 자른 구간을 다시 자유공간으로 칠한다)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
-- Baseline for low-speed mapping; measure before increasing search cost.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false
POSE_GRAPH.optimize_every_n_nodes = 90

return options

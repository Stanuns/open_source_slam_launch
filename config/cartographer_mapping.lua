-- cartographer_mapping.lua  from revo_lds.lua
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",    --地图坐标系名字 
  tracking_frame = "imu_link",  -- 将所有传感器数据转换到这个坐标系下，如果有imu的，就写imu_link，如果没有，就写base_link或者base_footprint. 
                                -- cartographer会将所有传感器进行坐标变换到tracking_frame坐标系下，每个传感器的频率不一样，imu频率远高于laser的频率，这样做可以减少计算
  published_frame = "base_footprint",  --cartographer发布发布map到published_frame之间的tf;
  odom_frame = "odom", --里程计坐标系名字
  provide_odom_frame = true,-- 是否提供odom的tf, 如果为true,则tf树为map->odom->base_footprint(即published_frame)
                             -- 如果为false tf树为map->base_footprint(即published_frame)
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true, --发布tf时是使用pose_extrapolator的位姿还是前端计算出来的位姿，前端计算的位姿更准
  use_odometry = true, --是否使用odom数据，如果使用要求一定要有odom的tf, 使用odom数据时需要看在打滑情况下odom数据是否可信
  use_nav_sat = false, -- 是否使用gps topic形式订阅，不可订阅多个里程计/gps/landmark，要注意做重映射
  use_landmarks = false,

  num_laser_scans = 1,  --订阅的lidar scan数量
  num_multi_echo_laser_scans = 0,

  num_subdivisions_per_laser_scan = 1,  --扫描分割数量，为1不分割
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

MAP_BUILDER.use_trajectory_builder_2d = true --在这里进行重写会覆盖子文件中的参数

-- 根据激光雷达的性能，最小范围
TRAJECTORY_BUILDER_2D.min_range = 0.5
-- 根据激光雷达的性能，最大范围
TRAJECTORY_BUILDER_2D.max_range = 30
-- 无效激光数据设置距离为该数值
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- false,不使用IMU数据
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- true,使用实时回环检测来进行前端的扫描匹配
-- false, 关闭该功能后，如果use_imu_data=false， 原地打滑旋转会造成严重的方向角的错误，建图失败，即没有使用lidar scan数据进行方向角判断
-- 理论上该功能也能够优化位移打滑的情况（todo）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
--尽量小点；如果移动距离或旋转过小, 或者时间过短, 不进行地图的更新
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.2

--TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1

--FastCorrelativeScanMatcher初步匹配的结果分数，高于此分数才进入下一步的Ceres Scan Matcher处理。
POSE_GRAPH.constraint_builder.min_score = 0.65
--全局定位最小分数，低于此分数则认为目前全局定位不准确
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 关闭后端优化 注释之后打开后端优化
-- 由于后端优化会用到odometry数据，如果不关闭后端优化，在原地长时间快速旋转时，会导致建图重影或失败
-- POSE_GRAPH.optimize_every_n_nodes = 0
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0
-- POSE_GRAPH.global_sampling_ratio = 0

return options 

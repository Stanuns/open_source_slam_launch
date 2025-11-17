# ***一、ros2中，Cartographer与Navigation2的安装及使用***

## 1.1 安装 cartographer、navigation2

```bashrc
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```
## 1.2 安装依赖
```bashrc
sudo apt update
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-robot-localization
sudo apt-get install python3-serial
sudo apt-get install ros-humble-tf-transformations
sudo apt install python3-pip
pip3 install nest-asyncio
sudo apt install ros-humble-nav2-map-server
sudo apt install ros-humble-nav2-lifecycle-manager
```

## 1.3 DEBUG阶段保存地图方法
  [refer 1](https://github.com/ros-navigation/navigation2/tree/humble/nav2_map_server)
  [refer 2](https://roboticsbackend.com/ros2-nav2-tutorial/)

  ```bashrc
  ros2 run nav2_map_server map_saver_cli -f my_map_name
  ```

#### issue 1: if local_costmap/costmap of rviz cannot show, you need check the use_sim_time.

#### issue2: Now, you should just see the map on the screen, but no robot. You’ll also see some kinds of error logs in the terminal. This is because Nav2 doesn’t know where your robot is, and you need to **provide the first 2D pose estimate**.

# ***二、建图与导航算法配置文件***
## 2.1 建图算法配置文件(单独建图与自动导航探索建图共用同一个配置文件)
  该包config目录下的[luxsharerobot_cartographer_mapping.lua](./config/luxsharerobot_cartographer_mapping.lua)， 配置文件参数解释见该文件中的注释

## 2.2 导航避障算法配置文件
  - 2.2.1 单独导航的配置文件在该包目录launch/luxsharerobot_nav2下
    [nav2_params_luxsharerobot.yaml](./launch/luxsharerobot_nav2/nav2_params_luxsharerobot.yaml), 配置文件参数解释见该文件中的注释

  - 2.2.2 自动导航探索建图的配置文件在该包目录launch/luxsharerobot_nav2_aem下
    [nav2_auto_explore_mapping_luxsharerobot.yaml](./launch/luxsharerobot_nav2_aem/nav2_auto_explore_mapping_luxsharerobot.yaml)


# ***三、配置RGBD相机进行导航避障***
## 3.1 首先确保RGBD相机正确启动，并发出点云数据到topic /pointcloud

## 3.2 配置RGBD相机与机器人的tf转换关系
  配置文件路径：robot_description包中diff_robot.urdf.xacro文件这一部分：
  ```bashrc
      <joint name="base_link_to_camera_link" type="fixed">
        <parent link="base_link" />
        <child link="camera_link" />
        <origin xyz="0.06 0.00 0.356" rpy="1.2391 3.14159 1.5707"/> <!-- RGBD相机下俯19° -->
      </joint>
  ```

## 3.3 在nav2_params_luxsharerobot.yaml文件中加载点云数据topic /pointcloud
  在local_costmap及global_costmap中加载
  ```bashrc
    local_costmap:
      local_costmap:
        ros__parameters:
          update_frequency: 20.0           # 更新频率20Hz（高频更新，用于实时避障）
          publish_frequency: 10.0
          global_frame: odom_combined      # 全局坐标系（里程计融合坐标系）
          robot_base_frame: base_footprint # 机器人基座坐标系
          use_sim_time: False
          rolling_window: true   # 使用滚动窗口模式（local_costmap地图随机器人移动）
          width: 3               # local_costmap地图宽度3米
          height: 3              # local_costmap地图高度3米
          resolution: 0.05       # 分辨率0.05m/像素
          robot_radius: 0.3      # 机器人半径0.3m（用于圆形机器人模型）
          # footprint: "[ [-0.03, -0.0900], [-0.03, 0.0900], [0.2, 0.0900], [0.2, -0.0900] ]"
          plugins: ["voxel_layer", "inflation_layer"] # 使用的图层
          inflation_layer:
            plugin: "nav2_costmap_2d::InflationLayer" # 膨胀图层插件
            cost_scaling_factor: 3.0 # 3.0 10.0       # 代价缩放因子3.0（控制代价衰减速度),这个值越大，安全区域越小，越靠近障碍物行驶
            inflation_radius: 0.35   # 0.25 0.5       # local_costmap膨胀半径0.35m
          voxel_layer:
            plugin: "nav2_costmap_2d::VoxelLayer" # 3D体素图层插件
            enabled: True
            publish_voxel_map: True
            origin_z: 0.0                         # 原点Z坐标，表示体素地图的Z轴原点位于地面（z=0m）高度
            z_resolution: 0.05                    # Z轴分辨率0.05m
            z_voxels: 16                          # Z轴体素数（16×0.05=0.8m高度）
            max_obstacle_height: 2.0              # 最大障碍物高度2.0m
            mark_threshold: 1                     # 标记阈值（1个体素即视为障碍物)
            observation_sources: scan pointcloud  # **使用激光雷达数据/scan与RGBD相机数据/pointcloud进行避障**
            scan:
              topic: /scan                        # 激光雷达数据topic
              max_obstacle_height: 2.0            # 最大障碍物高度
              clearing: True                      # 启用障碍物清除，设置True：当传感器检测到之前标记为障碍物的区域现在为空闲时，清除该区域的障碍物标记。
              marking: True                       # 启用障碍物标记，设置True：当传感器检测到障碍物时，在代价地图中标记相应区域。
              data_type: "LaserScan"              # 数据类型
              raytrace_max_range: 10.0 #10.0      # 光线追踪最大范围10m （调大能更好的清除远处障碍物，增加计算开销；调小减少计算量，减小远处传感器噪声影响）
              raytrace_min_range: 0.2             # 光线追踪最小范围0.2m （调大能更好的减少近处噪声，但近处障碍物清除能力下降；调小能够更精确的近处障碍物处理，增加计算开销）
              obstacle_max_range: 2.5             # 障碍物检测最大范围2.5m(距离机器人)
              obstacle_min_range: 0.05            # 障碍物检测最小范围0.05m(距离机器人)
            pointcloud:
              topic: /pointcloud                 # RGBD相机点云数据topic
              max_obstacle_height: 1.5           # 最大障碍物高度1.5m
              min_obstacle_height: 0.12          # 最小障碍物高度0.12m（过滤地面，由于点云误差设置成0.12m）
              clearing: True                     # 启用障碍物清除, 设置True：当传感器检测到之前标记为障碍物的区域现在为空闲时，清除该区域的障碍物标记。
              marking: True                      # 启用障碍物标记，设置True：当传感器检测到障碍物时，在代价地图中标记相应区域。
              data_type: "PointCloud2"           # 数据类型
              raytrace_max_range: 4.0            # 光线追踪最大范围4m  （调大能更好的清除远处障碍物，增加计算开销；调小减少计算量，减小远处传感器噪声影响）
              raytrace_min_range: 0.05           # 光线追踪最小范围0.05m (调大能更好的减少近处噪声，但近处障碍物清除能力下降；调小能够更精确的近处障碍物处理，增加计算开销）
              obstacle_max_range: 3.0            # 障碍物检测最大范围3m(距离机器人)
              obstacle_min_range: 0.01           # 障碍物检测最小范围0.01m(距离机器人)

          always_send_full_costmap: True
      local_costmap_client:
        ros__parameters:
          use_sim_time: False
      local_costmap_rclcpp_node:
        ros__parameters:
          use_sim_time: False

    global_costmap:
      global_costmap:
        ros__parameters:
          update_frequency: 5.0                   # 更新频率5Hz（低频更新）
          publish_frequency: 2.0
          global_frame: map
          robot_base_frame: base_footprint
          use_sim_time: False
          robot_radius: 0.3                       # 机器人半径0.3m（用于圆形机器人模型）
          #footprint: "[ [-0.03, -0.0900], [-0.03, 0.0900], [0.2, 0.0900], [0.2, -0.0900] ]"
          resolution: 0.05
          track_unknown_space: true               # 跟踪未知空间（为true时未知区域可通行）
          plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
          obstacle_layer:
            mark_threshold: 1
            plugin: "nav2_costmap_2d::ObstacleLayer"
            enabled: True
            observation_sources: scan pointcloud  # **使用激光雷达数据/scan与RGBD相机数据/pointcloud进行避障**
            scan:
              topic: /scan                        # 激光雷达数据topic
              max_obstacle_height: 2.0
              clearing: True
              marking: True
              data_type: "LaserScan"
              raytrace_max_range: 10.0
              raytrace_min_range: 0.2
              obstacle_max_range: 3.0
              obstacle_min_range: 0.05
              observation_persistence: 0.0      #障碍物持久性，0.0：实时性最好，但对传感器噪声更敏感；调大对短暂障碍物更鲁棒，减少障碍物闪烁，缺点是致已移动障碍物的残留
            pointcloud:
              topic: /pointcloud                # RGBD相机点云数据topic
              max_obstacle_height: 1.5          
              min_obstacle_height: 0.12
              clearing: True
              marking: True
              data_type: "PointCloud2"
              raytrace_max_range: 10.0
              raytrace_min_range: 0.1
              obstacle_max_range: 3.0
              obstacle_min_range: 0.01
              observation_persistence: 0.0     #障碍物持久性，0.0：实时性最好，但对传感器噪声更敏感；调大对短暂障碍物更鲁棒，减少障碍物闪烁，缺点是致已移动障碍物的残留
          static_layer:
            plugin: "nav2_costmap_2d::StaticLayer"      # 静态地图图层
            map_subscribe_transient_local: True         
          inflation_layer:
            plugin: "nav2_costmap_2d::InflationLayer"
            cost_scaling_factor: 10.0 #5.0              # 代价缩放因子
            inflation_radius: 0.35                      # 膨胀半径
            decay_time: 0.0                             # 0.0表示代价值立即衰减到0； 增大，比如从0.0改为1.0（提供更平滑的代价衰减），提供历史障碍物信息，对短暂遮挡更稳定，缺点是导致已清除障碍物的代价值残留
          always_send_full_costmap: True
      global_costmap_client:
        ros__parameters:
          use_sim_time: False
      global_costmap_rclcpp_node:
        ros__parameters:
          use_sim_time: False
  ```
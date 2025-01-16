# ***install ros2 mapping package and use Navigation2***

## install cartographer and test

- 1.1 install cartographer

```bashrc
sudo apt install ros-humble-cartographer
sudo apt install ros-humble-cartographer-ros
```

- 1.2 测试cartographer是否安装成功

```bashrc
ros2 pkg list | grep cartographer
```

- 1.3 出现以下安装成功

```bashrc
cartographer_ros
cartographer_ros_msgs
```

- 1.4 launch cartographer (use open_source_slam_launch package)
  [launch and lua file configuration refer](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel/turtlebot3_cartographer)

```bashrc
ros2 launch open_source_slam_launch cartographer_mapping.launch.py
```

- 1.5 save map file
  [refer 1](https://github.com/ros-navigation/navigation2/tree/humble/nav2_map_server)
  [refer 2](https://roboticsbackend.com/ros2-nav2-tutorial/)

```bashrc
ros2 run nav2_map_server map_saver_cli -f my_map_name
```

## use Navigation2 package to navigation in the above map

[refer 1](https://roboticsbackend.com/ros2-nav2-tutorial/)
[refer 2](https://github.com/ROBOTIS-GIT/turtlebot3/tree/humble-devel/turtlebot3_navigation2)

- 1.6 launch navigation

```bashrc
ros2 launch open_source_slam_launch nav2.launch.py use_sim_time:=True map:=path_to/open_source_slam_launch/maps/cartographermap2.yaml
```

#### issue 1: if local_costmap/costmap of rviz cannot show, you need check the use_sim_time.

```bashrc
ros2 launch open_source_slam_launch nav2.launch.py use_sim_time:=True
```

#### issue2: Now, you should just see the map on the screen, but no robot. You’ll also see some kinds of error logs in the terminal. This is because Nav2 doesn’t know where your robot is, and you need to **provide the first 2D pose estimate**.

# ***wheeltec Mini Diff Robot***

## Launch Cartographer on WheelTec Mini Diff Robot

```bashrc
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py
ros2 launch open_source_slam_launch wheeltec_cartographer_mapping.launch.py
ros2 launch nav2_map_server map_saver_server.launch.py
```

## Launch Navigation2 on WheelTec Mini Diff Robot

### 1.launch base and nav2

```bashrc
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py
ros2 launch open_source_slam_launch wheeltec_nav2.launch.py
```

Attention：

- If cannot show map in rviz2:
  *Set rviz2 with configuration,
  Map->Topic->Reliability Policy: Reliable
  Durability policy: Transient Local*

### 2.launch Rviz

```bashrc
ros2 launch open_source_slam_launch nav2_rviz.launch.py
```

# ***Luxshare Robot***
## Luxshare Robot cartographer

### Launch base

```
ros2 launch luxshare_robot luxshare_robot.launch.py
```
*Attention*:
- 做cartographer时，启动base不需要tf：odom->base_footprint,


### Launch Cartographer on Luxshare Robot

```bashrc
ros2 launch open_source_slam_launch luxsharerobot_cartographer_mapping.launch.py
ros2 launch nav2_map_server map_saver_server.launch.py
```

### Save map

```
ros2 run map_server_extension map_saver_client 
```
## Luxshare Robot navigation2
### Launch base
```
ros2 launch luxshare_robot luxshare_robot.launch.py
```
*Attention*:
- 在单独做nav2时，需要tf：odom->base_footprint,
- 在该机器人中是方法是打开ekf，得到tf：odom->base_footprint，但是测试发现luxshare robot的imu数据有问题，截止20250116，得到/odom_combined数据是和/odom_org一样

### Launch Navigation2 on Luxshare Robot

```
ros2 launch open_source_slam_launch luxsharerobot_nav2.launch.py
```

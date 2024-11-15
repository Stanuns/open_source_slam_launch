# install ros2 mapping package and use Navigation2

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

## Launch Navigation2 on WheelTec Mini Diff Robot

### 1.launch base and nav2

```bashrc
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py
ros2 launch open_source_slam_launch wheeltec_nav2.launch.py
```

### 2.launch Rviz

```bashrc
ros2 launch open_source_slam_launch nav2_rviz.launch.py
```

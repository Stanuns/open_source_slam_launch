"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ## ***** Launch arguments *****
    # bag_filename_arg = DeclareLaunchArgument('bag_filename')

  ## ***** File paths ******
    # pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    # urdf_dir = os.path.join(pkg_share, 'urdf')
    # urdf_file = os.path.join(urdf_dir, 'backpack_2d.urdf')
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    # ## ***** Nodes *****
    # robot_state_publisher_node = Node(
    #     package = 'robot_state_publisher',
    #     executable = 'robot_state_publisher',
    #     parameters=[
    #         {'robot_description': robot_desc},
    #         {'use_sim_time': True}],
    #     output = 'screen'
    #     )

    cartographer_prefix = get_package_share_directory('open_source_slam_launch')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='wheeltec_cartographer_mapping.lua')
    DeclareLaunchArgument(
        'cartographer_config_dir',
        default_value=cartographer_config_dir,
        description='Full path to config file to load')
    DeclareLaunchArgument(
        'configuration_basename',
        default_value=configuration_basename,
        description='Name of lua file for cartographer')
    
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': False}],
        # arguments = [
        #     '-configuration_directory', FindPackageShare('open_source_slam_launch').find('config') + '/config',
        #     '-configuration_basename', 'wheeltec_cartographer_mapping.lua'],
        arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
        remappings = [
            ('/odom', '/odom_combined'),
            ('/imu','/imu/data')
            # ('/scan_1','/scan'),
            # ('/scan_2','/camera/depth/scan')
            ],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05},
            {'-publish_period_sec': 1.0}],
        )
    
    # cartographer_odom_preproc_node = Node(
    #     package = 'cartographer_ros',
    #     executable = 'cartographer_odom_preproc',
    #     name='cartographer_odom_preproc',
    #     parameters = [
    #         {'use_sim_time': True}],
    #     )

    # rviz_node = Node(
    #     package = 'rviz2',
    #     executable = 'rviz2',
    #     on_exit = Shutdown(),
    #     arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_2d.rviz'],
    #     parameters = [{'use_sim_time': True}],
    # )

    # ros2_bag_play_cmd = ExecuteProcess(
    #     cmd = ['ros2', 'bag', 'play', LaunchConfiguration('bag_filename'), '--clock'],
    #     name = 'rosbag_play',
    # )

    # wheeltec_bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    # wheeltec_launch_dir = os.path.join(wheeltec_bringup_dir, 'launch')
    # wheeltec_robot = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(wheeltec_launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    #         launch_arguments={'carto_slam': 'true'}.items(),
    # )
    # wheeltec_lidar = IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(os.path.join(wheeltec_launch_dir, 'wheeltec_lidar.launch.py')),
    # )

    return LaunchDescription([
        # wheeltec_robot,
        # wheeltec_lidar,

        # Launch arguments
        # bag_filename_arg,
        # Nodes
        # robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        # cartographer_odom_preproc_node,
        # rviz_node,
        # ros2_bag_play_cmd
    ])
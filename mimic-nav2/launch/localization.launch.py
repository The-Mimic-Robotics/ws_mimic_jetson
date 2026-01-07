#!/usr/bin/env python3
"""
Localization Launch File for Nav2
Uses ZED2 camera from mimic-zed package + depth_to_laserscan

Topics from ZED (via mimic-zed):
  - /zed/zed_node/odom (nav_msgs/Odometry)
  - /zed/zed_node/depth/depth_registered (sensor_msgs/Image)
  - /zed/zed_node/depth/camera_info (sensor_msgs/CameraInfo)
  - /tf (ZED publishes odom → base_link)
  - /tf_static (robot_state_publisher publishes base_link → camera_link)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mimic_nav2_dir = get_package_share_directory('mimic-nav2')
    
    depth_to_scan_config = os.path.join(mimic_nav2_dir, 'config', 'depth_to_laserscan.yaml')
    urdf_file = os.path.join(mimic_nav2_dir, 'urdf', 'mecanum_robot_nav2.urdf')
    
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Robot State Publisher - static TF: base_link → camera_link, wheels, etc
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )
    
    # ZED Camera from mimic-zed package (uses default ZED config with TF enabled)
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mimic-zed'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': 'zed2',
            'camera_name': 'zed',
        }.items()
    )
    
    # Depth to LaserScan - converts ZED depth to 2D laser scan for Nav2
    depth_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        parameters=[depth_to_scan_config],
        remappings=[
            ('depth', '/zed/zed_node/depth/depth_registered'),
            ('depth_camera_info', '/zed/zed_node/depth/camera_info'),
            ('scan', '/scan'),
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        zed_launch,
        depth_to_laserscan,
    ])

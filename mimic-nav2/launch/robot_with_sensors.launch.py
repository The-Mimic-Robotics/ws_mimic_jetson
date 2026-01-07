#!/usr/bin/env python3
"""
Robot with Sensors Launch (Jetson side for distributed setup)
Launches robot driver + ZED localization only
For use when Nav2 runs on separate dev machine

Run on Jetson when using remote setup:
  ros2 launch mimic-nav2 robot_with_sensors.launch.py

Then on dev machine:
  ros2 launch mimic-nav2 navigation_remote.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port',
        default_value='/dev/ttyUSB0',
        description='ESP32 serial port'
    )
    
    camera_model_arg = DeclareLaunchArgument(
        'camera_model',
        default_value='zed2',
        description='ZED camera model'
    )
    
    # 1. Robot Driver (ESP32 communication)
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mimic-nav2'),
                'launch',
                'robot_bringup.launch.py'
            ])
        ]),
        launch_arguments={
            'esp32_port': LaunchConfiguration('esp32_port'),
        }.items()
    )
    
    # 2. Localization (ZED + depth to scan)
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mimic-nav2'),
                'launch',
                'localization.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_model': LaunchConfiguration('camera_model'),
        }.items()
    )
    
    return LaunchDescription([
        esp32_port_arg,
        camera_model_arg,
        robot_bringup,
        localization,
    ])

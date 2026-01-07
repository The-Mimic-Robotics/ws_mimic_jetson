#!/usr/bin/env python3
"""
Remote Navigation Launch File (Dev Machine)
For distributed ROS2 setup: Run this on powerful dev machine
Jetson runs: robot_bringup.launch.py + localization.launch.py
Dev machine runs: THIS FILE (navigation + RViz)

Setup:
1. On both machines, set same ROS_DOMAIN_ID:
   export ROS_DOMAIN_ID=42
   
2. Configure network (both on same subnet):
   Jetson: export ROS_LOCALHOST_ONLY=0
   Dev:    export ROS_LOCALHOST_ONLY=0
   
3. Jetson: ros2 launch mimic-nav2 robot_with_sensors.launch.py
4. Dev:    ros2 launch mimic-nav2 navigation_remote.launch.py

Topics transmitted over network:
  Jetson -> Dev: /scan, /odom, /tf, /wheel_odom
  Dev -> Jetson: /cmd_vel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    mimic_nav2_dir = get_package_share_directory('mimic-nav2')
    
    # Paths
    nav2_params_file = os.path.join(mimic_nav2_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(mimic_nav2_dir, 'rviz', 'nav2_config.rviz')
    urdf_file = os.path.join(mimic_nav2_dir, 'urdf', 'mecanum_robot_nav.urdf')
    
    # Read URDF for robot visualization
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    # Robot State Publisher (for visualization on dev machine)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Navigation (Nav2 stack running on powerful dev machine)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mimic-nav2'),
                'launch',
                'navigation.launch.py'
            ])
        ]),
        launch_arguments={
            'params_file': nav2_params_file,
        }.items()
    )
    
    # RViz for goal setting and visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': False
        }],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        use_rviz_arg,
        robot_state_publisher,
        navigation,
        rviz_node,
    ])

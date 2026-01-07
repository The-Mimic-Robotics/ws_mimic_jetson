#!/usr/bin/env python3
"""
Full System Bringup Launch File
Launches everything needed for autonomous navigation:
  - Robot driver (ESP32 communication)
  - ZED2 localization (visual odometry + depth to scan)
  - Nav2 stack (planning, control, behaviors)
  - RViz with Nav2 panel

This is the main launch file for complete autonomous navigation on Jetson.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    mimic_nav2_dir = get_package_share_directory('mimic-nav2')
    
    # Paths
    nav2_params_file = os.path.join(mimic_nav2_dir, 'config', 'nav2_params.yaml')
    rviz_config = os.path.join(mimic_nav2_dir, 'rviz', 'nav2_config.rviz')
    
    # Launch arguments
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port',
        default_value='/dev/esp32',
        description='ESP32 serial port'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
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
    
    # 3. Navigation (Nav2 stack)
    # IMPORTANT: Delay Nav2 startup to give ZED time to initialize and publish odom frame
    # ZED takes ~5-6 seconds to start positional tracking
    navigation = TimerAction(
        period=8.0,  # Wait 8 seconds for ZED to be ready
        actions=[
            IncludeLaunchDescription(
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
        ]
    )
    
    # 4. RViz for visualization and goal setting
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
        # Arguments
        esp32_port_arg,
        use_rviz_arg,
        camera_model_arg,
        
        # Launch includes
        robot_bringup,
        localization,
        navigation,
        
        # RViz
        rviz_node,
    ])

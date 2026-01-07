#!/usr/bin/env python3
"""
Robot Bringup Launch File
Starts the robot driver (ESP32 communication) only
Used when running Nav2 on a separate machine
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    mimic_teleop_dir = get_package_share_directory('mimic-teleop')
    
    # Launch arguments
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port',
        default_value='/dev/ttyUSB0',
        description='ESP32 serial port'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )
    
    # Twist to Serial Node (Robot Driver)
    # This node handles ESP32 communication and publishes /odom (NO TF!)
    twist_to_serial_node = Node(
        package='mimic-teleop',
        executable='twist_to_serial.py',
        name='twist_to_serial',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('esp32_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'twist_topic': '/cmd_vel',
            'publish_tf': False,  # IMPORTANT: Let ZED publish odom->base_link
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/odom', '/wheel_odom'),  # Rename to avoid confusion with ZED odom
        ]
    )
    
    return LaunchDescription([
        esp32_port_arg,
        baud_rate_arg,
        twist_to_serial_node,
    ])

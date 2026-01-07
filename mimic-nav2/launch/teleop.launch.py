#!/usr/bin/env python3
"""
Teleoperation Launch File
Launches Xbox controller for manual driving during Nav2 mapping
Uses the EXACT implementation from mimic-teleop package

This is useful for:
- Manual driving while Nav2 costmaps are active
- Generating maps by manually exploring the environment
- Testing robot movement before autonomous navigation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    mimic_nav2_dir = get_package_share_directory('mimic-nav2')
    joy_params = os.path.join(mimic_nav2_dir, 'config', 'joystick.yml')
    
    # Launch arguments
    enable_teleop_arg = DeclareLaunchArgument(
        'enable_teleop',
        default_value='true',
        description='Enable joystick teleoperation'
    )
    
    # joy_node - Reads Xbox controller input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
        output='screen'
    )

    # teleop_node - Converts joystick to Twist messages
    # Publishes to /cmd_vel which Nav2 also uses
    # When you press the enable button, joystick takes control
    # When released, Nav2 can control again
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[('/cmd_vel', '/cmd_vel')],  # Ensure it publishes to /cmd_vel
        output='screen'
    )

    return LaunchDescription([
        enable_teleop_arg,
        joy_node,
        teleop_node,
    ])

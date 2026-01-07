#!/usr/bin/env python3
"""
ZED2 Camera Launch File (No RViz)
For integration with Nav2 - only launches camera node
"""

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription
)
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
import os


def generate_launch_description():
    mimic_zed_dir = get_package_share_directory('mimic-zed')
    zed_config = os.path.join(mimic_zed_dir, 'config', 'zed2_config.yaml')
    
    # ZED Wrapper launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_model': LaunchConfiguration('camera_model'),
            'publish_urdf': 'true',  # Let ZED publish its own URDF with correct frame names
            'ros_params_override_path': zed_config,  # Use our config with base_frame='base_link'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value=TextSubstitution(text='zed'),
            description='The name of the camera. It will be used as node namespace.'
        ),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2',
            description='The model of the camera.',
            choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm']
        ),
        zed_wrapper_launch
    ])

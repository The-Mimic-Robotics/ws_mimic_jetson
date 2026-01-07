#!/usr/bin/env python3
# Copyright 2024 Stereolabs - Adapted for Mimic Nav2
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
ZED Display Launch File - Following Stereolabs Example Pattern
Starts ZED camera with RViz visualization, matching working example
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):

    # Launch configuration variables
    start_zed_node = LaunchConfiguration('start_zed_node')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    publish_svo_clock = LaunchConfiguration('publish_svo_clock')
    use_rviz = LaunchConfiguration('use_rviz')

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    camera_type = ''
    if( camera_model_val=='zed' or
        camera_model_val=='zedm' or
        camera_model_val=='zed2' or
        camera_model_val=='zed2i' or
        camera_model_val=='zedx' or
        camera_model_val=='zedxm' or
        camera_model_val=='virtual'):
        camera_type = 'stereo'
    else: # 'zedxonegs' or 'zedxone4k')
        camera_type = 'mono'

    # Get package directories
    mimic_nav2_dir = get_package_share_directory('mimic-nav2')
    
    # Override config path (optional - comment out to use defaults like Stereolabs)
    # zed_override_config = os.path.join(mimic_nav2_dir, 'config', 'zed_nav2_override.yaml')
    
    # RVIZ2 Configuration - use ZED's default or our custom
    config_rviz2 = os.path.join(
        mimic_nav2_dir,
        'rviz',
        'nav2_default_view.rviz'  # Our Nav2 config
    )
    
    # Alternative: Use ZED's default RViz config (uncomment to use)
    # try:
    #     zed_display_dir = get_package_share_directory('zed_display_rviz2')
    #     config_rviz2 = os.path.join(
    #         zed_display_dir,
    #         'rviz2',
    #         'zed_' + camera_type + '.rviz'
    #     )
    # except:
    #     pass  # Fallback to our config if zed_display_rviz2 not installed

    # RVIZ2 node - following Stereolabs pattern exactly
    rviz2_node = Node(
        package='rviz2',
        namespace=camera_name_val,
        executable='rviz2',
        name=camera_model_val + '_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
        parameters=[{'use_sim_time': publish_svo_clock}],
        condition=IfCondition(use_rviz)
    )

    # ZED Wrapper launch file - EXACTLY like Stereolabs example
    # NO ros_params_override_path - use defaults!
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('zed_wrapper'),
                'launch',
                'zed_camera.launch.py'
            ])
        ]),
        launch_arguments={
            'camera_name': camera_name_val,
            'camera_model': camera_model_val,
            'publish_svo_clock': publish_svo_clock
        }.items(),
        condition=IfCondition(start_zed_node)
    )

    return [
        rviz2_node,
        zed_wrapper_launch
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_zed_node',
                default_value='True',
                description='Set to `False` to start only RVIZ2 if a ZED node is already running.'),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text='zed'),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'camera_model',
                default_value='zed2',
                description='The model of the camera. Using a wrong camera model can disable camera features.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual', 'zedxonegs', 'zedxone4k']),
            DeclareLaunchArgument(
                'publish_svo_clock',
                default_value='false',
                description='If set to `true` the node will act as a clock server publishing the SVO timestamp. This is useful for node synchronization'),
            DeclareLaunchArgument(
                'use_rviz',
                default_value='true',
                description='Start RViz2 for visualization'),
            OpaqueFunction(function=launch_setup)
        ]
    )

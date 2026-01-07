#!/usr/bin/env python3
"""
ZED2 Camera Display Launch File
Based on Stereolabs zed_display_rviz2 example
Launches ZED2 camera with RViz2 visualization
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
    TextSubstitution
)
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Launch configuration variables
    start_zed_node = LaunchConfiguration('start_zed_node')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    publish_svo_clock = LaunchConfiguration('publish_svo_clock')

    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)

    if camera_name_val == '':
        camera_name_val = 'zed'

    # ZED2 is a stereo camera
    camera_type = 'stereo'

    # RViz2 Configuration
    config_rviz2 = os.path.join(
        get_package_share_directory('mimic-zed'),
        'rviz',
        'zed_stereo.rviz'
    )

    # RViz2 node
    rviz2_node = Node(
        package='rviz2',
        namespace=camera_name_val,
        executable='rviz2',
        name=camera_model_val + '_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
        parameters=[{'use_sim_time': publish_svo_clock}]
    )

    # ZED Wrapper launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
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
    return LaunchDescription([
        DeclareLaunchArgument(
            'start_zed_node',
            default_value='True',
            description='Set to `False` to start only RVIZ2 if a ZED node is already running.'
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value=TextSubstitution(text='zed'),
            description='The name of the camera. It will be used as node namespace.'
        ),
        DeclareLaunchArgument(
            'camera_model',
            default_value='zed2',
            description='The model of the camera.',
            choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual']
        ),
        DeclareLaunchArgument(
            'publish_svo_clock',
            default_value='false',
            description='If set to `true` the node will act as a clock server publishing the SVO timestamp.'
        ),
        OpaqueFunction(function=launch_setup)
    ])

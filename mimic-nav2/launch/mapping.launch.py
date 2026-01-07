#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch mapping mode with SLAM Toolbox
    - Robot driver (twist_to_serial)
    - ZED camera with positional tracking
    - depth_to_laserscan
    - SLAM Toolbox for mapping
    - RViz with mapping visualization (map, colored point cloud, video feed)
    """
    
    # Package directories
    mimic_nav2_dir = get_package_share_directory('mimic-nav2')
    mimic_teleop_dir = get_package_share_directory('mimic-teleop')
    
    # Configuration files
    slam_params_file = os.path.join(mimic_nav2_dir, 'config', 'slam_toolbox_params.yaml')
    rviz_config = os.path.join(mimic_nav2_dir, 'rviz', 'nav2_mapping.rviz')
    urdf_file = os.path.join(mimic_nav2_dir, 'urdf', 'mecanum_robot_nav2.urdf')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 1. Robot driver (mimic-teleop)
    robot_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mimic-teleop'),
                'launch',
                'mimic_teleop.launch.py'
            ])
        ]),
        launch_arguments={
            'publish_tf': 'false',  # ZED handles TF publishing
        }.items()
    )
    
    # 2. Robot state publisher (for robot URDF: base_footprint → base_link → wheels)
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )
    
    # 2b. Static transform: zed_camera_link → base_link (ZED as parent, base_link is robot center below camera)
    # ZED publishes: odom → zed_camera_link
    # This adds: zed_camera_link → base_link (-0.2m in z = 20cm below camera)
    # Result: odom → zed_camera_link → base_link
    zed_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='zed_to_base_static_tf',
        arguments=['0', '0', '-0.2', '0', '0', '0', 'zed_camera_link', 'base_link'],
        output='screen'
    )
    
    # 3. ZED camera with positional tracking (from mimic-zed package)
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mimic-zed'),
                'launch',
                'zed_camera.launch.py'
            ])
        ])
    )
    
    # 4. Depth to laserscan converter
    depth_to_laserscan = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_node',
        remappings=[
            ('depth', '/zed/zed_node/depth/depth_registered'),
            ('depth_camera_info', '/zed/zed_node/depth/camera_info'),
            ('scan', '/scan')
        ],
        parameters=[{
            'output_frame': 'zed_left_camera_frame',
            'scan_height': 10,
            'range_min': 0.3,
            'range_max': 8.0,
        }]
    )
    
    # 5. SLAM Toolbox (async mapping mode)
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 6. RViz with mapping configuration (shows map, colored point cloud, video)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Delay SLAM Toolbox to ensure ZED is ready
    delayed_slam = TimerAction(
        period=8.0,
        actions=[slam_toolbox]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        robot_driver,
        robot_state_publisher,
        zed_to_base_tf,
        zed_launch,
        depth_to_laserscan,
        delayed_slam,
        rviz,
    ])

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('mimic-teleop')
    
    # Paths to URDF and RViz config
    urdf_file = os.path.join(package_dir, 'urdf', 'mecanum_robot.urdf')
    rviz_config = os.path.join(package_dir, 'rviz', 'mecanum_robot.rviz')
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    # Robot State Publisher - publishes robot TF transforms
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
    
    # NOTE: joint_state_publisher removed!
    # Our twist_to_serial node now publishes joint states from encoder data
    
    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': False
        }]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        rviz_node
    ])

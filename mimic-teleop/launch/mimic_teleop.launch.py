from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('mimic-teleop')
    joy_params = os.path.join(package_dir, 'config', 'joystick.yml')
    
    # Launch arguments
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port',
        default_value='/dev/esp32',
        description='ESP32 serial port'
    )
    
    # joy_node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        parameters=[joy_params],
    )

    # joy to twist
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_node',
        parameters=[joy_params],
        remappings=[('/cmd_vel', '/cmd_vel')],  # ensure it publishes to /cmd_vel
        output='screen'
    )

    # Our custom twist to serial bridge (replaces twist_to_wheel_node)
    twist_to_serial_node = Node(
        package='mimic-teleop',
        executable='twist_to_serial.py',
        name='twist_to_serial',
        parameters=[{
            'serial_port': LaunchConfiguration('esp32_port'),
            'baud_rate': 115200,
            'twist_topic': '/cmd_vel',
        }],
        output='screen'
    )

    topic_monitor = Node(
        package='mimic-teleop',
        executable='topic_monitor.py',  
        name='topic_monitor',
        output='screen'
    )

    return LaunchDescription([
        esp32_port_arg,
        joy_node,
        teleop_node,
        twist_to_serial_node,
        topic_monitor
    ])

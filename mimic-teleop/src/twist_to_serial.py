#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import time
import sys
import math

class TwistToSerial(Node):
    def __init__(self):
        super().__init__('twist_to_serial')
        
        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')  # ESP32 USB port
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('twist_topic', '/cmd_vel')
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        twist_topic = self.get_parameter('twist_topic').get_parameter_value().string_value
        
        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(serial_port, baud_rate, timeout=1)
            time.sleep(2)  # Give ESP32 time to reset
            self.get_logger().info(f'Connected to ESP32 on {serial_port} at {baud_rate} baud')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to ESP32: {e}')
            sys.exit(1)
        
        # Subscribe to twist messages
        self.subscription = self.create_subscription(
            Twist,
            twist_topic,
            self.twist_callback,
            10
        )
        
        # Publishers for odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info(f'Listening to {twist_topic} and forwarding to ESP32')
        
        # Safety timer - send zero commands if no messages received
        self.last_message_time = time.time()
        self.safety_timer = self.create_timer(0.1, self.safety_check)  # 10Hz safety check
        
        # Timer for reading odometry from ESP32
        self.odom_timer = self.create_timer(0.05, self.read_odometry)  # 20Hz odometry read
        
    def twist_callback(self, msg):
        """Convert ROS2 Twist message to serial string and send to ESP32"""
        
        # Extract twist values
        linear_x = msg.linear.x    # forward/backward
        linear_y = msg.linear.y    # strafe left/right (usually 0 for diff drive)
        angular_z = msg.angular.z  # rotation
        
        corrected_linear_y = -linear_y  # Keep strafing as is
        corrected_angular_z = -angular_z  # Invert rotation direction
        
        
        # Create serial message: "TWIST,linear_x,linear_y,angular_z\n"
        serial_msg = f"TWIST,{linear_x:.3f},{corrected_linear_y:.3f},{corrected_angular_z:.3f}\n"
        
        try:
            self.serial_conn.write(serial_msg.encode('utf-8'))
            self.last_message_time = time.time()
            
            # Debug output (comment out if too verbose)
            if abs(linear_x) > 0.01 or abs(linear_y) > 0.01 or abs(angular_z) > 0.01:
                self.get_logger().debug(f'Sent: {serial_msg.strip()}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to send serial data: {e}')
    
    def safety_check(self):
        """Send stop command if no twist messages received for too long"""
        
        timeout = 1.0  # 1 second timeout
        if time.time() - self.last_message_time > timeout:
            # Send stop command
            stop_msg = "TWIST,0.000,0.000,0.000\n"
            try:
                self.serial_conn.write(stop_msg.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Failed to send stop command: {e}')
    
    def read_odometry(self):
        """Read odometry data from ESP32 serial port"""
        try:
            if self.serial_conn.in_waiting:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line.startswith('ODOM,'):
                    self.parse_and_publish_odom(line)
        except Exception as e:
            self.get_logger().error(f'Failed to read odometry: {e}')
    
    def parse_and_publish_odom(self, line):
        """Parse ODOM message and publish as ROS2 Odometry
        
        Expected format: ODOM,x,y,theta,vx,vy,omega,enc1,enc2,enc3,enc4
        """
        try:
            parts = line.split(',')
            if len(parts) != 11:
                self.get_logger().warning(f'Invalid ODOM message format: {line}')
                return
            
            # Parse odometry values
            x = float(parts[1])
            y = float(parts[2])
            theta = float(parts[3])
            vx = float(parts[4])
            vy = float(parts[5])
            omega = float(parts[6])
            # enc1-4 are in parts[7:11] if needed for debugging
            
            # Create and publish odometry message
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            
            # Position
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0
            
            # Orientation (quaternion from theta)
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = math.sin(theta / 2.0)
            odom.pose.pose.orientation.w = math.cos(theta / 2.0)
            
            # Velocity (in robot frame for mecanum)
            odom.twist.twist.linear.x = vy  # forward
            odom.twist.twist.linear.y = vx  # strafe
            odom.twist.twist.angular.z = omega
            
            # Publish odometry
            self.odom_pub.publish(odom)
            
            # Publish TF transform (odom -> base_link)
            t = TransformStamped()
            t.header.stamp = odom.header.stamp
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)
            
        except (ValueError, IndexError) as e:
            self.get_logger().error(f'Failed to parse odometry data: {e}')
    
    def destroy_node(self):
        """Clean up when shutting down"""
        if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
            # Send final stop command
            stop_msg = "TWIST,0.000,0.000,0.000\n"
            self.serial_conn.write(stop_msg.encode('utf-8'))
            time.sleep(0.1)
            self.serial_conn.close()
            self.get_logger().info('Serial connection closed')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TwistToSerial()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

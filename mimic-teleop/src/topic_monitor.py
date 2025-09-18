#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor')
        
        # Subscribe to cmd_vel to monitor commands
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.get_logger().info('Topic monitor started - monitoring /cmd_vel')
        
    def cmd_vel_callback(self, msg):
        """Monitor and log cmd_vel messages"""
        
        # Only log if there's significant movement
        if (abs(msg.linear.x) > 0.01 or 
            abs(msg.linear.y) > 0.01 or 
            abs(msg.angular.z) > 0.01):
            
            self.get_logger().info(
                f'CMD_VEL: linear=[{msg.linear.x:.3f}, {msg.linear.y:.3f}, {msg.linear.z:.3f}], '
                f'angular=[{msg.angular.x:.3f}, {msg.angular.y:.3f}, {msg.angular.z:.3f}]'
            )

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TopicMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

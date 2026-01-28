#!/usr/bin/env python3
"""
Node to automatically set initial pose for AMCL localization.
Publishes initial pose at (0, 0) after a short delay.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class SetInitialPoseNode(Node):
    def __init__(self):
        super().__init__('set_initial_pose_node')
        
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('delay', 3.0)  # Delay before publishing (seconds)
        
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )
        
        # Wait for delay, then publish
        delay = self.get_parameter('delay').value
        self.get_logger().info(f'Waiting {delay} seconds before setting initial pose...')
        
        self.timer = self.create_timer(delay, self.publish_initial_pose)
        self.published = False
    
    def publish_initial_pose(self):
        if self.published:
            return
        
        x = self.get_parameter('x').value
        y = self.get_parameter('y').value
        yaw = self.get_parameter('yaw').value
        
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        
        # Convert yaw to quaternion (simplified for z-axis rotation)
        import math
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # Set covariance
        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467
        ]
        
        self.publisher.publish(msg)
        self.get_logger().info(f'Initial pose set: x={x}, y={y}, yaw={yaw}')
        
        self.published = True
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = SetInitialPoseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

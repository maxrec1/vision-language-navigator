#!/usr/bin/env python3
"""
Test script for Vision Detector Node
Tests if the detector can receive target objects and publish detections
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time


class DetectorTester(Node):
    """Test client for vision detector"""
    
    def __init__(self):
        super().__init__('detector_tester')
        self.pub_target = self.create_publisher(String, '/detection_target', 10)
        self.get_logger().info('Detector Tester ready')
    
    def set_target(self, target_name):
        """Publish a target object to detect"""
        msg = String()
        msg.data = target_name
        self.pub_target.publish(msg)
        self.get_logger().info(f'Published target: {target_name}')


def main():
    rclpy.init()
    tester = DetectorTester()
    
    # Wait for publisher to establish
    time.sleep(1)
    
    if len(sys.argv) > 1:
        target = sys.argv[1]
    else:
        target = 'chair'
    
    print(f'\n{"="*60}')
    print(f'VISION DETECTOR TEST')
    print(f'{"="*60}')
    print(f'Setting detection target to: "{target}"')
    print(f'Now drive your robot in front of a {target} to test detection')
    print(f'View results: ros2 run rqt_image_view rqt_image_view /vision/detections')
    print(f'{"="*60}\n')
    
    tester.set_target(target)
    
    # Keep node alive
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

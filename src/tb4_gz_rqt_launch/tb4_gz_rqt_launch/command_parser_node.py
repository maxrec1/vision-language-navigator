#!/usr/bin/env python3
"""
ROS 2 Service Node for LLM-based Command Parsing
Wraps ollama_test.parse_command() in a ROS 2 service interface
"""

import rclpy
from rclpy.node import Node
from .ollama_test import parse_command
import logging
import threading
from std_msgs.msg import String
import sys
from rcl_interfaces.msg import ParameterValue

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('CommandParserNode')


class CommandParserNode(Node):
    """ROS 2 Interactive Command Parser - publishes parsed targets"""
    
    def __init__(self):
        super().__init__('command_parser_node')
        
        # Publisher for parsed targets
        self.target_pub = self.create_publisher(String, '/detection_target', 10)
        
        # Thread control
        self._stop_event = threading.Event()
        self._interactive_thread = threading.Thread(
            target=self._interactive_loop, daemon=True
        )
        self._interactive_thread.start()
        
        self.get_logger().info('Command Parser Node started - interactive mode active')
    
    def _interactive_loop(self):
        """Continuously accept user input, parse it, and publish targets one at a time."""
        try:
            print('\n' + '=' * 60)
            print('[6] INTERACTIVE MODE - Enter your own command')
            print('=' * 60 + '\n')
            
            while not self._stop_event.is_set():
                try:
                    print("\nEnter a navigation command (or press Enter to skip):")
                    print(">> ", end="", flush=True)  # Add flush=True to ensure prompt prints immediately
                    sys.stdout.flush()
                except (EOFError, KeyboardInterrupt):
                    print('\nInterrupted by user.\n')
                    break

                user_command = input()

                if user_command.strip() == '':
                    print('\nInterrupted by user.\n')
                    break

                # Parse command and publish only the target
                try:
                    result = parse_command(user_command)
                    target = (result or {}).get('target', '') or ''
                    relation = (result or {}).get('relation') or ''
                    reference = (result or {}).get('reference') or ''

                    # Publish the target to the topic (one message per command)
                    msg = String()
                    msg.data = target
                    self.target_pub.publish(msg)

                    # Print parsed summary
                    print('\nParsed:')
                    print(f'  target="{target}"')
                    print(f'  relation="{relation}"')
                    print(f'  reference="{reference}"\n')
                except Exception as e:
                    self.get_logger().error(f'Interactive parse error: {e}')
                    msg = String()
                    msg.data = ''
                    self.target_pub.publish(msg)

            print('=' * 60)
            print('✅ Test complete!')
            print('=' * 60 + '\n')
        finally:
            self._stop_event.set()

    def shutdown(self):
        """Signal the interactive loop to stop and join the thread."""
        self._stop_event.set()
        if self._interactive_thread.is_alive():
            self._interactive_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = CommandParserNode()
    
    # Suppress INFO logs during interactive mode
    node.declare_parameter('log_level', rclpy.logging.LoggingSeverity.WARN)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

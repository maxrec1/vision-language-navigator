#!/usr/bin/env python3
"""
ROS 2 Service Node for LLM-based Command Parsing
Wraps ollama_test.parse_command() in a ROS 2 service interface
"""

import rclpy
from rclpy.node import Node
from tb4_interfaces.srv import ParseCommand
from .ollama_test import parse_command
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('CommandParserNode')


class CommandParserNode(Node):
    """ROS 2 Service Server for parsing natural language commands"""
    
    def __init__(self):
        super().__init__('command_parser_node')
        
        # Create service server
        self.srv = self.create_service(
            ParseCommand,
            '/parse_command',
            self.handle_parse_command
        )
        
        self.get_logger().info('Command Parser Service started on /parse_command')
        self.get_logger().info('Waiting for requests...')
    
    def handle_parse_command(self, request, response):
        """
        Service callback: Parse natural language command
        
        Args:
            request: ParseCommand.Request with command_text field
            response: ParseCommand.Response to fill with results
        
        Returns:
            ParseCommand.Response with parsed goal fields
        """
        command_text = request.command_text
        self.get_logger().info(f'Received command: "{command_text}"')
        
        try:
            # Call the parse_command function from ollama_test.py
            result = parse_command(command_text)
            
            if result is None:
                self.get_logger().error('parse_command returned None')
                response.success = False
                response.target_object = ''
                response.relation_label = ''
                response.reference_object = ''
                return response
            
            # Map JSON keys to service response fields
            response.target_object = result.get('target', '')
            response.relation_label = result.get('relation') or ''
            response.reference_object = result.get('reference') or ''
            response.success = True
            
            self.get_logger().info(
                f'Parsed: target="{response.target_object}", '
                f'relation="{response.relation_label}", '
                f'reference="{response.reference_object}"'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error parsing command: {e}')
            response.success = False
            response.target_object = ''
            response.relation_label = ''
            response.reference_object = ''
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin the node
    node = CommandParserNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
YOLOWorld-based Object Detection Node for TurtleBot4
Block 1: Basic detector with bounding box visualization
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLOWorld
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy


class VisionDetectorNode(Node):
    """Real-time object detection using YOLOWorld"""
    
    def __init__(self):
        super().__init__('vision_detector_node')
        
        # Declare parameters
        self.declare_parameter('model_size', 'l')  # s, m, l
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('camera_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('detection_rate_hz', 10.0)
        
        model_size = self.get_parameter('model_size').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        camera_topic = self.get_parameter('camera_topic').value
        detection_rate = self.get_parameter('detection_rate_hz').value
        
        self.get_logger().info(f'Loading YOLOWorld model (size: {model_size})...')
        try:
            self.model = YOLOWorld(f'yolov8{model_size}-worldv2.pt')
        except Exception as e:
            self.get_logger().warn(f'Could not load YOLOWorld, trying YOLOv8: {e}')
            self.model = YOLOWorld(f'yolov8{model_size}.pt')
        
        # Set default detection classes
        self.model.set_classes([
            'chair', 'table', 'lamp', 'sofa', 'desk', 'cabinet', 'door',
            'window', 'bed', 'shelf', 'plant', 'person', 'cup', 'bottle',
            'monitor', 'keyboard', 'mouse', 'book', 'vase', 'clock'
        ])
        
        self.cv_bridge = CvBridge()
        self.latest_frame = None
        self.target_object = None  # What we're looking for
        self.latest_detections = {}
        self.lock = threading.Lock()
        self.inference_running = False
        
        # Subscribe to camera with best effort QoS (important for Gazebo)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.sub_camera = self.create_subscription(
            Image,
            camera_topic,
            self.camera_callback,
            qos
        )
        
        # Subscribe to target object commands
        self.sub_target = self.create_subscription(
            String,
            '/detection_target',
            self.target_callback,
            10
        )
        
        # Publisher for annotated detections
        self.pub_detections = self.create_publisher(
            Image, 
            '/vision/detections', 
            10
        )
        
        # Timer for periodic detection (non-blocking)
        detection_period = 1.0 / detection_rate
        self.timer = self.create_timer(detection_period, self.detect_objects)
        
        self.get_logger().info(f'Vision Detector initialized')
        self.get_logger().info(f'Subscribing to: {camera_topic}')
        self.get_logger().info(f'Publishing to: /vision/detections')
        self.get_logger().info(f'Listening for target on: /detection_target')
        self.get_logger().info(f'Detection rate: {detection_rate} Hz')
    
    def camera_callback(self, msg: Image):
        """Store latest camera frame (non-blocking)"""
        try:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.lock:
                self.latest_frame = frame
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def target_callback(self, msg: String):
        """Update target object to detect"""
        target = msg.data.strip().lower()
        with self.lock:
            self.target_object = target
        self.get_logger().info(f'Target object set to: "{target}"')
    
    def detect_objects(self):
        """Run YOLOWorld inference on latest frame (timer callback)"""
        # Skip if inference is already running
        if self.inference_running:
            self.get_logger().debug('Skipping frame - inference still running')
            return
        
        # Get latest frame
        with self.lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()
            target = self.target_object
        
        self.inference_running = True
        
        try:
            # Run YOLO inference
            results = self.model.predict(
                frame, 
                conf=self.conf_threshold, 
                verbose=False
            )
            
            if len(results) == 0:
                # No detections
                annotated = frame
            else:
                result = results[0]
                
                # Parse detections
                detections = []
                for box, cls_id, conf in zip(
                    result.boxes.xyxy, 
                    result.boxes.cls, 
                    result.boxes.conf
                ):
                    class_name = result.names[int(cls_id)].lower()
                    confidence = float(conf)
                    bbox = box.cpu().numpy().astype(int)
                    
                    detections.append({
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': bbox,
                        'is_target': (class_name == target) if target else False
                    })
                
                # Store detections
                with self.lock:
                    self.latest_detections = detections
                
                # Draw bounding boxes
                annotated = self.draw_detections(frame, detections)
                
                # Log if target found
                if target:
                    target_found = any(d['is_target'] for d in detections)
                    if target_found:
                        count = sum(1 for d in detections if d['is_target'])
                        self.get_logger().info(f'✓ Target "{target}" found ({count} instances)')
            
            # Publish annotated frame
            try:
                msg = self.cv_bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                self.pub_detections.publish(msg)
            except Exception as e:
                self.get_logger().error(f'Error publishing annotated image: {e}')
        
        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
        
        finally:
            self.inference_running = False
    
    def draw_detections(self, frame, detections):
        """Draw bounding boxes on frame. Highlight target in different color."""
        annotated = frame.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            class_name = det['class']
            conf = det['confidence']
            is_target = det['is_target']
            
            # Choose color: GREEN for target, BLUE for others
            color = (0, 255, 0) if is_target else (255, 100, 0)  # Green / Orange-Blue
            thickness = 3 if is_target else 2
            
            # Draw bounding box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)
            
            # Draw label with background
            label = f'{class_name} {conf:.2f}'
            if is_target:
                label = f'TARGET: {label}'
            
            (text_w, text_h), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            
            # Background rectangle for text
            cv2.rectangle(
                annotated, 
                (x1, y1 - text_h - 10), 
                (x1 + text_w, y1), 
                color, 
                -1
            )
            
            # Text
            cv2.putText(
                annotated, 
                label, 
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 
                0.6, 
                (255, 255, 255),  # White text
                2
            )
        
        # Draw status text
        status_y = 30
        if self.target_object:
            status_text = f'Looking for: {self.target_object}'
            cv2.putText(
                annotated,
                status_text,
                (10, status_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),  # Yellow
                2
            )
        
        return annotated
    
    def get_detected_objects(self):
        """Get currently detected objects (thread-safe)"""
        with self.lock:
            return self.latest_detections.copy()


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down Vision Detector...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

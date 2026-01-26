#!/usr/bin/env python3
"""
YOLOWorld-based Object Detection Node for TurtleBot4
Block 1: Basic detector with bounding box visualization
Block 2: 3D localization with depth camera and TF2 transforms
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLOWorld
import threading
from rclpy.qos import QoSProfile, ReliabilityPolicy
import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
import message_filters


class VisionDetectorNode(Node):
    """Real-time object detection using YOLOWorld with 3D localization"""
    
    def __init__(self):
        super().__init__('vision_detector_node')
        
        # Declare parameters
        self.declare_parameter('model_size', 'l')  # s, m, l
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('camera_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('depth_topic', '/oakd/rgb/preview/depth')
        self.declare_parameter('camera_info_topic', '/oakd/rgb/preview/camera_info')
        self.declare_parameter('detection_rate_hz', 10.0)
        self.declare_parameter('target_frame', 'map')  # or 'odom'
        self.declare_parameter('camera_frame', 'oakd_rgb_camera_optical_frame')
        
        model_size = self.get_parameter('model_size').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        camera_topic = self.get_parameter('camera_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        detection_rate = self.get_parameter('detection_rate_hz').value
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        
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
        self.latest_depth = None
        self.camera_info = None
        self.target_object = None  # What we're looking for
        self.latest_detections = {}
        self.lock = threading.Lock()
        self.inference_running = False
        
        # TF2 setup for coordinate transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to camera with best effort QoS (important for Gazebo)
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        
        # Synchronized RGB + Depth subscription
        self.sub_rgb = message_filters.Subscriber(self, Image, camera_topic, qos_profile=qos)
        self.sub_depth = message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos)
        
        # Approximate time synchronizer (handles slight timing differences)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.sub_rgb, self.sub_depth],
            queue_size=10,
            slop=0.1  # 100ms tolerance
        )
        self.sync.registerCallback(self.rgbd_callback)
        
        # Subscribe to camera info
        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
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
        
        # Publisher for 3D object pose (Block 2 addition)
        self.pub_object_pose = self.create_publisher(
            PoseStamped,
            '/vision/object_pose',
            10
        )
        
        # Timer for periodic detection (non-blocking)
        detection_period = 1.0 / detection_rate
        self.timer = self.create_timer(detection_period, self.detect_objects)
        
        self.get_logger().info(f'Vision Detector initialized (Block 2: 3D Localization)')
        self.get_logger().info(f'Subscribing to RGB: {camera_topic}')
        self.get_logger().info(f'Subscribing to Depth: {depth_topic}')
        self.get_logger().info(f'Subscribing to CameraInfo: {camera_info_topic}')
        self.get_logger().info(f'Publishing detections: /vision/detections')
        self.get_logger().info(f'Publishing object pose: /vision/object_pose')
        self.get_logger().info(f'Listening for target on: /detection_target')
        self.get_logger().info(f'Detection rate: {detection_rate} Hz')
        self.get_logger().info(f'Target frame: {self.target_frame}')
    
    def rgbd_callback(self, rgb_msg: Image, depth_msg: Image):
        """Store synchronized RGB + Depth frames"""
        try:
            rgb_frame = self.cv_bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            
            # Handle depth encoding (16UC1 in mm or 32FC1 in meters)
            if depth_msg.encoding == '16UC1':
                depth_frame = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
                depth_frame = depth_frame.astype(np.float32) / 1000.0  # mm to meters
            elif depth_msg.encoding == '32FC1':
                depth_frame = self.cv_bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            else:
                self.get_logger().warn(f'Unknown depth encoding: {depth_msg.encoding}')
                return
            
            with self.lock:
                self.latest_frame = rgb_frame
                self.latest_depth = depth_frame
                
        except Exception as e:
            self.get_logger().error(f'Error converting RGBD images: {e}')
    
    def camera_info_callback(self, msg: CameraInfo):
        """Store camera intrinsics"""
        if self.camera_info is None:
            self.camera_info = {
                'fx': msg.k[0],
                'fy': msg.k[4],
                'cx': msg.k[2],
                'cy': msg.k[5],
                'width': msg.width,
                'height': msg.height
            }
            self.get_logger().info(
                f'Camera intrinsics: fx={self.camera_info["fx"]:.1f}, '
                f'fy={self.camera_info["fy"]:.1f}, '
                f'cx={self.camera_info["cx"]:.1f}, '
                f'cy={self.camera_info["cy"]:.1f}'
            )
    
    def target_callback(self, msg: String):
        """Update target object to detect"""
        target = msg.data.strip().lower()
        with self.lock:
            self.target_object = target
        self.get_logger().info(f'Target object set to: "{target}"')
    
    def compute_3d_position(self, bbox, depth_img):
        """
        Convert 2D bounding box to 3D point in camera frame.
        
        Args:
            bbox: [x1, y1, x2, y2] bounding box
            depth_img: Depth image (meters)
            
        Returns:
            PointStamped in camera optical frame, or None if invalid
        """
        if self.camera_info is None:
            self.get_logger().warn('No camera info available for 3D projection')
            return None
        
        # Get bbox center
        x1, y1, x2, y2 = bbox
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)
        
        # Check bounds
        h, w = depth_img.shape
        if not (0 <= u < w and 0 <= v < h):
            self.get_logger().warn(f'Bbox center ({u}, {v}) out of depth image bounds ({w}x{h})')
            return None
        
        # Get depth value
        depth = depth_img[v, u]
        
        # Check for invalid depth
        if np.isnan(depth) or np.isinf(depth) or depth <= 0.0 or depth > 10.0:
            self.get_logger().warn(f'Invalid depth at ({u}, {v}): {depth}m')
            return None
        
        # Apply pinhole camera model
        fx = self.camera_info['fx']
        fy = self.camera_info['fy']
        cx = self.camera_info['cx']
        cy = self.camera_info['cy']
        
        X_cam = (u - cx) * depth / fx
        Y_cam = (v - cy) * depth / fy
        Z_cam = depth
        
        # Create PointStamped in camera optical frame
        point_cam = PointStamped()
        point_cam.header.frame_id = self.camera_frame
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.point.x = float(X_cam)
        point_cam.point.y = float(Y_cam)
        point_cam.point.z = float(Z_cam)
        
        return point_cam
    
    def transform_to_map(self, point_cam: PointStamped):
        """
        Transform point from camera frame to map frame.
        
        Args:
            point_cam: PointStamped in camera optical frame
            
        Returns:
            PointStamped in target frame (map/odom), or None if transform fails
        """
        try:
            # Look up transform (with timeout)
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                point_cam.header.frame_id,
                rclpy.time.Time(),  # Latest available
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            # Apply transform
            point_map = tf2_geometry_msgs.do_transform_point(point_cam, transform)
            
            return point_map
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'TF2 transform failed: {e}')
            return None
    
    def detect_objects(self):
        """Run YOLOWorld inference on latest frame (timer callback)"""
        # Skip if inference is already running
        if self.inference_running:
            self.get_logger().debug('Skipping frame - inference still running')
            return
        
        # Get latest frame and depth
        with self.lock:
            if self.latest_frame is None:
                return
            frame = self.latest_frame.copy()
            depth = self.latest_depth.copy() if self.latest_depth is not None else None
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
                    
                    is_target = (class_name == target) if target else False
                    
                    detections.append({
                        'class': class_name,
                        'confidence': confidence,
                        'bbox': bbox,
                        'is_target': is_target
                    })
                    
                    # Block 2: Compute and publish 3D pose for target objects
                    if is_target and depth is not None:
                        point_cam = self.compute_3d_position(bbox, depth)
                        if point_cam is not None:
                            point_map = self.transform_to_map(point_cam)
                            if point_map is not None:
                                # Create PoseStamped message
                                pose_msg = PoseStamped()
                                pose_msg.header = point_map.header
                                pose_msg.pose.position = point_map.point
                                pose_msg.pose.orientation.w = 1.0  # No rotation
                                
                                # Publish
                                self.pub_object_pose.publish(pose_msg)
                                
                                self.get_logger().info(
                                    f'📍 Target "{class_name}" at ({point_map.point.x:.2f}, '
                                    f'{point_map.point.y:.2f}, {point_map.point.z:.2f}) '
                                    f'in frame "{point_map.header.frame_id}"'
                                )
                
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

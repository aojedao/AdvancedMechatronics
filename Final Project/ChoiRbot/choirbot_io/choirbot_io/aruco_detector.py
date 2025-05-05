#!/usr/bin/env python3

"""
Aruco Detector Node for ChoiRbot
- Detects ArUco markers using OpenCV
- Supports both static image testing and live Raspberry Pi camera feed
- Estimates their 3D pose using camera calibration
- Publishes each marker's pose as a PoseStamped message
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import os

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.get_logger().info(' cv2 version: ' + cv2.__version__)

        # Parameters (customize as needed)
        self.declare_parameter('use_image', True)  # Use static image or camera feed
        self.declare_parameter('image_path', '~/Desktop/aruco_test/image1.jpeg')  # Path to test image
        self.declare_parameter('camera_topic', '/camera/image_raw')  # Camera feed topic
        self.declare_parameter('aruco_dict', 'DICT_7X7_250')
        self.declare_parameter('marker_length', 0.07)  # meters

        # Load parameters
        self.use_image = self.get_parameter('use_image').get_parameter_value().bool_value
        self.image_path = os.path.expanduser(self.get_parameter('image_path').get_parameter_value().string_value)
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.aruco_dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        
        # Log the expanded image path for debugging
        self.get_logger().info(f"Expanded image path: {self.image_path}")

        # Camera calibration (replace with your actual calibration)
        self.camera_matrix = np.array([[600, 0, 320],
                                       [0, 600, 240],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))  # Assume no distortion for demo

        # Set up ArUco dictionary
        try:
            dictionary = aruco.getPredefinedDictionary(getattr(aruco, self.aruco_dict_name))
            parameters = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(dictionary, parameters)
            self.get_logger().info(f"Using ArUco dictionary: {self.aruco_dict_name}")
        except AttributeError as e:
            self.get_logger().error(f"Invalid ArUco dictionary name: {self.aruco_dict_name}. Error: {e}")
            raise

        # ROS <-> OpenCV bridge
        self.bridge = CvBridge()

        # Publishers for marker poses (created dynamically)
        self.pose_publishers = {}

        # Start based on mode
        if self.use_image:
            self.get_logger().info('Using static image for testing.')
            self.process_image()
        else:
            self.get_logger().info('Using Raspberry Pi camera feed.')
            self.subscription = self.create_subscription(
                Image, self.camera_topic, self.image_callback, 10)

    def process_image(self):
        """Process a static image for testing."""
        
        # Log the directory contents for debugging
        directory = os.path.dirname(self.image_path)
        self.get_logger().info(f"Listing files in directory: {directory}")
        try:
            files = os.listdir(directory)
            self.get_logger().info(f"Files in directory: {files}")
        except FileNotFoundError:
            self.get_logger().error(f"Directory not found: {directory}")
            return
        
        # Load the image
        frame = cv2.imread(self.image_path)
        if frame is None:
            self.get_logger().error(f"Failed to load image from {self.image_path}")
            return

        # Detect and process markers
        self.detect_and_publish_markers(frame)

    def image_callback(self, msg):
        """Callback for processing camera feed."""
        # Convert ROS Image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect and process markers
        self.detect_and_publish_markers(frame, msg.header)

    def detect_and_publish_markers(self, frame, header=None):
        """Detect ArUco markers and publish their poses."""
        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(frame)
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                # Get the corner points for the marker
                marker_corners = corners[i].reshape((4, 2))

                # Define the 3D points of the marker corners in the marker's coordinate system
                marker_size = self.marker_length
                object_points = np.array([
                    [-marker_size / 2, -marker_size / 2, 0],
                    [ marker_size / 2, -marker_size / 2, 0],
                    [ marker_size / 2,  marker_size / 2, 0],
                    [-marker_size / 2,  marker_size / 2, 0]
                ], dtype=np.float32)

                # Estimate pose using solvePnP
                success, rvec, tvec = cv2.solvePnP(
                    object_points, marker_corners, self.camera_matrix, self.dist_coeffs
                )
                if success:
                    # Prepare PoseStamped message
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = header.stamp if header else self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = header.frame_id if header else "camera_frame"  # Replace "camera_frame" with your frame ID
                    pose_msg.pose.position.x = float(tvec[0])
                    pose_msg.pose.position.y = float(tvec[1])
                    pose_msg.pose.position.z = float(tvec[2])
                    # Convert rotation vector to quaternion
                    rot_matrix, _ = cv2.Rodrigues(rvec)
                    quat = self.rotation_matrix_to_quaternion(rot_matrix)
                    pose_msg.pose.orientation.x = quat[0]
                    pose_msg.pose.orientation.y = quat[1]
                    pose_msg.pose.orientation.z = quat[2]
                    pose_msg.pose.orientation.w = quat[3]

                   # Optionally, draw the detected markers on the image

                    # Publish on a topic for this marker
                    topic_name = f'/aruco/pose_{marker_id}'
                    if marker_id not in self.pose_publishers:
                       self.pose_publishers[marker_id] = self.create_publisher(PoseStamped, topic_name, 10)
                    self.pose_publishers[marker_id].publish(pose_msg)
                    self.get_logger().info(f'Published pose for marker {marker_id} on {topic_name}')
                    
                    aruco.drawDetectedMarkers(frame, corners, ids)
                    # Display the image with detected markers
                    cv2.imshow('Aruco Markers', frame)
                    cv2.waitKey(1)
        else:
            self.get_logger().warn("No markers detected.")

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        """Convert a rotation matrix to a quaternion (x, y, z, w)."""
        q = np.empty((4, ), dtype=np.float64)
        t = np.trace(R)
        if t > 0.0:
            t = np.sqrt(t + 1.0)
            q[3] = 0.5 * t
            t = 0.5 / t
            q[0] = (R[2, 1] - R[1, 2]) * t
            q[1] = (R[0, 2] - R[2, 0]) * t
            q[2] = (R[1, 0] - R[0, 1]) * t
        else:
            i = 0
            if R[1, 1] > R[0, 0]:
                i = 1
            if R[2, 2] > R[i, i]:
                i = 2
            j = (i + 1) % 3
            k = (j + 1) % 3
            t = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0)
            q[i] = 0.5 * t
            t = 0.5 / t
            q[3] = (R[k, j] - R[j, k]) * t
            q[j] = (R[j, i] + R[i, j]) * t
            q[k] = (R[k, i] + R[i, k]) * t
        return q

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
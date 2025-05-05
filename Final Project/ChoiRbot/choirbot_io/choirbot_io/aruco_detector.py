#!/usr/bin/env python3

"""
Aruco Detector Node for ChoiRbot
- Subscribes to a camera image topic
- Detects ArUco markers using OpenCV
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

url = "http://10.18.195.1:4747/video"

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        # Parameters (customize as needed)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_length', 0.05)  # meters

        # Load parameters
        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.aruco_dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value

        # Camera calibration (replace with your actual calibration)
        self.camera_matrix = np.array([[600, 0, 320],
                                       [0, 600, 240],
                                       [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1))  # Assume no distortion for demo

        # Set up ArUco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, self.aruco_dict_name))
        self.aruco_params = aruco.DetectorParameters()

        # ROS <-> OpenCV bridge
        self.bridge = CvBridge()

        # Subscribe to camera images
        # Use OpenCV VideoCapture to get a live feed from the URL
        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video stream from {url}")
            return

            # Timer to periodically read frames from the video feed
            self.timer = self.create_timer(0.1, self.timer_callback)

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect markers
        corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            # Estimate pose for each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            for i, marker_id in enumerate(ids.flatten()):
                # Prepare PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose.position.x = float(tvecs[i][0][0])
                pose_msg.pose.position.y = float(tvecs[i][0][1])
                pose_msg.pose.position.z = float(tvecs[i][0][2])
                # Convert rotation vector to quaternion
                rot_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                quat = self.rotation_matrix_to_quaternion(rot_matrix)
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]

                # Publish on a topic for this marker
                topic_name = f'/aruco/pose_{marker_id}'
                if marker_id not in self.pose_publishers:
                    self.pose_publishers[marker_id] = self.create_publisher(PoseStamped, topic_name, 10)
                self.pose_publishers[marker_id].publish(pose_msg)
                self.get_logger().debug(f'Published pose for marker {marker_id} on {topic_name}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame from video stream")
            return

        # Convert OpenCV image to ROS Image and process it
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_callback(msg)

        # Publishers for marker poses (created dynamically)
        self.pose_publishers = {}

        self.get_logger().info('ArucoDetector node started.')
        # Display live feed with pose estimation
        
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame from video stream")
        else:
            self.get_logger().info("Frame read successfully")

        # Detect markers
        corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
        # Estimate pose for each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            for i, marker_id in enumerate(ids.flatten()):
                # Draw axis and marker ID on the frame
                cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_length)
                cv2.putText(frame, f"ID: {marker_id}", (int(corners[i][0][0][0]), int(corners[i][0][0][1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Display the frame
        cv2.imshow("ArUco Pose Estimation", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Exiting...")
            self.cap.release()
            cv2.destroyAllWindows()
            return

        self.cap.release()
        cv2.destroyAllWindows()
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Detect markers
        corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            # Estimate pose for each marker
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
            for i, marker_id in enumerate(ids.flatten()):
                # Prepare PoseStamped message
                pose_msg = PoseStamped()
                pose_msg.header = msg.header
                pose_msg.pose.position.x = float(tvecs[i][0][0])
                pose_msg.pose.position.y = float(tvecs[i][0][1])
                pose_msg.pose.position.z = float(tvecs[i][0][2])
                # Convert rotation vector to quaternion
                rot_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                quat = self.rotation_matrix_to_quaternion(rot_matrix)
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]

                # Publish on a topic for this marker
                topic_name = f'/aruco/pose_{marker_id}'
                if marker_id not in self.pose_publishers:
                    self.pose_publishers[marker_id] = self.create_publisher(PoseStamped, topic_name, 10)
                self.pose_publishers[marker_id].publish(pose_msg)
                self.get_logger().debug(f'Published pose for marker {marker_id} on {topic_name}')

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

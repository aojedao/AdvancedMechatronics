#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import os

file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'calibration_data.npz')
data = np.load(file_path)

# DroidCam video stream URL
url = "http://10.18.238.136:4747/video"  # Nishant's DroidCam URL

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('aruco_dict', 'DICT_7X7_50')
        self.declare_parameter('marker_length', 0.07)

        self.aruco_dict_name = self.get_parameter('aruco_dict').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value

        # Load camera calibration from .npz file
        try:
            data = np.load('calibration_data.npz')
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']
            self.get_logger().info("Loaded camera calibration data.")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration file: {e}")
            return

        # Set up ArUco detection
        self.aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, self.aruco_dict_name))
        self.aruco_params = aruco.DetectorParameters()

        # Video stream
        self.cap = cv2.VideoCapture(url)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video stream from {url}")
            return
        self.get_logger().info("Video stream opened successfully")

        # ROS <-> OpenCV bridge
        self.bridge = CvBridge()

        # Pose publishers for each detected marker
        self.pose_publishers = {}

        # Timer to periodically grab and process frames
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame from video stream")
            return

        self.get_logger().info("Image callback triggered")
        cv2.imshow("Image", frame)

        corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            self.get_logger().info("Markers detected")
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            for i, marker_id in enumerate(ids.flatten()):
                # Build PoseStamped
                pose_msg = PoseStamped()
                pose_msg.header = Header()
                pose_msg.header.stamp = self.get_clock().now().to_msg()
                pose_msg.header.frame_id = "camera_frame"

                pose_msg.pose.position.x = float(tvecs[i][0][0])
                pose_msg.pose.position.y = float(tvecs[i][0][1])
                pose_msg.pose.position.z = float(tvecs[i][0][2])
                self.get_logger().info(f"Marker ID: {marker_id}, Position: {pose_msg.pose.position}")

                # Convert to quaternion
                rot_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                quat = self.rotation_matrix_to_quaternion(rot_matrix)
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]

                # Publish pose
                topic_name = f'/aruco/pose_{marker_id}'
                if marker_id not in self.pose_publishers:
                    self.pose_publishers[marker_id] = self.create_publisher(PoseStamped, topic_name, 10)
                self.pose_publishers[marker_id].publish(pose_msg)
                self.get_logger().debug(f'Published pose for marker {marker_id} on {topic_name}')

                # Annotate image
                cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], self.marker_length)
                corner = corners[i][0][0]
                cv2.putText(frame, f"ID: {marker_id}", (int(corner[0]), int(corner[1] - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Show the result
        cv2.imshow("ArUco Pose Estimation", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Exiting...")
            self.destroy_node()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

    @staticmethod
    def rotation_matrix_to_quaternion(R):
        q = np.empty((4,), dtype=np.float64)
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
            if R[1, 1] > R[0, 0]: i = 1
            if R[2, 2] > R[i, i]: i = 2
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
    rclpy.shutdown()

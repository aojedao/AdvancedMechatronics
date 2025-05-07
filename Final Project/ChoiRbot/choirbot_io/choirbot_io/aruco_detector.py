#!/usr/bin/env python3

"""
Aruco Detector Node for ChoiRbot - Fixed Version
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
import os
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Publishers for odometry data
        self.agent_0_odom_publisher = self.create_publisher(Odometry, '/agent_0/odom', 10)
        self.agent_1_odom_publisher = self.create_publisher(Odometry, '/agent_1/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('cv2 version: ' + cv2.__version__)

        # Parameters
        self.declare_parameter('droidcam_url', 'http://10.18.238.136:4747/video')
        self.declare_parameter('marker_length', 0.07)
        self.declare_parameter('show_gui', False)
        self.declare_parameter('calibration_file', 
            '/home/robot_2004/Documents/Projects/AdvMec2025/AdvancedMechatronics/Final Project/ChoiRbot/choirbot_io/choirbot_io/calibration_data.npz')

        # Load parameters
        self.droidcam_url = self.get_parameter('droidcam_url').get_parameter_value().string_value
        self.marker_length = self.get_parameter('marker_length').get_parameter_value().double_value
        self.show_gui = self.get_parameter('show_gui').get_parameter_value().bool_value
        calibration_file = self.get_parameter('calibration_file').get_parameter_value().string_value

        # Load camera calibration
        try:
            data = np.load(calibration_file)
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']
            self.get_logger().info("Loaded camera calibration data.")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration file: {e}")
            raise

        # Set up ArUco dictionary
        try:
            dictionary = aruco.getPredefinedDictionary(aruco.DICT_7X7_250)
            parameters = aruco.DetectorParameters()
            self.detector = aruco.ArucoDetector(dictionary, parameters)
            self.get_logger().info("Using ArUco dictionary: DICT_7X7_250")
        except AttributeError as e:
            self.get_logger().error(f"Error setting up ArUco detector: {e}")
            raise

        # ROS <-> OpenCV bridge
        self.bridge = CvBridge()

        # Video capture from DroidCam
        self.cap = cv2.VideoCapture(self.droidcam_url)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video stream from {self.droidcam_url}")
            raise RuntimeError(f"Could not open video stream from {self.droidcam_url}")

        # Store reference frame (marker 0) transform
        self.ref_transform = None
        self.ref_rvec = None
        self.ref_tvec = None

        # Timer to process frames
        self.timer = self.create_timer(0.04, self.process_frame)

    def process_frame(self):
        """Capture and process frame from DroidCam."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to read frame from video stream")
            return

        self.detect_and_publish_markers(frame)

        if self.show_gui:
            self.display_frame(frame)

    def display_frame(self, frame):
        """Display the processed frame with markers and info."""
        try:
            # Draw coordinate axes for marker 0 if found
            if self.ref_rvec is not None and self.ref_tvec is not None:
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, 
                                 self.ref_rvec, self.ref_tvec, self.marker_length * 1.5)
                
                # Display reference marker info
                cv2.putText(frame, "Reference Frame (Marker 0)", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow("ArUco Marker Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("User requested shutdown")
                self.destroy_node()
        except Exception as e:
            self.get_logger().error(f"Error in display_frame: {e}")

    def detect_and_publish_markers(self, frame):
        """Detect ArUco markers and publish their poses as odometry."""
        try:
            corners, ids, rejected = self.detector.detectMarkers(frame)
            
            if ids is not None:
                # Find marker 0 first to use as reference
                ref_idx = None
                if 0 in ids:
                    ref_idx = np.where(ids == 0)[0][0]
                    self.process_reference_marker(corners[ref_idx][0])
                
                # Process other markers relative to marker 0
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id in [4, 5]:  # Only process markers 4 and 5
                        self.process_agent_marker(marker_id, corners[i][0], frame)
        except Exception as e:
            self.get_logger().error(f"Error in detect_and_publish_markers: {e}")

    def process_reference_marker(self, corners):
        """Process marker 0 and store its transform."""
        try:
            # Estimate pose of reference marker
            object_points = self.get_marker_object_points()
            success, rvec, tvec = cv2.solvePnP(
                object_points, corners, self.camera_matrix, self.dist_coeffs)
            
            if success:
                # Store the reference transform and pose data
                self.ref_rvec = rvec
                self.ref_tvec = tvec
                
                rot_matrix, _ = cv2.Rodrigues(rvec)
                self.ref_transform = TransformStamped()
                self.ref_transform.header.stamp = self.get_clock().now().to_msg()
                self.ref_transform.header.frame_id = "camera_frame"
                self.ref_transform.child_frame_id = "marker_0_frame"
                
                self.ref_transform.transform.translation.x = float(tvec[0])
                self.ref_transform.transform.translation.y = float(tvec[1])
                self.ref_transform.transform.translation.z = float(tvec[2])
                
                quat = self.rotation_matrix_to_quaternion(rot_matrix)
                self.ref_transform.transform.rotation.x = quat[0]
                self.ref_transform.transform.rotation.y = quat[1]
                self.ref_transform.transform.rotation.z = quat[2]
                self.ref_transform.transform.rotation.w = quat[3]
                
                # Broadcast TF
                self.tf_broadcaster.sendTransform(self.ref_transform)
        except Exception as e:
            self.get_logger().error(f"Error in process_reference_marker: {e}")

    def process_agent_marker(self, marker_id, corners, frame):
        """Process agent markers (4 and 5) relative to marker 0."""
        try:
            if self.ref_transform is None:
                self.get_logger().warn("Reference marker 0 not found, skipping agent markers")
                return
                
            # Estimate pose of agent marker
            object_points = self.get_marker_object_points()
            success, rvec, tvec = cv2.solvePnP(
                object_points, corners, self.camera_matrix, self.dist_coeffs)
            
            if success:
                # Convert to relative pose with respect to marker 0
                agent_pos = np.array([tvec[0], tvec[1], tvec[2]], dtype=np.float32).reshape(3, 1)
                ref_pos = np.array([
                    self.ref_transform.transform.translation.x,
                    self.ref_transform.transform.translation.y,
                    self.ref_transform.transform.translation.z
                ], dtype=np.float32).reshape(3, 1)
                
                relative_pos = agent_pos - ref_pos
                
                # Create Odometry message
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = "marker_0_frame"
                odom_msg.child_frame_id = f"marker_{marker_id}_frame"
                
                # Set the pose (relative to marker 0)
                odom_msg.pose.pose.position.x = float(relative_pos[0])
                odom_msg.pose.pose.position.y = float(relative_pos[1])
                odom_msg.pose.pose.position.z = float(relative_pos[2])
                
                rot_matrix, _ = cv2.Rodrigues(rvec)
                quat = self.rotation_matrix_to_quaternion(rot_matrix)
                odom_msg.pose.pose.orientation.x = quat[0]
                odom_msg.pose.pose.orientation.y = quat[1]
                odom_msg.pose.pose.orientation.z = quat[2]
                odom_msg.pose.pose.orientation.w = quat[3]
                
                # Publish to the appropriate topic
                if marker_id == 4:
                    self.agent_0_odom_publisher.publish(odom_msg)
                    log_msg = f'Marker 4 odom (rel to 0): x={relative_pos[0][0]:.2f}, y={relative_pos[1][0]:.2f}, z={relative_pos[2][0]:.2f}'
                elif marker_id == 5:
                    self.agent_1_odom_publisher.publish(odom_msg)
                    log_msg = f'Marker 5 odom (rel to 0): x={relative_pos[0][0]:.2f}, y={relative_pos[1][0]:.2f}, z={relative_pos[2][0]:.2f}'
                
                self.get_logger().info(log_msg)
                
                if self.show_gui:
                    # Draw marker outline and info
                    color = (255, 0, 0) if marker_id == 4 else (0, 0, 255)
                    cv2.polylines(frame, [corners.astype(int)], True, color, 2)
                    text_pos = tuple(corners[0].astype(int))
                    cv2.putText(frame, f"ID: {marker_id}", text_pos,
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                    cv2.putText(frame, f"X: {relative_pos[0][0]:.2f}", (text_pos[0], text_pos[1]+20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                    cv2.putText(frame, f"Y: {relative_pos[1][0]:.2f}", (text_pos[0], text_pos[1]+40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        except Exception as e:
            self.get_logger().error(f"Error in process_agent_marker: {e}")

    def get_marker_object_points(self):
        """Return 3D points of marker corners in marker coordinate system."""
        marker_size = self.marker_length
        return np.array([
            [-marker_size / 2, -marker_size / 2, 0],
            [ marker_size / 2, -marker_size / 2, 0],
            [ marker_size / 2,  marker_size / 2, 0],
            [-marker_size / 2,  marker_size / 2, 0]
        ], dtype=np.float32)

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

    def destroy_node(self):
        self.cap.release()
        if self.show_gui:
            cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

"""
Goal GUI Node for ChoiRbot
- Displays a simple window for user to click and set goal positions
- Publishes clicked positions as PoseStamped messages to /external_goals
"""

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt5.QtCore import Qt

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_gui_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/external_goals', 10)

    def publish_goal(self, x, y):
        # Create and publish a PoseStamped message with the clicked coordinates
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0  # No rotation
        self.publisher.publish(msg)
        self.get_logger().info(f"Published goal: ({x:.2f}, {y:.2f})")

class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("ChoiRbot Goal Setter")
        self.setGeometry(100, 100, 600, 600)
        self.label = QLabel(self)
        self.label.setGeometry(0, 0, 600, 600)
        self.label.setStyleSheet("background-color: white;")
        self.scale = 0.05  # 1 pixel = 0.05 meters (example scaling)
        self.offset_x = 300  # Center of the window is (0,0) in map coordinates
        self.offset_y = 300

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            # Convert pixel coordinates to map coordinates
            x = (event.x() - self.offset_x) * self.scale
            y = (event.y() - self.offset_y) * self.scale
            self.ros_node.publish_goal(x, y)

def main():
    rclpy.init()
    ros_node = GoalPublisher()
    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()
    app.exec_()
    rclpy.shutdown()

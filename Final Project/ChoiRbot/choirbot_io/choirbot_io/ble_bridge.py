#!/usr/bin/env python3

"""
BLE Bridge Node for ChoiRbot
- Subscribes to a ROS 2 topic (e.g., /agent_0/cmd_vel) for velocity commands
- Connects to an Arduino (or similar) robot via BLE
- Sends velocity commands to the robot over BLE
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
from bleak import BleakClient

# Replace with your BLE device's MAC address and characteristic UUID
BLE_DEVICE_ADDRESS = "AA:BB:CC:DD:EE:FF"
BLE_CHARACTERISTIC_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"

class BLEBridge(Node):
    def __init__(self):
        super().__init__('ble_bridge')
        self.declare_parameter('cmd_topic', '/agent_0/cmd_vel')
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        # Start the asyncio event loop for BLE communication
        self.loop = asyncio.get_event_loop()
        self.ble_client = BleakClient(BLE_DEVICE_ADDRESS, loop=self.loop)
        self.connected = False

        # Subscribe to velocity commands
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_topic,
            self.cmd_callback,
            10
        )

        # Connect to BLE device asynchronously
        self.loop.run_until_complete(self.connect_ble())

        self.get_logger().info(f"BLEBridge node started. Listening on {self.cmd_topic}")

    async def connect_ble(self):
        try:
            await self.ble_client.connect()
            self.connected = await self.ble_client.is_connected()
            if self.connected:
                self.get_logger().info(f"Connected to BLE device at {BLE_DEVICE_ADDRESS}")
            else:
                self.get_logger().error("Failed to connect to BLE device.")
        except Exception as e:
            self.get_logger().error(f"BLE connection error: {e}")

    def cmd_callback(self, msg):
        """
        Callback for received Twist messages.
        Sends the velocity command to the BLE device.
        """
        if not self.connected:
            self.get_logger().warning("Not connected to BLE device. Command not sent.")
            return

        # Format the command string as needed by your Arduino firmware
        # Example: "vx,vy,omega\n"
        command = f"{msg.linear.x:.2f},{msg.linear.y:.2f},{msg.angular.z:.2f}\n"
        self.get_logger().debug(f"Sending command over BLE: {command.strip()}")

        # Send the command asynchronously
        asyncio.ensure_future(self.send_ble_command(command))

    async def send_ble_command(self, command):
        """
        Sends the command string to the BLE device.
        """
        try:
            await self.ble_client.write_gatt_char(BLE_CHARACTERISTIC_UUID, command.encode())
        except Exception as e:
            self.get_logger().error(f"Failed to send BLE command: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BLEBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Disconnect BLE client on shutdown
        if node.connected:
            node.loop.run_until_complete(node.ble_client.disconnect())
        node.destroy_node()
        rclpy.shutdown()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
from bleak import BleakClient

class BLEBridge(Node):
    def __init__(self):
        super().__init__('ble_bridge')
        self.declare_parameter('cmd_topic', '/agent_0/cmd_vel')
        self.declare_parameter('ble_address', 'AA:BB:CC:DD:EE:FF')
        self.declare_parameter('ble_uuid', '0000ffe1-0000-1000-8000-00805f9b34fb')

        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.ble_address = self.get_parameter('ble_address').get_parameter_value().string_value
        self.ble_uuid = self.get_parameter('ble_uuid').get_parameter_value().string_value

        self.loop = asyncio.get_event_loop()
        self.ble_client = BleakClient(self.ble_address, loop=self.loop)
        self.connected = False

        self.subscription = self.create_subscription(
            Twist,
            self.cmd_topic,
            self.cmd_callback,
            10
        )

        self.loop.run_until_complete(self.connect_ble())
        self.get_logger().info(f"BLEBridge node started. Listening on {self.cmd_topic}")

    async def connect_ble(self):
        try:
            await self.ble_client.connect()
            self.connected = await self.ble_client.is_connected()
            if self.connected:
                self.get_logger().info(f"Connected to BLE device at {self.ble_address}")
            else:
                self.get_logger().error("Failed to connect to BLE device.")
        except Exception as e:
            self.get_logger().error(f"BLE connection error: {e}")

    def cmd_callback(self, msg):
        if not self.connected:
            self.get_logger().warning("Not connected to BLE device. Command not sent.")
            return

        command = f"{msg.linear.x:.2f},{msg.linear.y:.2f},{msg.angular.z:.2f}\n"
        self.get_logger().debug(f"Sending command over BLE: {command.strip()}")
        asyncio.ensure_future(self.send_ble_command(command))

    async def send_ble_command(self, command):
        try:
            await self.ble_client.write_gatt_char(self.ble_uuid, command.encode())
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
        if node.connected:
            node.loop.run_until_complete(node.ble_client.disconnect())
        node.destroy_node()
        rclpy.shutdown()
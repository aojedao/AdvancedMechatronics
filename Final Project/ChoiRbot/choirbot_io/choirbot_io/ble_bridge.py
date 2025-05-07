import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import threading
from bleak import BleakClient
import time

class BLEBridge(Node):
    """
    ROS 2 Node that bridges Twist messages to a BLE device using Bleak.
    """
    def __init__(self):
        super().__init__('ble_bridge')

        # Declare and get parameters
        self.declare_parameter('cmd_topic', '/agent0/cmd_vel')
        self.declare_parameter('ble_address', 'AA:BB:CC:DD:EE:FF')
        self.declare_parameter('ble_uuid', '0000ffe1-0000-1000-8000-00805f9b34fb')

        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.ble_address = self.get_parameter('ble_address').get_parameter_value().string_value
        self.ble_uuid = self.get_parameter('ble_uuid').get_parameter_value().string_value

        if not self.cmd_topic or not self.ble_address or not self.ble_uuid:
            raise ValueError("cmd_topic, ble_address, and ble_uuid parameters must be set")

        # Set up asyncio event loop in a background thread
        self.loop = asyncio.new_event_loop()
        self.loop_thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.loop_thread.start()

        # BLE client and connection state
        self.ble_client = BleakClient(self.ble_address, loop=self.loop)
        self.connected = False

        # Subscribe to Twist messages
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_topic,
            self.cmd_callback,
            10
        )

        # Schedule BLE connection and monitoring
        asyncio.run_coroutine_threadsafe(self.connect_ble(), self.loop)
        asyncio.run_coroutine_threadsafe(self.monitor_connection(), self.loop)

        self.get_logger().info(f"BLEBridge node started. Listening on {self.cmd_topic}")

    async def connect_ble(self):
        """
        Attempt to connect to the BLE device.
        """
        try:
            await self.ble_client.connect()
            self.connected = await self.ble_client.is_connected()
            if self.connected:
                self.get_logger().info(f"Connected to BLE device at {self.ble_address}")
            else:
                self.get_logger().error("Failed to connect to BLE device.")
        except Exception as e:
            self.get_logger().error(f"BLE connection error: {e}")
            await self.reconnect_ble()

    async def reconnect_ble(self):
        """
        Attempt to reconnect to the BLE device until successful.
        """
        while not self.connected:
            self.get_logger().warning("Attempting to reconnect to BLE device...")
            try:
                await self.ble_client.connect()
                self.connected = await self.ble_client.is_connected()
                if self.connected:
                    self.get_logger().info(f"Reconnected to BLE device at {self.ble_address}")
                    break
            except Exception as e:
                self.get_logger().error(f"Reconnection failed: {e}")
            await asyncio.sleep(5)  # Retry every 5 seconds

    async def monitor_connection(self):
        """
        Monitor BLE connection and attempt reconnection if lost.
        """
        while rclpy.ok():
            if not self.connected:
                self.get_logger().warning("BLE connection lost. Attempting to reconnect...")
                await self.reconnect_ble()
            await asyncio.sleep(2)  # Check connection status every 2 seconds

    # Add these global variables at the class level
    MESSAGE_RATE_LIMIT = 1.0  # seconds between messages (adjust as needed)
    last_sent_time = 0.0

    def cmd_callback(self, msg):
        """
        Callback for received Twist messages. Sends command over BLE.
        """
        command = f"{msg.linear.x:.2f},{msg.linear.y:.2f},{msg.angular.z:.2f}\n"
        special_command = "0.00,0.00,0.00\n"

        now = time.time()
        should_send = False

        if command == special_command:
            should_send = True
        elif now - BLEBridge.last_sent_time >= BLEBridge.MESSAGE_RATE_LIMIT:
            should_send = True
            BLEBridge.last_sent_time = now

        if should_send:
            self.get_logger().info(
                f"[{self.cmd_topic}] Sending command over BLE: {command.strip()}"
            )
            asyncio.run_coroutine_threadsafe(self.send_ble_command(command), self.loop)
        else:
            self.get_logger().info(
                f"[{self.cmd_topic}] Rate limit: Command not sent: {command.strip()}"
            )

    async def send_ble_command(self, command):
        """
        Send a command string to the BLE device.
        """
        try:
            await self.ble_client.write_gatt_char(self.ble_uuid, command.encode())
        except Exception as e:
            self.get_logger().error(f"[{self.cmd_topic}] Failed to send BLE command: {e}")

    def shutdown(self):
        """
        Cleanly shutdown BLE connection and event loop.
        """
        if self.connected:
            fut = asyncio.run_coroutine_threadsafe(self.ble_client.disconnect(), self.loop)
            try:
                fut.result(timeout=5)
                self.get_logger().info(f"Disconnected from BLE device at {self.ble_address}")
            except Exception as e:
                self.get_logger().error(f"Error during BLE disconnect: {e}")
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.loop_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = BLEBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

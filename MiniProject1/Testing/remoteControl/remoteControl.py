import asyncio
import tkinter as tk
from bleak import BleakClient, BleakScanner
import threading

# UUIDs (Replace these with the actual ones from your Arduino code)
ENC_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
POSE_CHAR_UUID = "19B10003-E8F2-537E-4F6C-D104768A1214"
WASD_CHAR_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"

client = None
loop = asyncio.new_event_loop()  # Create a new event loop for asyncio

def update_pose_label(pose):
    """Update the pose label in the GUI."""
    pose_label.config(text=f"Current Pose: {pose}")
    print(f"Updated pose label: {pose}")  # Debugging

async def find_device():
    """Scan for the BLE device."""
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover()
    for device in devices:
        print(f"Found device: {device.name} ({device.address})")
        if device.name and "Robot-BLE-DeadReckon" in device.name:
            print(f"Target device found: {device.name} ({device.address})")
            return device.address
    print("Arduino Nano 33 BLE not found.")
    return None

async def connect_to_robot(address):
    """Connect to the BLE device."""
    global client
    print(f"Attempting to connect to {address}...")
    client = BleakClient(address, loop=loop)
    try:
        await client.connect()
        print("Connected to robot.")
        await update_pose()  # Update pose after connecting
    except Exception as e:
        print(f"Failed to connect: {e}")

async def write_command(command):
    """Send a command to the BLE device."""
    if client and client.is_connected:
        try:
            await client.write_gatt_char(WASD_CHAR_UUID, bytearray(command, "utf-8"), response=True)
            print(f"Sent command: {command}")
            await update_pose()  # Update pose after sending command
        except Exception as e:
            print(f"Error sending command {command}: {e}")
    else:
        print("Client not connected.")

async def update_pose():
    """Read the pose from the BLE device and update the GUI."""
    if client and client.is_connected:
        try:
            pose_data = await client.read_gatt_char(POSE_CHAR_UUID)
            pose = pose_data.decode("utf-8")  # Decode the pose data
            print(f"Read pose: {pose}")  # Debugging
            update_pose_label(pose)  # Update the GUI label
        except Exception as e:
            print(f"Error reading pose: {e}")
    else:
        print("Client not connected.")

def send_command(command):
    """Schedule a command to be sent to the BLE device."""
    if client and client.is_connected:
        asyncio.run_coroutine_threadsafe(write_command(command), loop)

def start_ble_connection():
    """Start the BLE connection process."""
    asyncio.run_coroutine_threadsafe(main(), loop)

async def main():
    """Main function to find and connect to the BLE device."""
    address = await find_device()
    if address:
        await connect_to_robot(address)

def run_asyncio_loop():
    """Run the asyncio event loop in a separate thread."""
    asyncio.set_event_loop(loop)
    loop.run_forever()

# GUI Setup
root = tk.Tk()
root.title("BLE Robot Control")

frame = tk.Frame(root)
frame.pack()

pose_label = tk.Label(root, text="Current Pose: N/A", font=("Arial", 14))
pose_label.pack()

btn_forward = tk.Button(frame, text="↑ Forward", command=lambda: send_command('W'), height=2, width=10)
btn_backward = tk.Button(frame, text="↓ Backward", command=lambda: send_command('S'), height=2, width=10)
btn_left = tk.Button(frame, text="← Left", command=lambda: send_command('A'), height=2, width=10)
btn_right = tk.Button(frame, text="→ Right", command=lambda: send_command('D'), height=2, width=10)

btn_forward.grid(row=0, column=1)
btn_left.grid(row=1, column=0)
btn_right.grid(row=1, column=2)
btn_backward.grid(row=2, column=1)

btn_connect = tk.Button(root, text="Connect", command=start_ble_connection, height=2, width=10)
btn_connect.pack()

# Start the asyncio event loop in a separate thread
threading.Thread(target=run_asyncio_loop, daemon=True).start()

# Start the Tkinter main loop
root.mainloop()
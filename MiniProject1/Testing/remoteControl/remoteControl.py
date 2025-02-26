import asyncio
import tkinter as tk
from bleak import BleakClient, BleakScanner

# UUIDs (Replace these with the actual ones from your Arduino code)
ENC_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
POSE_CHAR_UUID = "19B10003-E8F2-537E-4F6C-D104768A1214"
WASD_CHAR_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"

global client
client = None

def update_pose_label(pose):
    pose_label.config(text=f"Current Pose: {pose}")

async def find_device():
    devices = await BleakScanner.discover()
    for device in devices:
        if device.name:
            print(device.name)
        if device.name and "Robot-BLE-DeadReckon" in device.name:
            print(f"Found device: {device.name} ({device.address})")
            return device.address
    print("Arduino Nano 33 BLE not found.")
    return None

async def connect_to_robot(address):
    global client
    client = BleakClient(address)
    await client.connect()
    print("Connected to robot.")
    await update_pose()

def send_command(command):
    if client and client.is_connected:
        asyncio.run(write_command(command))

async def write_command(command):
    try:
        await client.write_gatt_char(WASD_CHAR_UUID, bytearray(command, "utf-8"), response=True)
        print(f"Sent command: {command}")
        await update_pose()
    except Exception as e:
        print(f"Error sending command {command}: {e}")

async def update_pose():
    try:
        pose = await client.read_gatt_char(POSE_CHAR_UUID)
        update_pose_label(pose.decode())
    except Exception as e:
        print(f"Error reading pose: {e}")

def start_ble_connection():
    asyncio.run(main())

async def main():
    address = await find_device()
    if address:
        await connect_to_robot(address)

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

root.mainloop()

import asyncio
import tkinter as tk
from bleak import BleakClient, BleakScanner
import threading

# UUIDs (Replace these with the actual ones from your Arduino code)
ENC_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
POSE_CHAR_UUID = "19B10003-E8F2-537E-4F6C-D104768A1214"
WASD_CHAR_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"
ROT_TIME_UINT_UUID = "19B10004-E8F2-537E-4F6C-D104768A1214"  # Replace with actual UUID
LIN_TIME_UINT_UUID = "19B10005-E8F2-537E-4F6C-D104768A1214"  # Replace with actual UUID
MOTORSPEED_UINT_UUID = "19B10006-E8F2-537E-4F6C-D104768A1214"  # Replace with actual UUID

client = None
loop = asyncio.new_event_loop()  # Create a new event loop for asyncio

def update_pose_label(pose):
    """Update the pose label in the GUI."""
    pose_label.config(text=f"Current Pose: {pose}")
    print(f"Updated pose label: {pose}")  # Debugging

async def find_device():
    """Scan for the BLE device."""
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=10.0)  # Increase scan time
    for device in devices:
        device_name = device.name if device.name else "Unknown"
        print(f"Found device: {device_name} ({device.address})")
        # Check both possible names from Arduino
        if device_name in ["Robot-BLE-DeadReckon", "Team3FirstRobot"]:
            print(f"Target device found: {device_name} ({device.address})")
            return device.address
        # Debug: Print advertisement data
        print(f"Device details: {device.details}")
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
        
        entry1.delete(0, tk.END)
        entry2.delete(0, tk.END)
        entry3.delete(0, tk.END)
        
        ent1 = await client.read_gatt_char(ROT_TIME_UINT_UUID)
        ent2 = await client.read_gatt_char(LIN_TIME_UINT_UUID)
        ent3 = await client.read_gatt_char(MOTORSPEED_UINT_UUID)
        
        entry1.insert(0,int.from_bytes(ent1, byteorder='little'))
        entry2.insert(0,int.from_bytes(ent2, byteorder='little'))
        entry3.insert(0,int.from_bytes(ent3, byteorder='little'))
        
        # entry1.insert(0,100)
        # entry2.insert(0,610)
        # entry3.insert(0,610)
        
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
    

async def set_characteristic(char_uuid, value):
    """Write a value to a specific BLE characteristic."""
    if client and client.is_connected:
        try:
            int_value = int(value)
            byte_data = int_value.to_bytes(2, byteorder="little")  # 2 bytes
            await client.write_gatt_char(char_uuid, byte_data, response=True)
            print(f"Set characteristic {char_uuid} to: {value}")
        except Exception as e:
            print(f"Error setting characteristic {char_uuid}: {e}")
    else:
        print("Client not connected.")

def on_set_click():
    """Handle the 'Set' button click."""
    value1 = entry1.get()
    value2 = entry2.get()
    value3 = entry3.get()

    if value1:
        asyncio.run_coroutine_threadsafe(set_characteristic(ROT_TIME_UINT_UUID, value1), loop)
    if value2:
        asyncio.run_coroutine_threadsafe(set_characteristic(LIN_TIME_UINT_UUID, value2), loop)
    if value3:
        asyncio.run_coroutine_threadsafe(set_characteristic(MOTORSPEED_UINT_UUID, value3), loop)

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
window = tk.Tk()
window.title("BLE Robot Control")

# Pose Label
pose_label = tk.Label(window, text="Current Pose: N/A", font=("Arial", 14))
pose_label.pack(pady=10)

# Movement Buttons
frame = tk.Frame(window)
frame.pack(pady=10)

btn_forward = tk.Button(frame, text="↑ Forward", command=lambda: send_command('W'), height=2, width=10)
btn_backward = tk.Button(frame, text="↓ Backward", command=lambda: send_command('S'), height=2, width=10)
btn_left = tk.Button(frame, text="← Left", command=lambda: send_command('A'), height=2, width=10)
btn_right = tk.Button(frame, text="→ Right", command=lambda: send_command('D'), height=2, width=10)

btn_forward.grid(row=0, column=1)
btn_left.grid(row=1, column=0)
btn_right.grid(row=1, column=2)
btn_backward.grid(row=2, column=1)

# Editable Fields and Set Button
entry_frame = tk.Frame(window)
entry_frame.pack(pady=10)

label1 = tk.Label(entry_frame, text="rotation time (ms):")
label1.grid(row=0, column=0, padx=5, pady=5)
entry1 = tk.Entry(entry_frame)
entry1.grid(row=0, column=1, padx=5, pady=5)

label2 = tk.Label(entry_frame, text="linear time (ms):")
label2.grid(row=1, column=0, padx=5, pady=5)
entry2 = tk.Entry(entry_frame)
entry2.grid(row=1, column=1, padx=5, pady=5)

label3 = tk.Label(entry_frame, text="motor speed:")
label3.grid(row=2, column=0, padx=5, pady=5)
entry3 = tk.Entry(entry_frame)
entry3.grid(row=2, column=1, padx=5, pady=5)

btn_set = tk.Button(entry_frame, text="Set", command=on_set_click, height=2, width=10)
btn_set.grid(row=3, column=0, columnspan=2, pady=10)

# Connect Button
btn_connect = tk.Button(window, text="Connect", command=start_ble_connection, height=2, width=10)
btn_connect.pack(pady=10)

# Connect Reset position button
btn_right = tk.Button(window, text="Reset Location", command=lambda: send_command('Q'), height=2, width=10)
btn_right.pack(pady=10)

# Start the asyncio event loop in a separate thread
threading.Thread(target=run_asyncio_loop, daemon=True).start()

# Start the Tkinter main loop
window.mainloop()
import asyncio
import tkinter as tk
from bleak import BleakClient, BleakScanner
import threading
import ast

import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

# Constants
WHEEL_RADIUS = 0.0325  # meters
WHEEL_DISTANCE = 0.145  # meters
TICKS_PER_REVOLUTION = 937  # encoder ticks per wheel revolution
ENCODER_RESOLUTION = 2 * np.pi / TICKS_PER_REVOLUTION  # radians per tick
dt = 0.01  # time step in seconds (adjust based on your sensor update rate)

# Initialize EKF
#changing to only have [x,y,theta]
ekf = ExtendedKalmanFilter(dim_x=3, dim_z=1)
ekf.x = np.array([0., 0., 0.]) # Initial state: [x, y, theta, v, w]
ekf.P *= 1.0  # Covariance matrix
ekf.Q = np.diag([0.01, 0.01, 0.001])  # Process noise
ekf.R = np.diag([0.1])  # Measurement noise

def f(x, u):
    """
    Process model: predicts the new state given the current state and control input.
    State: x = [x, y, theta]
    Control input: u = [v, w] where
       v: linear velocity (m/s)
       w: angular velocity (rad/s)
       
    Equations:
       x_new = x + v * cos(theta) * dt
       y_new = y + v * sin(theta) * dt
       theta_new = theta + w * dt
    """
    theta = x[2]
    v, w = u
    x_new = x[0] + v * np.cos(theta) * dt
    y_new = x[1] + v * np.sin(theta) * dt
    theta_new = theta + w * dt
    return np.array([x_new, y_new, theta_new])


def F_jacobian(x, u):
    """
    Jacobian of the process model f with respect to the state x.
    Partial derivatives of f with respect to x:
       ∂f/∂x = [[1, 0, -v*sin(theta)*dt],
                 [0, 1,  v*cos(theta)*dt],
                 [0, 0, 1]]
    """
    theta = x[2]
    v, _ = u
    return np.array([
        [1, 0, -v * np.sin(theta) * dt],
        [0, 1,  v * np.cos(theta) * dt],
        [0, 0, 1]
    ])

def h(x):
    """
    Measurement model: we measure the robot's orientation.
    Here, we assume the gyro provides an integrated heading measurement:
       h(x) = theta.
    """
    return np.array([x[2]])

def H_jacobian(x):
    """
    Jacobian of the measurement model h with respect to the state.
    Since h(x) = theta, the derivative is [0, 0, 1].
    """
    return np.array([[0, 0, 1]])

last_encoder_value_r = 0
last_encoder_value_l = 0

def normalize_angle(angle):
    """Normalize angle to be within [-pi, pi]."""
    return (angle + np.pi) % (2 * np.pi) - np.pi

# Function to process incoming sensor data
def process_sensor_data(data):
    global last_encoder_value_r, last_encoder_value_l

    ticks_r = data["rightWheelCount"]
    ticks_l = data["leftWheelCount"]
    gz_dps = data["gz"]  # in degrees per second

    # ✅ Convert gyro from deg/s → rad/s
    gz_rad = gz_dps * np.pi / 180.0

    delta_ticks_r = ticks_r - last_encoder_value_r
    delta_ticks_l = ticks_l - last_encoder_value_l

    d_right = (delta_ticks_r / TICKS_PER_REVOLUTION) * (2 * np.pi * WHEEL_RADIUS)
    d_left  = (delta_ticks_l  / TICKS_PER_REVOLUTION) * (2 * np.pi * WHEEL_RADIUS)

    v = (d_right + d_left) / (2 * dt)
    w = (d_right - d_left) / (WHEEL_DISTANCE * dt)
    u = np.array([v, w])

    # Predict step
    ekf.F = F_jacobian(ekf.x, u)
    ekf.x = f(ekf.x, u)
    ekf.P = ekf.F @ ekf.P @ ekf.F.T + ekf.Q

    # Measurement: use integrated gyro to estimate theta
    theta_meas = ekf.x[2] + gz_rad * dt
    theta_meas = normalize_angle(theta_meas)  # keep θ bounded
    z = np.array([theta_meas])

    # Update step
    ekf.H = H_jacobian(ekf.x)
    y = z - h(ekf.x)
    y[0] = normalize_angle(y[0])  # residual also normalized
    S = ekf.H @ ekf.P @ ekf.H.T + ekf.R
    K = ekf.P @ ekf.H.T @ np.linalg.inv(S)

    ekf.x += K @ y
    ekf.x[2] = normalize_angle(ekf.x[2])  # normalize state θ
    ekf.P = (np.eye(3) - K @ ekf.H) @ ekf.P

    last_encoder_value_r = ticks_r
    last_encoder_value_l = ticks_l

    return ekf.x


robot_name = "Robot-BLE-IMU"
robot_name2 = "Team3SecondRobot"

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
    x, y, theta = pose
    pose_label.config(text=f"Current Pose: x: {x:.3f}, y: {y:.3f}, θ: {theta:.3f}")
    print(f"Updated pose label: x: {x:.3f}, y: {y:.3f}, θ: {theta:.3f}")  # Debugging

def notification_handler(sender, data):
    """Handle notifications from the BLE device."""
    try:
        pose = data.decode("utf-8")  # Decode the pose data
        print(f"Notification received from {sender}: {pose}")  # Debugging
        # TODO Insert string parsing and data fusion for pose calculation.
        state = process_sensor_data(ast.literal_eval(pose))
        print("Updated state:", state)
        print(f"Read pose: {pose}")  # Debugging
        print(f"Pose KMF: {state}")  # Debugging
        
        
        # Update the GUI label with the new pose
        
        update_pose_label(state)  # Update the GUI label
    except Exception as e:
        print(f"Error handling notification: {e}")
        
        
async def find_device():
    """Scan for the BLE device."""
    print("Scanning for BLE devices...")
    devices = await BleakScanner.discover(timeout=10.0)  # Increase scan time
    for device in devices:
        device_name = device.name if device.name else "Unknown"
        print(f"Found device: {device_name} ({device.address})")
        # Check both possible names from Arduino
        if device_name in [robot_name, robot_name2]:
            print(f"Target device found: {device_name} ({device.address})")
            return device.address
        # Debug: Print advertisement data
        print(f"Device details: {device.details}")
    print("ESP32 BLE not found.")
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
        
        await client.start_notify(POSE_CHAR_UUID, notification_handler)
        print("Subscribed to pose notifications.")
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
    
    
def restart_location():
    """Restart the robot's location."""
    if client and client.is_connected:
        #send_command('Q')
        asyncio.run_coroutine_threadsafe(update_pose(), loop)
        ekf.x = np.array([0., 0., 0.])  # Reset EKF state
        print("EKF state reset.")
        print("Restarting location...")
        #wait 1 second
        asyncio.run_coroutine_threadsafe(asyncio.sleep(1), loop)
        print("Location restarted.")
    else:
        print("Client not connected.")

async def disconnect_from_robot():
    """Disconnect from the BLE device."""
    global client
    if client and client.is_connected:
        try:
            await client.stop_notify(POSE_CHAR_UUID)
            await client.disconnect()
            print("Disconnected from robot.")
        except Exception as e:
            print(f"Error during disconnect: {e}")

async def set_characteristic(char_uuid, value):
    """Write a value to a specific BLE characteristic."""
    if client and client.is_connected:
        try:
            await client.write_gatt_char(char_uuid, value.encode('utf-8'), response=True)
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
btn_right = tk.Button(window, text="Reset Location", command=restart_location, height=2, width=10)
btn_right.pack(pady=10)

# Start the asyncio event loop in a separate thread
threading.Thread(target=run_asyncio_loop, daemon=True).start()

# Start the Tkinter main loop
window.mainloop()
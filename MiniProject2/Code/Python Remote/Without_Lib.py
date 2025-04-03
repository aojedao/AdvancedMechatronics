import asyncio
import tkinter as tk
from bleak import BleakClient, BleakScanner
import threading
import ast
import math
import numpy as np
from filterpy.kalman import ExtendedKalmanFilter

# Constants
wheel_radius = 0.0325  # meters
wheel_base = 0.145  # meters
ticks_per_rev = 937  # encoder ticks per wheel revolution
distance_per_tick = 2 * np.pi / ticks_per_rev  # radians per tick
dt = 0.01  # time step in seconds (adjust based on your sensor update rate)

# EKF state: [x, y, theta, v, w]  
x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])    # initial state
P = np.diag([0.1, 0.1, 1.0, 1.0, 1.0])     # initial covariance guess

# Noise covariances (tuning required):

Q = np.diag([1e-4, 1e-4, 1e-3, 5e-2, 5e-2])   # Process noise covariance Q// (very small for x,y,θ; higher for v,w to allow changes) chatgpt asks to keepthis high


R = np.diag([ (0.05)**2,  (0.5)**2 ])   # Measurement noise covariance R //deepseek ->  (assume 0.05 m/s std dev for v_meas, 0.5 °/s for w_meas as an example)


# Previous encoder readings to compute differences
prev_left_ticks = None
prev_right_ticks = None

def ekf_update(data):
    """Perform one EKF predict-update cycle using encoder ticks and gyro z-rate."""
    global x, P, prev_left_ticks, prev_right_ticks
    left_ticks =     data["leftWheelCount"]
    right_ticks = data["rightWheelCount"]
    gyro_z = data["gz"]

    # if it is the first time then it should be the first value that we are getting
    if prev_left_ticks is None:
        prev_left_ticks = left_ticks
        prev_right_ticks = right_ticks
        return x  #return x, prev we were trying to play with values when ticks was 0 and that is why the error was increasing I guess. Not sure

    
    #These are the steps we need to follow to get the EKF
    # 1. Compute odometry (encoder-based) measurements
    # 2. EKF Predict Step
    # 3. EKF Update Step



    '''.............STEP 2...................'''
    # Calculate tick differences
    delta_left = (left_ticks - prev_left_ticks) * distance_per_tick
    delta_right = (right_ticks - prev_right_ticks) * distance_per_tick
    prev_left_ticks = left_ticks
    prev_right_ticks = right_ticks

    # Measured linear and angular velocities
    v_meas = (delta_right + delta_left) / (2 * dt)   # m/s
    # Angular change (rad) from encoders:
    delta_theta_enc = (delta_right - delta_left) / wheel_base  # in radians
    w_meas_enc = delta_theta_enc / dt                        # rad/s from encoders
    
    # Gyro measurement is given in deg/s (already angular velocity):
    w_meas_gyro = gyro_z   # deg/s 
    # Choose w measurement from gyro (more reliable), convert enc w to degrees for info
    w_meas = w_meas_gyro

    '''.............STEP 2...................'''
    # Unpack state for readability
    x_k, y_k, theta_k_deg, v_k, w_k = x
    theta_k = math.radians(theta_k_deg)  # convert degrees to radians for calc

    # State prediction using motion model
    x_pred = np.empty_like(x) #creates an array similar to x
    
    x_pred[0] = x_k + v_k * math.cos(theta_k) * dt #the math was done by wolfram
    x_pred[1] = y_k + v_k * math.sin(theta_k) * dt #the math was done by wolfram
    x_pred[2] = theta_k_deg + w_k * dt           # theta in degrees ,,,,,,the math was done by wolfram
    # Normalize theta to [-180,180) or [0,360) as needed
    x_pred[2] = ((x_pred[2] + 180) % 360) - 180   # constant velocity model................the math was done by wolfram
    x_pred[3] = v_k   
    x_pred[4] = w_k   # constant angular rate model............the math was done by wolfram

    # Jacobian F computation
    F = np.eye(5)
    # Partial derivatives for x, y w.rt theta, v
    F[0,2] = -v_k * math.sin(theta_k) * dt * (math.pi/180.0)   # note: dθ (rad)/dθ (deg) = pi/180 ............the math was done by wolfram
    F[0,3] =  math.cos(theta_k) * dt
    F[1,2] =  v_k * math.cos(theta_k) * dt * (math.pi/180.0) #............the math was done by wolfram
    F[1,3] =  math.sin(theta_k) * dt #............the math was done by wolfram
    # Partial derivative for theta w.rt w (theta, w in deg units)
    F[2,4] = dt
    # (Derivatives for v_k -> v_pred and w_k -> w_pred are 1 on the diagonal, already set)

    # Covariance prediction
    P_pred = F.dot(P).dot(F.T) + Q

    '''.............STEP 3...................'''
    # Measurement vector
    z = np.array([v_meas, w_meas])
    # Predicted measurement from predicted state
    z_pred = np.array([ x_pred[3], x_pred[4] ])   # [v_pred, w_pred]
    # Measurement residual
    y_res = z - z_pred

    # Measurement Jacobian H (constant in this case)
    H = np.array([
        [0, 0, 0, 1, 0],   # partials of v (measured) w.rt [x,y,theta,v,w]
        [0, 0, 0, 0, 1]    # partials of w (measured) w.rt [x,y,theta,v,w]
    ])

    # Innovation covariance and Kalman gain
    S = H.dot(P_pred).dot(H.T) + R
    K = P_pred.dot(H.T).dot(np.linalg.inv(S))

    # State update 
    x_new = x_pred + K.dot(y_res)
    # Ensure theta stays normalized
    x_new[2] = ((x_new[2] + 180) % 360) - 180

    # Covariance update (Joseph form can be used for numerical stability)
    I = np.eye(5)
    P_new = (I - K.dot(H)).dot(P_pred)

    # Save updated state and covariance
    x = x_new
    P = P_new

    # Optionally, enforce symmetry on P_new (small numerical errors)
    P = 0.5 * (P + P.T)

    return x  # return the updated state estimate



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
    pose_label.config(text=f"Current Pose: {pose}")
    print(f"Updated pose label: {pose}")  # Debugging

def notification_handler(sender, data):
    """Handle notifications from the BLE device."""
    try:
        pose = data.decode("utf-8")  # Decode the pose data
        print(f"Notification received from {sender}: {pose}")  # Debugging
        # TODO Insert string parsing and data fusion for pose calculation.
        state = ekf_update(ast.literal_eval(pose))
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
btn_right = tk.Button(window, text="Reset Location", command=lambda: send_command('Q'), height=2, width=10)
btn_right.pack(pady=10)

# Start the asyncio event loop in a separate thread
threading.Thread(target=run_asyncio_loop, daemon=True).start()

# Start the Tkinter main loop
window.mainloop()
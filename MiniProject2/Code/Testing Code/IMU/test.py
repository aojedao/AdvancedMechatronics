import serial
import re
import matplotlib.pyplot as plt
from collections import deque
from matplotlib.animation import FuncAnimation
import numpy as np

# Initialize data storage
max_points = 100
data_points = deque(maxlen=max_points)  # Stores complete sets of measurements

# Set up the figure
plt.style.use('ggplot')
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10))
fig.suptitle('IMU Real-time Data Visualization', fontsize=14)

# Initialize empty plots
def init_plots():
    for ax in (ax1, ax2, ax3):
        ax.clear()
        ax.grid(True)
    
    # Accelerometer plot
    ax1.set_ylim(-2, 2)
    ax1.set_ylabel('Acceleration (g)')
    accel_lines = [
        ax1.plot([], [], 'r-', label='X')[0],
        ax1.plot([], [], 'g-', label='Y')[0],
        ax1.plot([], [], 'b-', label='Z')[0]
    ]
    ax1.legend()
    
    # Gyroscope plot
    ax2.set_ylim(-300, 300)
    ax2.set_ylabel('Rotation Rate (°/s)')
    gyro_lines = [
        ax2.plot([], [], 'c-', label='X')[0],
        ax2.plot([], [], 'm-', label='Y')[0],
        ax2.plot([], [], 'y-', label='Z')[0]
    ]
    ax2.legend()
    
    # Orientation plot
    ax3.set_ylim(-200, 200)
    ax3.set_ylabel('Orientation Rate (°/s)')
    ax3.set_xlabel('Time (samples)')
    angle_lines = [
        ax3.plot([], [], 'r--', label='Roll')[0],
        ax3.plot([], [], 'g--', label='Pitch')[0],
        ax3.plot([], [], 'b--', label='Yaw')[0]
    ]
    ax3.legend()
    
    return accel_lines + gyro_lines + angle_lines

lines = init_plots()

# Data structure to hold current frame
current_frame = {
    'accel': None,
    'gyro': None,
    'angles': None,
    'time': 0
}

def update(frame):
    global current_frame
    
    while ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            continue
        
        print(f"Raw: {line}")  # Debug print
        
        try:
            if line.startswith('Accel:'):
                match = re.search(r'X=\s*([-\d.]+)g\s+Y=\s*([-\d.]+)g\s+Z=\s*([-\d.]+)g', line)
                if match:
                    current_frame['accel'] = (
                        float(match.group(1)),
                        float(match.group(2)),
                        float(match.group(3))
                    )
            
            elif line.startswith('Gyro:'):
                match = re.search(r'X=\s*([-\d.]+)°/s\s+Y=\s*([-\d.]+)°/s\s+Z=\s*([-\d.]+)°/s', line)
                if match:
                    current_frame['gyro'] = (
                        float(match.group(1)),
                        float(match.group(2)),
                        float(match.group(3))
                    )
            
            elif line.startswith('Roll ='):
                match = re.search(r'Roll =\s*([-\d.]+)°/s\s+Pitch=\s*([-\d.]+)°/s\s+Yaw=\s*([-\d.]+)°/s', line)
                if match and current_frame['accel'] and current_frame['gyro']:
                    current_frame['angles'] = (
                        float(match.group(1)),
                        float(match.group(2)),
                        float(match.group(3))
                    )
                    current_frame['time'] = frame
                    data_points.append(current_frame.copy())
        
        except Exception as e:
            print(f"Error processing line: {e}")
            continue
    
    # Only update plots if we have data
    if data_points:
        times = [f['time'] for f in data_points]
        
        # Update accelerometer plot
        lines[0].set_data(times, [f['accel'][0] for f in data_points])
        lines[1].set_data(times, [f['accel'][1] for f in data_points])
        lines[2].set_data(times, [f['accel'][2] for f in data_points])
        
        # Update gyroscope plot
        lines[3].set_data(times, [f['gyro'][0] for f in data_points])
        lines[4].set_data(times, [f['gyro'][1] for f in data_points])
        lines[5].set_data(times, [f['gyro'][2] for f in data_points])
        
        # Update orientation plot
        lines[6].set_data(times, [f['angles'][0] for f in data_points])
        lines[7].set_data(times, [f['angles'][1] for f in data_points])
        lines[8].set_data(times, [f['angles'][2] for f in data_points])
        
        # Adjust view
        for ax in (ax1, ax2, ax3):
            ax.relim()
            ax.autoscale_view(scalex=True, scaley=False)
            if times:
                ax.set_xlim(max(0, times[-1] - max_points), max(max_points, times[-1]))
    
    return lines

# Open serial connection
try:
    ser = serial.Serial('COM17', 115200, timeout=1)
    print(f"Connected to {ser.name}")
    ser.reset_input_buffer()
    
    # Start animation
    ani = FuncAnimation(
        fig, update, init_func=lambda: lines,
        blit=True, interval=50, cache_frame_data=False
    )
    
    plt.tight_layout()
    plt.show()
    
except serial.SerialException as e:
    print(f"Error opening serial port: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed")
import time
import random

# Problem 2: Simulate sensor readings and log to a file
def simulate_sensor_readings():
    with open('sensor_readings.txt', 'w') as file:  # Open file to log readings
        start_time = time.time()  # Record start time
        print("Sensor simulation started. Press CTRL+C to exit.")
        print("Generating sensor readings...")
        while time.time() - start_time < 60:  # Run for 60 seconds
            print("...\n")
            sensor1 = random.randint(0, 25)  # Generate random integer for sensor 1
            sensor2 = round(random.uniform(0.00, 5.00), 2)  # Generate random float for sensor 2
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")  # Get current timestamp
            # Write to file
            file.write(f"{timestamp}, Sensor1: {sensor1}, Sensor2: {sensor2}\n")
            time.sleep(1)  # Wait for one second

# Call the function to execute
simulate_sensor_readings()

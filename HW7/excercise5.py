import RPi.GPIO as GPIO
import time

# Problem 5: Measure distance with PING sensor and control LED
def ping_sensor_logging():
    TRIG_PIN = 23
    ECHO_PIN = 24
    LED_PIN = 18

    GPIO.setmode(GPIO.BCM)
    GPIO.setup(TRIG_PIN, GPIO.OUT)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    GPIO.setup(LED_PIN, GPIO.OUT)

    try:
        while True:
            GPIO.output(TRIG_PIN, True)  # Send a pulse
            time.sleep(0.00001)  # Wait for 10 microseconds
            GPIO.output(TRIG_PIN, False)  # Stop pulse

            while GPIO.input(ECHO_PIN) == 0:  # Wait for echo start
                start_time = time.time()
            while GPIO.input(ECHO_PIN) == 1:  # Wait for echo end
                end_time = time.time()

            duration = end_time - start_time  # Calculate duration
            distance = duration * 17150  # Convert to cm

            with open('distance_log.txt', 'a') as file:  # Log to file
                file.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')}, Distance: {distance:.2f} cm\n")

            # LED control based on distance
            if distance < 20:  # If distance is less than 20 cm
                GPIO.output(LED_PIN, True)  # Turn LED ON
            else:
                GPIO.output(LED_PIN, False)  # Turn LED OFF

            time.sleep(2)  # Wait for 2 seconds before next measurement

    except KeyboardInterrupt:
        GPIO.cleanup()  # Clean up GPIO on CTRL+C exit

# Call the function to execute
ping_sensor_logging()

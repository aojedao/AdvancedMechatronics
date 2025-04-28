import time
import RPi.GPIO as GPIO

# Problem 3: Simulate traffic light with LEDs
def traffic_light_sequence():
    GPIO.setmode(GPIO.BCM)  # Use Broadcom pin-numbering scheme
    RED_PIN = 17
    YELLOW_PIN = 27
    GREEN_PIN = 22

    # Setup pins as output
    GPIO.setup(RED_PIN, GPIO.OUT)
    GPIO.setup(YELLOW_PIN, GPIO.OUT)
    GPIO.setup(GREEN_PIN, GPIO.OUT)
    
    GPIO.output(RED_PIN, GPIO.LOW)  # Initialize RED LED OFF
    GPIO.output(YELLOW_PIN, GPIO.LOW)  # Initialize YELLOW LED OFF
    GPIO.output(GREEN_PIN, GPIO.LOW)  # Initialize GREEN LED OFF
    
    print("Traffic light simulation started. Press CTRL+C to exit.")
    

    try:
        while True:
            print("Traffic light sequence:")
            print("RED -> YELLOW -> GREEN")
            # Simulate traffic light sequence
            print("RED light ON 5 seconds")
            GPIO.output(RED_PIN, GPIO.HIGH)  # Turn RED LED ON
            time.sleep(5)  # Wait 5 seconds
            GPIO.output(RED_PIN, GPIO.LOW)  # Turn RED LED OFF
            
            print("YELLOW light ON 2 seconds")
            GPIO.output(YELLOW_PIN, GPIO.HIGH)  # Turn YELLOW LED ON
            time.sleep(2)  # Wait 2 seconds
            GPIO.output(YELLOW_PIN, GPIO.LOW)  # Turn YELLOW LED OFF
            
            print("GREEN light ON 5 seconds")
            GPIO.output(GREEN_PIN, GPIO.HIGH)  # Turn GREEN LED ON
            time.sleep(5)  # Wait 5 seconds
            GPIO.output(GREEN_PIN, GPIO.LOW)  # Turn GREEN LED OFF

    except KeyboardInterrupt:
        GPIO.cleanup()  # Clean up GPIO on CTRL+C exit

# Call the function to execute
traffic_light_sequence()

import RPi.GPIO as GPIO
import time

# Problem 4: Control LED brightness with buttons
def led_brightness_control():
    GPIO.setmode(GPIO.BCM)
    LED_PIN = 18
    BUTTON1_PIN = 17  # Increases brightness
    BUTTON2_PIN = 27  # Decreases brightness
    
    duty_cycle = 0  # Initial duty cycle

    GPIO.setup(LED_PIN, GPIO.OUT)
    GPIO.setup(BUTTON1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(BUTTON2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    pwm = GPIO.PWM(LED_PIN, 100)  # Create PWM instance
    pwm.start(0)  # Start PWM with 0% duty cycle

    try:
        while True:
            if GPIO.input(BUTTON1_PIN) == GPIO.LOW:  # Button 1 pressed
                duty_cycle += 5  # Increase brightness
                print("Increasing")
                if duty_cycle > 100:
                    duty_cycle = 100
                pwm.ChangeDutyCycle(duty_cycle)
                time.sleep(0.1)

            if GPIO.input(BUTTON2_PIN) == GPIO.LOW:  # Button 2 pressed
                print("Decreasing")
                duty_cycle -= 5
                if duty_cycle < 0:
                    duty_cycle = 0
                pwm.ChangeDutyCycle(duty_cycle)
                time.sleep(0.1)

    except KeyboardInterrupt:
        pwm.stop()  # Stop PWM on CTRL+C exit
        GPIO.cleanup()  # Clean up GPIO

# Call the function to execute
led_brightness_control()

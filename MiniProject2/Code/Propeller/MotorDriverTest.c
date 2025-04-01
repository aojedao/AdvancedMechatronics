#include "simpletools.h"  
#include <math.h>          
#include "servo.h"
#include <propeller.h>

// Define Propeller pins for motor control
#define INA_PIN 2  // Adjust as needed
#define INB_PIN 1  // Adjust as needed
#define PWM_PIN 0  // Adjust as needed (PWM control not implemented in this basic test)

// Define constants for motor directions
#define FORWARD 1
#define BACKWARD 0

void moveMotor(int direction, int duration_ms);

int main() {
    // Simple test sequence: Forward for 2 seconds, then backward for 2 seconds

    print("Starting motor test...\n");

    print("Moving forward for 2 seconds...\n");
    moveMotor(FORWARD, 5000);
    pause(1000); // Small pause between directions

    print("Moving backward for 2 seconds...\n");
    moveMotor(BACKWARD, 4000);

    print("Motor test complete.\n");

    return 0;
}

void moveMotor(int direction, int duration_ms) {
    if (direction == FORWARD) {
        OUTA |= (1 << INA_PIN); // Set INA high for forward
        OUTA &= ~(1 << INB_PIN); // Set INB low
        DIRA |= (1 << INA_PIN) | (1 << INB_PIN) | (1 << PWM_PIN); // Set as outputs
        //OUTA |= (1 << PWM_PIN); // Enable motor power (assuming PWM pin enables)
        servo_speed(PWM_PIN,100);
        servo_speed(PWM_PIN,20000);
        //servo_speed(14, 0);
    } else if (direction == BACKWARD) {
        OUTA &= ~(1 << INA_PIN); // Set INA low
        OUTA |= (1 << INB_PIN); // Set INB high
        DIRA |= (1 << INA_PIN) | (1 << INB_PIN) | (1 << PWM_PIN); // Set as outputs
        OUTA |= (1 << PWM_PIN); // Enable motor power
        //servo_speed(PWM_PIN,1000);
    } else {
        print("Invalid direction specified.\n");
        return;
    }

    pause(duration_ms); // Run for the specified duration

    // Stop the motor
    OUTA &= ~((1 << INA_PIN) | (1 << INB_PIN) | (1 << PWM_PIN));
    DIRA |= (1 << INA_PIN) | (1 << INB_PIN) | (1 << PWM_PIN); // Ensure pins are still outputs
}

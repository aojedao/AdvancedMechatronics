/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools

int main()                                    // Main function
{
  // Add startup code here.

 
  while(1)
  {
    // Add main loop code here.
    
  }  
}
#include "simpletools.h"  // Core Propeller functions
#include "servo.h"        // For PWM control via servo_speed
#include "propeller.h"    // For direct hardware access (if needed beyond simpletools)
#include <math.h>         // For abs()

// --- Pin Definitions ---
// Adjust these pins according to your actual hardware wiring

// Motor Left
#define L_INA_PIN   4  // Left Motor Direction Input A
#define L_INB_PIN   3  // Left Motor Direction Input B
#define L_PWM_PIN   5  // Left Motor PWM Speed Control

// Motor Right
#define R_INA_PIN   1  // Right Motor Direction Input A
#define R_INB_PIN   2  // Right Motor Direction Input B
#define R_PWM_PIN   0  // Right Motor PWM Speed Control

// Encoder Left (Example Pins - Adjust as needed)
#define L_ENCA_PIN  8  // Left Encoder Phase A
#define L_ENCB_PIN  9  // Left Encoder Phase B

// Encoder Right (Example Pins - Adjust as needed)
#define R_ENCA_PIN  10 // Right Encoder Phase A
#define R_ENCB_PIN  11 // Right Encoder Phase B

// --- Constants ---
#define LEFT_MOTOR  0
#define RIGHT_MOTOR 1

#define MAX_PWM_VAL 10000 // Define max value for our speed input (-100 to 100)

// --- Global Variables ---
// Use 'volatile' if these will be updated by another Cog (e.g., encoder cog)
volatile long encoder_left_count = 0;
volatile long encoder_right_count = 0;

// --- Function Prototypes ---
void initializeMotors();
void initializeEncoders(); // Placeholder for encoder setup
void setMotorSpeed(int motor_idx, int speed);
void stopMotors();
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);  // Rotate in place left
void turnRight(int speed); // Rotate in place right

// Placeholder for encoder reading function (likely run in separate cogs)
// void encoder_reader_cog(void *par);

// --- Main Program ---
int main() {
    print("Initializing Robot...\n");
    initializeMotors();
    initializeEncoders(); // Initialize encoder pins (basic setup)

    print("Robot Ready. Starting command sequence...\n");

    // Example Command Sequence:
    print("Moving Forward (Speed 50) for 2 seconds...\n");
    moveForward(500);
    pause(2000);

    print("Turning Left (Speed 40) for 1.5 seconds...\n");
    turnLeft(400);
    pause(1500);

    print("Moving Backward (Speed 50) for 2 seconds...\n");
    moveBackward(500);
    pause(2000);

    print("Turning Right (Speed 40) for 1.5 seconds...\n");
    turnRight(400);
    pause(1500);

    print("Stopping Motors...\n");
    stopMotors();

    print("Command sequence complete.\n");

    // --- Placeholder for where you'd integrate velocity commands ---
    // Example: Read a command from serial or another input
    // int desired_speed = 75;
    // char command = 'F'; // Read from input
    //
    // switch(command) {
    //    case 'F': moveForward(desired_speed); break;
    //    case 'B': moveBackward(desired_speed); break;
    //    case 'L': turnLeft(desired_speed); break;
    //    case 'R': turnRight(desired_speed); break;
    //    case 'S': stopMotors(); break;
    // }
    // pause(100); // Example delay or control loop timing
    // --- End Placeholder ---


    return 0; // Note: In embedded systems, main often loops forever.
}

// --- Function Implementations ---

/**
 * @brief Initializes motor control pins as outputs.
 */
void initializeMotors() {
    // Set direction pins as outputs
    set_direction(L_INA_PIN, 1);
    set_direction(L_INB_PIN, 1);
    set_direction(R_INA_PIN, 1);
    set_direction(R_INB_PIN, 1);

    // PWM pins are typically handled by the servo library or PWM functions,
    // but ensure they are not driven HIGH/LOW unintentionally if needed.
    // Setting them as output initially is safe.
    set_direction(L_PWM_PIN, 1);
    set_direction(R_PWM_PIN, 1);

    // Ensure motors are stopped initially
    stopMotors();
    print("Motor pins initialized.\n");
}

/**
 * @brief Placeholder: Initializes encoder input pins.
 * Actual reading requires dedicated Cogs for accuracy.
 */
void initializeEncoders() {
    // Set encoder pins as inputs
    set_direction(L_ENCA_PIN, 0);
    set_direction(L_ENCB_PIN, 0);
    set_direction(R_ENCA_PIN, 0);
    set_direction(R_ENCB_PIN, 0);

    // --- IMPORTANT ---
    // Accurate encoder reading at higher speeds requires using Propeller counters
    // (CTRA/CTRB) configured in PHS A/QUAD mode, usually within a separate Cog.
    // Example (conceptual):
    //   cogstart(&encoder_reader_cog, NULL, stack, sizeof(stack));
    // The encoder_reader_cog() function would then continuously update
    // the global 'encoder_left_count' and 'encoder_right_count' variables.
    // Consult Propeller documentation/examples for 'cogstart' and counter configuration.
    print("Encoder pins set as inputs (Basic Setup - Full reading requires Cogs).\n");
}


/**
 * @brief Sets the speed and direction of a single motor.
 *
 * @param motor_idx LEFT_MOTOR (0) or RIGHT_MOTOR (1).
 * @param speed Desired speed from -100 (full backward) to 100 (full forward).
 * 0 means stop (coast).
 */
void setMotorSpeed(int motor_idx, int speed) {
    int ina_pin, inb_pin, pwm_pin;
    int ina_val, inb_val;
    int pwm_val;

    // Select pins based on motor index
    if (motor_idx == LEFT_MOTOR) {
        ina_pin = L_INA_PIN;
        inb_pin = L_INB_PIN;
        pwm_pin = L_PWM_PIN;
    } else if (motor_idx == RIGHT_MOTOR) {
        ina_pin = R_INA_PIN;
        inb_pin = R_INB_PIN;
        pwm_pin = R_PWM_PIN;
    } else {
        print("Error: Invalid motor index %d\n", motor_idx);
        return;
    }

    // Clamp speed to the valid range
    if (speed > MAX_PWM_VAL) speed = MAX_PWM_VAL;
    if (speed < -MAX_PWM_VAL) speed = -MAX_PWM_VAL;

    // Determine direction pin states and PWM value
    if (speed > 0) { // Forward
        ina_val = 1;
        inb_val = 0;
        pwm_val = speed;
    } else if (speed < 0) { // Backward
        ina_val = 0;
        inb_val = 1;
        pwm_val = abs(speed); // PWM value is always positive
    } else { // Stop (Coast)
        ina_val = 0; // Set both low for coasting (check your motor driver!)
        inb_val = 0; // Some drivers brake if both are high. Coast is usually safer.
        pwm_val = 0;
    }

    // Set direction pins
    set_output(ina_pin, ina_val);
    set_output(inb_pin, inb_val);

    // Set PWM speed using servo_speed
    // NOTE: servo_speed(pin, value) might need calibration.
    // Assuming 'value' 0-100 roughly corresponds to 0-100% duty cycle.
    // A value of 0 should disable the PWM output or set 0% duty.
    // Check simpletools/servo.h documentation if behavior is unexpected.
    // Some setups might need values like 1000-2000 or other ranges.
    // If pwm_val is 0, we want to ensure the servo library stops the pulse.
    // Using 0 with servo_speed *should* effectively stop the PWM signal.
    servo_speed(pwm_pin, pwm_val);

}

/**
 * @brief Stops both motors (sets speed to 0).
 */
void stopMotors() {
    setMotorSpeed(LEFT_MOTOR, 0);
    setMotorSpeed(RIGHT_MOTOR, 0);
    // Optional: Add a small delay to ensure commands are processed
    // pause(10);
}

/**
 * @brief Moves the robot forward at a given speed.
 * @param speed Speed percentage (0-100).
 */
void moveForward(int speed) {
    speed = abs(speed); // Ensure speed is positive
    if (speed > MAX_PWM_VAL) speed = MAX_PWM_VAL;
    setMotorSpeed(LEFT_MOTOR, speed);
    setMotorSpeed(RIGHT_MOTOR, speed);
}

/**
 * @brief Moves the robot backward at a given speed.
 * @param speed Speed percentage (0-100).
 */
void moveBackward(int speed) {
    speed = abs(speed); // Ensure speed is positive
    if (speed > MAX_PWM_VAL) speed = MAX_PWM_VAL;
    setMotorSpeed(LEFT_MOTOR, -speed); // Negative for backward
    setMotorSpeed(RIGHT_MOTOR, -speed); // Negative for backward
}

/**
 * @brief Rotates the robot left (counter-clockwise) in place.
 * @param speed Speed percentage (0-100) for rotation.
 */
void turnLeft(int speed) {
    speed = abs(speed); // Ensure speed is positive
    if (speed > MAX_PWM_VAL) speed = MAX_PWM_VAL;
    setMotorSpeed(LEFT_MOTOR, -speed); // Left motor backward
    setMotorSpeed(RIGHT_MOTOR, speed);  // Right motor forward
}

/**
 * @brief Rotates the robot right (clockwise) in place.
 * @param speed Speed percentage (0-100) for rotation.
 */
void turnRight(int speed) {
    speed = abs(speed); // Ensure speed is positive
    if (speed > MAX_PWM_VAL) speed = MAX_PWM_VAL;
    setMotorSpeed(LEFT_MOTOR, speed);   // Left motor forward
    setMotorSpeed(RIGHT_MOTOR, -speed); // Right motor backward
}

// --- Placeholder for Encoder Reading Cog ---
/*
void encoder_reader_cog(void *par) {
  // --- Cog-Local Variables ---
  int cog_id = cogid(); // Get the ID of this cog
  unsigned int last_state_L = 0;
  unsigned int last_state_R = 0;
  int delta_L, delta_R;

  print("Encoder Reader Cog %d Started.\n", cog_id);

  // Configure Propeller Counters for Quadrature Input (Example - Details Matter!)
  // This requires careful reading of the Propeller manual section on I/O Pins and Counters
  // Example using INA/INB pins (adjust pins as needed!)
  // FRQA = 1; // Set frequency A register (if needed)
  // PHSA = 0; // Clear phase A register
  // CTRA = (0b01010 << 26) | L_ENCA_PIN; // PHS A mode, Pin L_ENCA
  //
  // FRQB = 1;
  // PHSB = 0;
  // CTRB = (0b01010 << 26) | R_ENCA_PIN; // PHS A mode, Pin R_ENCA

  while(1) {
    // --- Method 1: Using Propeller Counters (Preferred) ---
    // Assuming CTRA/CTRB are configured for quadrature counting on ENCA/ENCB pins
    // delta_L = PHSA; // Read accumulated counts for Left
    // PHSA = 0;       // Reset counter accumulator for next interval
    // delta_R = PHSB; // Read accumulated counts for Right
    // PHSB = 0;       // Reset counter accumulator

    // --- Method 2: Basic Pin Monitoring (Less Accurate at High Speed) ---
    // Read current state of Encoder A & B pins for both motors
     unsigned int current_state_L = (input(L_ENCB_PIN) << 1) | input(L_ENCA_PIN);
     unsigned int current_state_R = (input(R_ENCB_PIN) << 1) | input(R_ENCA_PIN);
    // Decode state changes to increment/decrement count (complex logic needed)
    // ... Lookup table or state machine logic here ...
    // delta_L = calculate_delta(last_state_L, current_state_L);
    // delta_R = calculate_delta(last_state_R, current_state_R);
    // last_state_L = current_state_L;
    // last_state_R = current_state_R;
    // delta_L = 0; // Replace with actual calculation
    // delta_R = 0; // Replace with actual calculation

    // Update global counts (use lock if main cog also writes/reads complexly)
    // Using __atomic builtins or disabling interrupts briefly might be needed
    // for true atomicity if just incrementing/decrementing.
    // For simple accumulation, direct access to volatile might be okay initially.
    encoder_left_count += delta_L;
    encoder_right_count += delta_R;

    // Pause briefly to control update rate and yield CPU time
    pause(10); // Adjust timing based on required resolution and speed
  }
}

// --- Required if using cogstart in main ---
// unsigned int stack[40 + 25]; // Stack for the encoder cog (size depends on cog needs)

*/
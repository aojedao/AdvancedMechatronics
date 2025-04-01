#include "simpletools.h"  // Core Propeller functions & includes propeller.h implicitly
#include "servo.h"        // For PWM control via servo_speed
#include <math.h>         // For abs() in motor control
#include <stdint.h>       // For specific integer types like int32_t, uint8_t

// --- Pin Definitions ---

// Encoder pins (FROM YOUR PROVIDED CODE)
#define RIGHT_A 11  // Right Encoder Phase A
#define RIGHT_B 12  // Right Encoder Phase B
#define LEFT_A  9   // Left Encoder Phase A
#define LEFT_B  10  // Left Encoder Phase B

// Motor Control Pins (ADD THESE - ADJUST TO YOUR WIRING)
#define L_INA_PIN   2  // Left Motor Direction Input A
#define L_INB_PIN   3  // Left Motor Direction Input B
#define L_PWM_PIN   0  // Left Motor PWM Speed Control

#define R_INA_PIN   4  // Right Motor Direction Input A
#define R_INB_PIN   5  // Right Motor Direction Input B
#define R_PWM_PIN   1  // Right Motor PWM Speed Control

// --- Constants ---
#define LEFT_MOTOR  0
#define RIGHT_MOTOR 1
#define MAX_PWM_VAL 100 // Define max value for our speed input (-100 to 100)

// --- Global Variables ---
// Encoder counts - updated by encoderCog
// Using static volatile as per your findings for PropGCC compatibility
static volatile int32_t rightWheelCount = 0;
static volatile int32_t leftWheelCount = 0;

// Stack for the encoder cog
static unsigned int encoderStack[128*2]; // 128*2 = 256 longs = 1024 bytes

// Encoder state lookup table (FROM YOUR PROVIDED CODE)
//                           _______         _______
//               Pin1 ______|       |_______|       |______ Pin1
// negative <---         _______         _______         __      --> positive
//               Pin2 __|       |_______|       |_______|   Pin2
static const int8_t decoderLookup[16] = {
    0,  +1, -1, +2,  // 00xx (+2/-2 are transitions measured on pin1 edge only)
   -1,  0, -2, +1,  // 01xx
   +1,  -2,  0, -1,  // 10xx
   +2,  -1, +1,  0   // 11xx
};
// Index: (new B << 3 | new A << 2 | old B << 1 | old A << 0) - Matches table structure

// --- Function Prototypes ---
// Cog Function
void encoderCog(void *par);

// Motor Control Functions
void initializeMotors();
void setMotorSpeed(int motor_idx, int speed);
void stopMotors();
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);  // Rotate in place left
void turnRight(int speed); // Rotate in place right

// --- Cog Implementation ---

/**
 * @brief Cog responsible for reading quadrature encoders using pin monitoring
 * and a state lookup table. Updates global wheel count variables.
 * (Based on your provided code)
 */
void encoderCog(void *par) {
    uint8_t lastRight = 0; // Stores previous state of Right Encoder (B << 1 | A)
    uint8_t lastLeft = 0;  // Stores previous state of Left Encoder (B << 1 | A)

    // Initialize last states based on current inputs
    lastRight = (input(RIGHT_B) << 1) | input(RIGHT_A);
    lastLeft  = (input(LEFT_B) << 1) | input(LEFT_A);

    print("Encoder Cog Started. Initial L:%d R:%d\n", lastLeft, lastRight);

    while(1) {
        // Read current encoder states
        uint8_t currRight = (input(RIGHT_B) << 1) | input(RIGHT_A);
        uint8_t currLeft  = (input(LEFT_B) << 1) | input(LEFT_A);

        // Create index for lookup table: (current B&A << 2) | (last B&A)
        uint8_t rightIndex = (currRight << 2) | lastRight;
        uint8_t leftIndex  = (currLeft << 2)  | lastLeft;

        // Update counts based on state change via lookup table
        rightWheelCount += decoderLookup[rightIndex];
        leftWheelCount  += decoderLookup[leftIndex];

        // Store current state as last state for next iteration (mask to keep only 2 bits)
        lastRight = currRight & 0x03;
        lastLeft  = currLeft & 0x03;

        // Wait for a short period (100 microseconds)
        // This determines the maximum encoder pulse rate you can reliably read.
        // Adjust if needed based on encoder resolution and max speed.
        waitcnt(CNT + CLKFREQ / 10000); // ~10kHz sampling rate
    }
}

// --- Main Program ---
int main() {
    // Ensure clock is configured (important for timing like pause and waitcnt)
    clkset(_CLKMODE, _CLKFREQ);

    // Set encoder pins as inputs (optional, input is default, but explicit is clear)
    set_direction(RIGHT_A, 0);
    set_direction(RIGHT_B, 0);
    set_direction(LEFT_A, 0);
    set_direction(LEFT_B, 0);

    print("Initializing Motors...\n");
    initializeMotors(); // Setup motor control pins

    print("Starting Encoder Cog...\n");
    // Start the encoder reading cog
    cogstart(&encoderCog, NULL, encoderStack, sizeof(encoderStack));

    print("Robot Ready. Starting command sequence...\n");
    pause(500); // Give cog time to print its start message

    // Example Command Sequence:
    print("=== Moving Forward (Speed 50) for 2 seconds... ===\n");
    moveForward(50);
    for(int i=0; i<10; i++) { // Print counts during movement
       pause(200);
       print("Encoders -> Right: %d  Left: %d\n", rightWheelCount, leftWheelCount);
    }

    print("=== Turning Left (Speed 40) for 1.5 seconds... ===\n");
    turnLeft(40);
     for(int i=0; i<8; i++) {
       pause(187); // Approx 1.5 sec total
       print("Encoders -> Right: %d  Left: %d\n", rightWheelCount, leftWheelCount);
    }

    print("=== Moving Backward (Speed 50) for 2 seconds... ===\n");
    moveBackward(50);
    for(int i=0; i<10; i++) {
       pause(200);
       print("Encoders -> Right: %d  Left: %d\n", rightWheelCount, leftWheelCount);
    }

    print("=== Turning Right (Speed 40) for 1.5 seconds... ===\n");
    turnRight(40);
    for(int i=0; i<8; i++) {
       pause(187);
       print("Encoders -> Right: %d  Left: %d\n", rightWheelCount, leftWheelCount);
    }

    print("=== Stopping Motors... ===\n");
    stopMotors();
    pause(500); // Pause after stopping
    print("Final Counts -> Right: %d  Left: %d\n", rightWheelCount, leftWheelCount);

    print("Command sequence complete.\n");

    // The program will end here. In a real robot, main often enters an infinite
    // loop to continuously process commands or sensor data.
    // while(1) {
    //    // Check for new commands (e.g., from serial)
    //    // Update motor speeds based on commands or PID controllers
    //    // Monitor sensors
    //    // print("Current Counts -> Right: %d  Left: %d\n", rightWheelCount, leftWheelCount);
    //    // pause(50); // Loop delay
    // }

    return 0;
}

// --- Motor Control Function Implementations ---

/**
 * @brief Initializes motor control pins as outputs.
 */
void initializeMotors() {
    // Set direction pins as outputs
    set_direction(L_INA_PIN, 1);
    set_direction(L_INB_PIN, 1);
    set_direction(R_INA_PIN, 1);
    set_direction(R_INB_PIN, 1);

    // PWM pins are handled by servo library, setting as output is safe.
    set_direction(L_PWM_PIN, 1);
    set_direction(R_PWM_PIN, 1);

    // Ensure motors are stopped initially
    stopMotors();
    print("Motor pins initialized.\n");
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
        ina_pin = L_INA_PIN; inb_pin = L_INB_PIN; pwm_pin = L_PWM_PIN;
    } else if (motor_idx == RIGHT_MOTOR) {
        ina_pin = R_INA_PIN; inb_pin = R_INB_PIN; pwm_pin = R_PWM_PIN;
    } else { return; } // Invalid index

    // Clamp speed
    if (speed > MAX_PWM_VAL) speed = MAX_PWM_VAL;
    if (speed < -MAX_PWM_VAL) speed = -MAX_PWM_VAL;

    // Determine direction and PWM value
    if (speed > 0) { // Forward
        ina_val = 1; inb_val = 0; pwm_val = speed;
    } else if (speed < 0) { // Backward
        ina_val = 0; inb_val = 1; pwm_val = abs(speed);
    } else { // Stop (Coast)
        ina_val = 0; inb_val = 0; pwm_val = 0;
    }

    // Set direction pins
    set_output(ina_pin, ina_val);
    set_output(inb_pin, inb_val);

    // Set PWM speed using servo_speed (Assuming value 0-100 maps reasonably)
    // A value of 0 should disable the PWM output.
    servo_speed(pwm_pin, pwm_val);
}

/**
 * @brief Stops both motors (sets speed to 0).
 */
void stopMotors() {
    setMotorSpeed(LEFT_MOTOR, 0);
    setMotorSpeed(RIGHT_MOTOR, 0);
}

/**
 * @brief Moves the robot forward at a given speed.
 * @param speed Speed percentage (0-100).
 */
void moveForward(int speed) {
    speed = abs(speed); // Ensure positive
    if (speed > MAX_PWM_VAL) speed = MAX_PWM_VAL;
    setMotorSpeed(LEFT_MOTOR, speed);
    setMotorSpeed(RIGHT_MOTOR, speed);
}

/**
 * @brief Moves the robot backward at a given speed.
 * @param speed Speed percentage (0-100).
 */
void moveBackward(int speed) {
    speed = abs(speed);
    if (speed > MAX_PWM_VAL) speed = MAX_PWM_VAL;
    setMotorSpeed(LEFT_MOTOR, -speed);
    setMotorSpeed(RIGHT_MOTOR, -speed);
}

/**
 * @brief Rotates the robot left (counter-clockwise) in place.
 * @param speed Speed percentage (0-100) for rotation.
 */
void turnLeft(int speed) {
    speed = abs(speed);
    if (speed > MAX_PWM_VAL) speed = MAX_PWM_VAL;
    setMotorSpeed(LEFT_MOTOR, -speed); // Left backward
    setMotorSpeed(RIGHT_MOTOR, speed);  // Right forward
}

/**
 * @brief Rotates the robot right (clockwise) in place.
 * @param speed Speed percentage (0-100) for rotation.
 */
void turnRight(int speed) {
    speed = abs(speed);
    if (speed > MAX_PWM_VAL) speed = MAX_PWM_VAL;
    setMotorSpeed(LEFT_MOTOR, speed);   // Left forward
    setMotorSpeed(RIGHT_MOTOR, -speed); // Right backward
}
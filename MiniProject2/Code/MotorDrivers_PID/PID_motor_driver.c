/*
 *0. int main() COG0
 *1. Motor Setup
 *2. Encoder Function COG1
 *3. Currently implementing Prop.Gain for the motors -
 *
 *
 *
 *
 *
*/

#include "simpletools.h"  // Core Propeller functions
#include "servo.h"        // For PWM control via servo_speed
#include "propeller.h"    // For direct hardware access (if needed beyond simpletools)
#include <math.h>         // For abs()
#include "mstimer.h"

// --- Pin Definitions ---
// Adjust these pins according to your actual hardware wiring

// Motor Left
#define L_INA_PIN   4  // Left Motor Direction Input A
#define L_INB_PIN   3  // Left Motor Direction Input B
#define L_PWM_PIN   5  // Left Motor PWM Speed Control

// Motor Right
#define R_INA_PIN   2  // Right Motor Direction Input A
#define R_INB_PIN   1  // Right Motor Direction Input B
#define R_PWM_PIN   0  // Right Motor PWM Speed Control

// Encoder Left (Example Pins - Adjust as needed)
#define L_ENCA_PIN  9  // Left Encoder Phase A
#define L_ENCB_PIN  10  // Left Encoder Phase B

// Encoder Right (Example Pins - Adjust as needed)
#define R_ENCA_PIN  11 // Right Encoder Phase A
#define R_ENCB_PIN  12 // Right Encoder Phase B

// --- Constants ---
#define LEFT_MOTOR  1
#define RIGHT_MOTOR 0

#define MAX_PWM_VAL 10000 // Define max value for our speed input (-100 to 100)
#define MOTOR_STOP_TIME 50000
#define KP 0.8f               // Proportional gain (adjust as needed)
#define MAX_PWM_CHANGE 200 //MAX Change per cycle
#define MOTOR_PWM 6000


// --- Global Variables ---
// Use 'volatile' if these will be updated by another Cog (e.g., encoder cog)
static volatile int32_t rightWheelCount = 0;
static volatile int32_t leftWheelCount = 0;
static volatile int32_t prevLeftCount = 0;
static volatile int32_t prevRightCount = 0;
static int leftTargetPWM = 0;  // Store target PWM values
static int rightTargetPWM = 0;
static unsigned int encoderStack[128*2]; 


//-----Lookup table for Encoder-----
static const int8_t decoderLookup[16] = {
  
    0,  +1, -1, +2, 
  
    -1, 0,  -2,+1, 
    
    +1, -2, 0,  -1, 
    
    +2, -1, +1, 0   
};

typedef enum{
  LINEAR,
  ROT
}CONTROL_MODE;

static CONTROL_MODE current_mode = LINEAR;  

// --- Function Prototypes ---
void initializeMotors();
void initializeEncoders(); // Placeholder for encoder setup
void setMotorSpeed(int motor_idx, int speed);
void stopMotors();
void moveForward(int speed);
void moveBackward(int speed);
void turnLeft(int speed);  // Rotate in place left
void turnRight(int speed); // Rotate in place right
void encoderCog(void *par);
void velocityControlLoop();

// Placeholder for encoder reading function (likely run in separate cogs)
// void encoder_reader_cog(void *par);

// --- Main Program ---
int main() {
    clkset(_CLKMODE, _CLKFREQ);
    print("Initializing Robot...\n");
    initializeMotors();
    initializeEncoders(); // Initialize encoder pins (basic setup)
    cogstart(&encoderCog, NULL, encoderStack, sizeof(encoderStack));
    print("Robot Ready. Starting command sequence...\n");
    mstime_start();
    while(1){
      simpleterm_open();
    // Example Command Sequence:
    print("Moving Forward (Speed 50) for 2 seconds...\n");
    moveForward(MOTOR_PWM);
    int16_t cur_time = mstime_get();
    
    while(mstime_get() - cur_time <= MOTOR_STOP_TIME){
      
      //print("Right: %6d  Left: %6d\n", rightWheelCount, leftWheelCount);
      waitcnt(CNT+=CLKFREQ/1000);  
      velocityControlLoop();
        
      
      pause(100);
    }       
    /*
    print("Turning Left (Speed 40) for 1.5 seconds...\n");
    turnLeft(MOTOR_PWM);
    pause(1500);

    print("Moving Backward (Speed 50) for 2 seconds...\n");
    moveBackward(MOTOR_PWM);
    pause(2000);

    print("Turning Right (Speed 40) for 1.5 seconds...\n");
    turnRight(MOTOR_PWM);
    pause(1500);

    print("Stopping Motors...\n");
    stopMotors();

    print("Command sequence complete.\n");
    */
  }    


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
    current_mode = LINEAR;
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
    current_mode = LINEAR;
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
    current_mode = ROT;
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
    current_mode = ROT;
}



void encoderCog(void *par) {
    uint8_t lastRight = 0;
    uint8_t lastLeft = 0;
    
    while(1) {
      //getting the input values as xx
        uint8_t currRight = (input(R_ENCB_PIN) << 1) | input(R_ENCA_PIN);
        uint8_t currLeft = (input(L_ENCB_PIN) << 1) | input(L_ENCA_PIN);
        
      //left shifting them and adding the last input values
        uint8_t rightIndex = (currRight << 2) | lastRight;
        uint8_t leftIndex = (currLeft << 2) | lastLeft;
        
      //checking the lookup table
        rightWheelCount += decoderLookup[rightIndex];
        leftWheelCount += decoderLookup[leftIndex];
       //saving only the first two bits of the current value 
        lastRight = currRight & 0x03;  
        lastLeft = currLeft & 0x03;
        
        waitcnt(CNT + CLKFREQ/10000);  // 100μs delay
    }
}


void velocityControlLoop(){
     // 1. Get current encoder counts (atomic read)
    int32_t currLeft = leftWheelCount;
    int32_t currRight = rightWheelCount;
    
    // 2. Calculate actual deltas since last loop
    int32_t leftDelta = currLeft - prevLeftCount;
    int32_t rightDelta = currRight - prevRightCount;
    prevLeftCount = currLeft;
    prevRightCount = currRight;

    // 3. Calculate error (how much right needs to change to match left)
    int error = leftDelta - rightDelta;
   
    rightTargetPWM += (int)(KP*error*50);
    
    // 5. Apply bounds and set motors
    
    if (rightTargetPWM > MAX_PWM_VAL) rightTargetPWM = MAX_PWM_VAL;
    if (rightTargetPWM < -MAX_PWM_VAL) rightTargetPWM = -MAX_PWM_VAL;
    print("leftDelta =%d rightDelta = %d\n",leftDelta,rightDelta);
    setMotorSpeed(LEFT_MOTOR, rightTargetPWM);   // Right follows left's actual movement
    pause(100);
    
    
}
  

  

 
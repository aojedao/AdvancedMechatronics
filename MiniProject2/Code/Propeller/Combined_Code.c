/*
 *0. int main() COG0
 *1. Motor Setup
 *2. Encoder Function COG1
 *3. IMU Calculations COG2
 *
*/

#include "simpletools.h"  // Core Propeller functions
#include "servo.h"        // For PWM control via servo_speed
#include "propeller.h"    // For direct hardware access (if needed beyond simpletools)
#include <math.h>         // For abs()
#include "mstimer.h"
#include "simplei2c.h"    // for the I2C communication for IMU.
#include "fdserial.h"

// --- Pin Definitions ---
// Adjust these pins according to your actual hardware wiring

// Motor Left
#define L_INA_PIN    1// Left Motor Direction Input A
#define L_INB_PIN    2// Left Motor Direction Input B
#define L_PWM_PIN    0// Left Motor PWM Speed Control

// Motor Right
#define R_INA_PIN   3 // Right Motor Direction Input A
#define R_INB_PIN   4 // Right Motor Direction Input B
#define R_PWM_PIN   5 // Right Motor PWM Speed Control

// Encoder Left (Example Pins - Adjust as needed)
#define L_ENCA_PIN  9  // Left Encoder Phase A
#define L_ENCB_PIN  10  // Left Encoder Phase B

// Encoder Right (Example Pins - Adjust as needed)
#define R_ENCA_PIN  11 // Right Encoder Phase A
#define R_ENCB_PIN  12 // Right Encoder Phase B

// --- Constants ---
#define LEFT_MOTOR  1
#define RIGHT_MOTOR 0

#define IMU_CALIBRATION_SAMPLES 200 // WARNING only 200 to speed up startup process.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1

#define MAX_PWM_VAL 10000 // Define max value for our speed input (-100 to 100)
#define MOTOR_STOP_TIME 50000
#define KP 0.8f               // Proportional gain (adjust as needed)
#define MAX_PWM_CHANGE 200 //MAX Change per cycle
#define MOTOR_PWM 6000
#define MPU6050_ADDR 0x68  // Default I2C address (AD0 pin low)
#define SCL_PIN 6          // Use P0 for SCL
#define SDA_PIN 7          // Use P1 for SDA

// Register addresses from the IMU datasheet
#define PWR_MGMT_1     0x6B 
#define GYRO_CONFIG    0x1B
#define ACCEL_CONFIG   0x1C
#define ACCEL_XOUT_H   0x3B
#define TEMP_OUT_H     0x41
#define GYRO_XOUT_H    0x43

#ifndef PI
#define PI 3.148
#endif

//WHEEL dimensions
#define WHEEL_DIAMETER 0.065     // meters
#define WHEEL_CIRCUMFERENCE (PI * WHEEL_DIAMETER)
#define COUNTS_PER_REV (48 * 4)  // 48 CPR with quadrature decoding
#define GEAR_RATIO 4.88          // Calculated from your data
#define COUNTS_PER_WHEEL_REV (COUNTS_PER_REV * GEAR_RATIO)
#define METERS_PER_COUNT (WHEEL_CIRCUMFERENCE / COUNTS_PER_WHEEL_REV)

// --- USART Configuration ---
#define RX_SERIAL_PIN 14 // Pin for receiving data (Connect to device's Tx)
#define TX_SERIAL_PIN 15  // Pin for transmitting data (Connect to device's Rx)
#define SERIAL_BAUD_RATE 115200 // Communication speed (must match connected device)
fdserial *usart;          // Full-duplex serial object

#define REPORT_PERIOD_MS 10

// --- Global Variables ---
int speed = 9000;         // Default speed for movement
int rot_time = 3250;      // Default duration for movement (in milliseconds)
int lin_time = 3250;      // Default duration for movement (in milliseconds)

// --- Global Variables ---
// Use 'volatile' if these will be updated by another Cog (e.g., encoder cog)
static volatile int32_t rightWheelCount = 0;
static volatile int32_t leftWheelCount = 0;
static volatile int32_t prevLeftCount = 0;
static volatile int32_t prevRightCount = 0;
static int leftTargetPWM = 0;  // Store target PWM values
static int rightTargetPWM = 0;
static unsigned int encoderStack[128*2]; 
static unsigned int imuStack[128*2]; 
static float ax = 0, ay = 0, az = 0, gx = 0, gy = 0,gz = 0;
static float print_gx = 0, print_gy = 0, print_gz = 0,ax_angle,ay_angle,yaw = 0,pitch =0, roll = 0,gyroAngleX = 0, gyroAngleY = 0;
static float gyro_offsets[3] = {0};
static float accel_offsets[3] = {0};
static float x_cord = 0,y_cord = 0,theta = 0,velocity = 0,omega = 0;
static int imu_calibration_flag = 0;
//-----Lookup table for Encoder-----
static const int8_t decoderLookup[16] = {
  
    0,  +1, -1, +2, 
  
    -1, 0,  -2,+1, 
    
    +1, -2, 0,  -1, 
    
    +2, -1, +1, 0   
};

typedef enum{
  FRWD,
  BKWD,
  ROT_L,
  ROT_R,
  IDLE
}CONTROL_MODE;

// IMU Sensor data structure
typedef struct {
    float accel[3];  // X,Y,Z
    float gyro[3];   // X,Y,Z
    float temp;
} imu_data_t;

typedef struct{
    int16_t accel[3];
    int16_t gyro[3];
    int16_t temp;
  
} received_data;


i2c imu;  // I2C bus for MPU-60X0

static CONTROL_MODE current_mode = IDLE;  
static int32_t last_mode_change = 0;
static int32_t last_report_timestamp = 0;

// --- Function Prototypes ---
// --- Initialization ---
void initializeMotors(); //set pins for motors
void initializeEncoders(); // Placeholder for encoder setup
void mpu6050_init(); // initialize mpu6050
void calibrate_gyro(int samples);

// --- Motor Setup ---
void setMotorSpeed(int motor_idx, int speed); //set motor speeds and also the direction
void stopMotors(); //set pwm to 0 to stop the motors
void moveForward(int speed); //move forward
void moveBackward(int speed); //...
void turnLeft(int speed);  // Rotate in place left
void turnRight(int speed); // Rotate in place right

// --- Encoder Function ---
void encoderCog(void *par); //function for encoder values

// --- Proportional gain for Wheels ---
void wheelVelocityControlLoop(); //function for Proportional gain 

// --- IMU Functions ---
void mpu6050_read(void *par2) ;
void mpu6050_read_cog(void *par2); // Function that runs read in while(1) to be used in a cog.
void calc_imu(int16_t elapsed_time);
void update_odometry(int16_t elapsed_time);

// --- USART Communication ---
void handleUSARTCommands();  // Function to handle USART commands
void parseCommand(char *command);  // Function to parse `tag:value` commands

void sendDataIfNeeded(); // This funciton is designed to be called from the main loop and will send data only if it needs to, based on time and data available.

// Placeholder for encoder reading function (likely run in separate cogs)
// void encoder_reader_cog(void *par);

// --- Main Program ---
int main() {
    // SETUP
 
    int16_t prev_time = 0;
    int16_t elapsed_time = 0; 
    int16_t current_time = 0;
    
    clkset(_CLKMODE, _CLKFREQ);
    print("..........Initializing Robot..........\n..........Initializing Motors..........\n");
    
    initializeMotors(); //Init motors
    pause(200);
    print("..........Initializing Encoder..........\n");
    initializeEncoders(); // Initialize encoder pins (basic setup)
    pause(200);
    print("..........Initializing IMU..........\n");
    mpu6050_init();
    pause(200);
    print("..........Starting Cog1 for Encoder..........\n");
    cogstart(&encoderCog, NULL, encoderStack, sizeof(encoderStack));
    print("..........Starting Cog2 for IMU..........\n");
    cogstart(&mpu6050_read_cog, NULL, imuStack, sizeof(imuStack));
    calibrate_gyro(IMU_CALIBRATION_SAMPLES);
    print("Robot Ready. Starting command sequence...\n");
    mstime_start();
    
    // --- Initialize the Serial Connection ---
    usart = fdserial_open(RX_SERIAL_PIN, TX_SERIAL_PIN, 0, SERIAL_BAUD_RATE);
    
    // --- Check for Initialization Errors ---
    if (usart == NULL) {
        print("Error: Failed to open fdserial on P%d(Rx)/P%d(Tx).\n", RX_SERIAL_PIN, TX_SERIAL_PIN);
        while (1) { // Halt with LED blink on error
            high(26); pause(100); low(26); pause(100);
        }
    }
    
    // Main loop
    while (1) {
      
      if (current_mode == IDLE){
        handleUSARTCommands();  // Check for and handle USART commands        
        sendDataIfNeeded();  
      }else{
      // We are moving! Controll the speed of both wheels and stop the motion.
        switch(current_mode){
            case FRWD:
       
            while((mstime_get() - last_mode_change)< lin_time){
              wheelVelocityControlLoop(); 
              sendDataIfNeeded();
            }
            stopMotors();
            break;
            
            case BKWD:
       
            while((mstime_get() - last_mode_change)< lin_time){
              wheelVelocityControlLoop(); 
              sendDataIfNeeded();
            }
            stopMotors();
            break;
            
            case ROT_L:
            
       
            while((mstime_get() - last_mode_change)< rot_time){
              wheelVelocityControlLoop(); 
              sendDataIfNeeded();
            }
            stopMotors();
            break;
            
            case ROT_R:
            while((mstime_get() - last_mode_change)< rot_time){
              wheelVelocityControlLoop(); 
              sendDataIfNeeded();
            }
            stopMotors();
            break;
          default:
          break;
        }          

      }  
      /*
      print("%c", CLS);  // Clear terminal
      print("%c",HOME);
      print("Odometry Data\n");
      print("-------------\n");
      print("Position: X=%.2fm Y=%.2fm\n", x_cord, y_cord);
      print("Heading: %.1f°\n", theta * 180.0 / PI);
      print("Velocity: %.2fm/s Omega=%.2frad/s\n", velocity, omega);
      print("Encoders: R=%d L=%d\n", rightWheelCount, leftWheelCount);
      print("\nMPU-60X0 IMU Data\n");
      print("-----------------\n");
      print("Accel: X=%7.2fg  Y=%7.2fg  Z=%7.2fg\n", ax, ay, az);
      print("Clockfreq = %d\t elapsed_time = %d\n",CLKFREQ,elapsed_time);
      print("Gyro:  X=%7.2f°/s Y=%7.2f°/s Z=%7.2f°/s\n", print_gx, print_gy, print_gz);
      print("Roll =%7.2f°/s Pitch=%7.2f°/s Yaw=%7.2f°/s\n", roll, pitch, yaw);  
      pause(500);
        */   
  

    }

    
    
    /*
    // INFINITE LOOP COG 0
    while(1){
      prev_time = current_time;
      waitcnt(CNT += CLKFREQ/1000);
      current_time = mstime_get();
      elapsed_time = (current_time - prev_time)/1000;
      calc_imu(elapsed_time);
      print("Moving Forward (Speed 50) for 2 seconds...\n");
      //turnRight(MOTOR_PWM); &tested and works&
      //turnLeft(MOTOR_PWM); & tested and works&
      //moveForward(MOTOR_PWM); &tested and works&
      moveBackward(MOTOR_PWM);
      int16_t cur_time = mstime_get();*/
      
      /*
       *I have not figured out how to make the robot move and when to call the WASD commands and put a loop into them
       *But below are all the values that We have computed.
       *all this needs is a propper call function from the values we get from the esp32
       *I also took the liberty to impliment a basic kalman filteration. I did the code without the help of AI so it might be wrong. Feel free to omit that part
       *the function is update_odometry. Just to see if we are getting the values with testing.
       *
       *ENTER THE CODE FOR ESP32 serial communication
       */
       
      /*while(mstime_get() - cur_time <= MOTOR_STOP_TIME){
      
      //print("Right: %6d  Left: %6d\n", rightWheelCount, leftWheelCount);
      prev_time = current_time;
      waitcnt(CNT+=CLKFREQ/1000);
      current_time = mstime_get();
      elapsed_time = (current_time - prev_time)/1000;  
      //velocityLinearControlLoop();
      velocityAngularControlLoop(); 
      update_odometry(elapsed_time);
      
      print("%c", HOME);  // Clear terminal
      print("Odometry Data\n");
      print("-------------\n");
      print("Position: X=%.2fm Y=%.2fm\n", x_cord, y_cord);
      print("Heading: %.1f°\n", theta * 180.0 / PI);
      print("Velocity: %.2fm/s Omega=%.2frad/s\n", velocity, omega);
      print("Encoders: R=%d L=%d\n", rightWheelCount, leftWheelCount);
      print("\nMPU-60X0 IMU Data\n");
      print("-----------------\n");
      print("Accel: X=%7.2fg  Y=%7.2fg  Z=%7.2fg\n", ax, ay, az);
      print("Clockfreq = %d\t elapsed_time = %d\n",CLKFREQ,elapsed_time);
      print("Gyro:  X=%7.2f°/s Y=%7.2f°/s Z=%7.2f°/s\n", temp_gx, temp_gy, temp_gz);
      print("Roll =%7.2f°/s Pitch=%7.2f°/s Yaw=%7.2f°/s\n", roll, pitch, yaw);
      
      pause(100);
    }       
    
  }    */


    return 0; 
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
        ina_val = 0;
        inb_val = 1;
        pwm_val = speed;
    } else if (speed < 0) { // Backward
        ina_val = 1;
        inb_val = 0;
        pwm_val = abs(speed); // PWM value is always positive
    } else { // Stop (Coast)
        ina_val = 0; // Set both low for coasting (check your motor driver!)
        inb_val = 0; // Some drivers brake if both are high. Coast is usually safer.
        pwm_val = 0;
    }

    // Set direction pins
    set_output(ina_pin, ina_val);
    set_output(inb_pin, inb_val);
    servo_speed(pwm_pin, pwm_val);

}

/**
 * @brief Stops both motors (sets speed to 0).
 */
void stopMotors() {
    setMotorSpeed(LEFT_MOTOR, 0);
    setMotorSpeed(RIGHT_MOTOR, 0);
    current_mode = IDLE;
    last_mode_change = mstime_get();
    // Optional: Add a small delay to ensure commands are processed
    pause(10);
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
    current_mode = FRWD;
    last_mode_change = mstime_get();
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
    current_mode = BKWD;
    last_mode_change = mstime_get();
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
    current_mode = ROT_L;
    last_mode_change = mstime_get();
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
    current_mode = ROT_R;
    last_mode_change = mstime_get();
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
 
void wheelVelocityControlLoop(){
     // 1. Get current encoder counts (atomic read)
    int32_t currLeft = leftWheelCount;
    int32_t currRight = rightWheelCount;
    
    // 2. Calculate actual deltas since last loop
    int32_t leftDelta = currLeft - prevLeftCount;
    int32_t rightDelta = currRight - prevRightCount;
    prevLeftCount = currLeft;
    prevRightCount = currRight;

    // 3. Calculate error (how much right needs to change to match left)
    int error = abs(leftDelta) - abs(rightDelta);
    /*
    if(current_mode != LINEAR){
      if(current_mode == ROT_L) error = leftDelta - abs(rightDelta);
      if(current_mode == ROT_R) error = leftDelta - rightDelta;
    }else{    
     error = leftDelta - abs(rightDelta);
     
    } 
    */
    rightTargetPWM += (int)(KP*error*50);   
    
    
    // 5. Apply bounds and set motors
    
    if (rightTargetPWM > MAX_PWM_VAL) rightTargetPWM = MAX_PWM_VAL;
    if (rightTargetPWM < -MAX_PWM_VAL) rightTargetPWM = -MAX_PWM_VAL;
    //print("leftDelta =%d rightDelta = %d\n",leftDelta,rightDelta);
    if(current_mode == FRWD || current_mode == ROT_L)
      setMotorSpeed(RIGHT_MOTOR, rightTargetPWM);   // Right follows left's actual movement
    else 
      setMotorSpeed(RIGHT_MOTOR, -1*rightTargetPWM);
    pause(100);
    
    
}

// Initialize MPU-60X0
void mpu6050_init() {
    // Initialize I2C
    i2c_open(&imu, SCL_PIN, SDA_PIN, 0);
    
    // Wake up MPU-60X0 (disable sleep mode)
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, PWR_MGMT_1);
    i2c_writeByte(&imu, 0x80);  // Set DEVICE_RESET bit
    i2c_stop(&imu);
    
    // Wait for reset to complete
    pause(100);
    
    // 2. Wake up MPU-6050 and configure clock source
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, PWR_MGMT_1);
    // CLKSEL = 1 (PLL with X gyro reference)
    // Also clears SLEEP bit and TEMP_DIS bit
    i2c_writeByte(&imu, 0x01);  
    i2c_stop(&imu);
    
    // Configure gyroscope range (±250°/s)
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, GYRO_CONFIG);
    i2c_writeByte(&imu, 0x00);  // ±250°/s (FS_SEL=0)
    i2c_stop(&imu);
    
    // Configure accelerometer range (±2g)
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, ACCEL_CONFIG);
    i2c_writeByte(&imu, 0x00);  // ±2g (AFS_SEL=0)
    i2c_stop(&imu);
   
}

void mpu6050_read(void *par2) {
  int16_t buf[14];
  
  // Start reading from register 0x3B (ACCEL_XOUT_H)
  i2c_start(&imu);
  i2c_writeByte(&imu, MPU6050_ADDR << 1);
  i2c_writeByte(&imu, ACCEL_XOUT_H);
  
  // Repeated start to begin reading
  i2c_start(&imu);
  i2c_writeByte(&imu, (MPU6050_ADDR << 1) | 1);
  
  // Read 14 bytes (accel, temp, gyro)
  for(int i = 0; i < 13; i++) {
     
      buf[i] = i2c_readByte(&imu, 0);  // ACK all but last byte

  }
  buf[13] = i2c_readByte(&imu, 1);     // NAK last byte
  i2c_stop(&imu);
  
  // Format data (registers are big-endian)
  received_data R_Data;
  int16_t temp_ax ,temp_ay,temp_az,temp_gx,temp_gy,temp_gz;
  temp_ax = ((buf[0] << 8) | buf[1]);
  ax = (float)(temp_ax ) / 16384.0 ;   // X
  temp_ay = ((buf[2] << 8) | buf[3]);
  ay = (float)(temp_ay) / 16384.0 ;   // Y
  temp_az = ((buf[4] << 8) | buf[5]);
  az = (float)(temp_az) / 16384.0 ;   // Z
  temp_gx  = (buf[8] << 8) | buf[9];   // X
  gx = (float)(temp_gx)/131.0;
  temp_gy  = (buf[10] << 8) | buf[11];   // X
  gy = (float)(temp_gy)/131.0;
  temp_gz  = (buf[12] << 8) | buf[13];   // X
  gz = (float)(temp_gz)/131.0;
    
}

void mpu6050_read_cog(void *par2) {
  while(1){
    int16_t buf[14];
    
    // Start reading from register 0x3B (ACCEL_XOUT_H)
    i2c_start(&imu);
    i2c_writeByte(&imu, MPU6050_ADDR << 1);
    i2c_writeByte(&imu, ACCEL_XOUT_H);
    
    // Repeated start to begin reading
    i2c_start(&imu);
    i2c_writeByte(&imu, (MPU6050_ADDR << 1) | 1);
    
    // Read 14 bytes (accel, temp, gyro)
    for(int i = 0; i < 13; i++) {
       
        buf[i] = i2c_readByte(&imu, 0);  // ACK all but last byte
  
    }
    buf[13] = i2c_readByte(&imu, 1);     // NAK last byte
    i2c_stop(&imu);
    
    // Format data (registers are big-endian)
    received_data R_Data;
    int16_t temp_ax ,temp_ay,temp_az,temp_gx,temp_gy,temp_gz;
    temp_ax = ((buf[0] << 8) | buf[1]);
    ax = (float)(temp_ax ) / 16384.0 ;   // X
    temp_ay = ((buf[2] << 8) | buf[3]);
    ay = (float)(temp_ay) / 16384.0 ;   // Y
    temp_az = ((buf[4] << 8) | buf[5]);
    az = (float)(temp_az) / 16384.0 ;   // Z
    temp_gx  = (buf[8] << 8) | buf[9];   // X
    gx = (float)(temp_gx)/131.0;
    temp_gy  = (buf[10] << 8) | buf[11];   // X
    gy = (float)(temp_gy)/131.0;
    temp_gz  = (buf[12] << 8) | buf[13];   // X
    gz = (float)(temp_gz)/131.0;
    print_gx = gx-gyro_offsets[0];
    print_gy = gy-gyro_offsets[1];
    print_gz = gz-gyro_offsets[2];
    
  }    
}



 void calibrate_gyro(int samples) {
   print("Calibrating gyro... keep sensor still!\n");

  
    
    float gyro_sum[3] = {0};
    float accel_sum[3] = {0};
    
    
    for(int i = 0; i < samples; i++) {
        gyro_sum[0] += gx;
        gyro_sum[1] += gy;
        gyro_sum[2] += gz;
        ax_angle = ax_angle + ((atan((ay) / sqrt(pow((ax), 2) + pow((az), 2))) * 180 / PI));
        ay_angle = ay_angle + ((atan(-1 * (ax) / sqrt(pow((ay), 2) + pow((az), 2))) * 180 / PI));
        pause(10);
    }
    
    gyro_offsets[0] = gyro_sum[0] / samples;
    gyro_offsets[1] = gyro_sum[1] / samples;
    gyro_offsets[2] = gyro_sum[2] / samples;
    accel_offsets[0] = accel_sum[0]/ samples;
    accel_offsets[1] = accel_sum[1]/ samples;
    print("Finished Calibration\n");
    pause(200);
}

void calc_imu(int16_t elapsed_time){
  
  
  print_gx = gx -  gyro_offsets[0];
  print_gy = gy -  gyro_offsets[1];    
  print_gz = gz - gyro_offsets[2];
  ax_angle = (atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * 180 / PI) - accel_offsets[0];
  ay_angle =  (atan(-1 * ax / sqrt(pow(ay, 2) + pow(az, 2))) * 180 / PI) - accel_offsets[0];
  gyroAngleX = gyroAngleX + gx * elapsed_time;
  gyroAngleY = gyroAngleY + gy * elapsed_time;
  yaw = yaw + gz*elapsed_time;
  roll = 0.96 * gx + 0.04*ax;
  pitch = 0.96*gy + 0.04*ay;
   
  
  
}  


void update_odometry(int16_t elapsed_time) {
    static int32_t prevRightCount = 0;
    static int32_t prevLeftCount = 0;
    
    float rightDist = (rightWheelCount - prevRightCount) * METERS_PER_COUNT;
    float leftDist = (leftWheelCount - prevLeftCount) * METERS_PER_COUNT;
    
    prevRightCount = rightWheelCount;
    prevLeftCount = leftWheelCount;
    
    
    float linear_dist = (rightDist + leftDist) / 2.0;
    float angular_dist = (rightDist - leftDist) / WHEEL_DIAMETER;
    
    
    float imu_omega = gx * PI / 180.0; 
    
    
    float alpha = 0.98; 
    
    theta += imu_omega * elapsed_time;
    theta = alpha * theta + (1-alpha) * angular_dist; 
    
    // Update position
    x_cord += linear_dist * cos(theta);
    y_cord += linear_dist * sin(theta);
    velocity = linear_dist / elapsed_time;
    omega = imu_omega;
}

/**
 * @brief Handles USART commands from the ESP32.
 */
void handleUSARTCommands() {
  char buffer[64];  // Buffer to store incoming command
  int index = 0;

  // Check if data is available on the USART
  while (fdserial_rxReady(usart)) {
      char c = fdserial_rxChar(usart);  // Read a character

      // Check for end of command (newline or carriage return)
      if (c == '\n') {
          buffer[index] = '\0';  // Null-terminate the string
          parseCommand(buffer);  // Parse the command
          index = 0;             // Reset the buffer index
      } else {
          buffer[index++] = c;   // Add character to buffer
          if (index >= sizeof(buffer) - 1) {
              buffer[index] = '\0';  // Null-terminate to prevent overflow
              parseCommand(buffer);  // Parse the command
              index = 0;             // Reset the buffer index
          }
      }
  }
}

/**
* @brief Parses a `tag:value` command and applies the corresponding action.
*
* @param command The command string in the format `tag:value`.
*/
void parseCommand(char *command) {
  char *tag = strtok(command, ":");  // Extract the tag
  char *value = strtok(NULL, ":");  // Extract the value

  if (tag && value) {
      if (strcmp(tag, "MotorSpeed") == 0) {
          speed = atoi(value);  // Convert value to integer and set speed
          print("Speed set to %d\n", speed);
      } else if (strcmp(tag, "RotTime") == 0) {
          rot_time = atoi(value);  // Convert value to integer and set duration
          print("Rotation duration set to %d ms\n", rot_time);
      } else if (strcmp(tag, "LinTime") == 0) {
          lin_time = atoi(value);  // Convert value to integer and set duration
          print("Lienar duration set to %d ms\n", lin_time);
      } else if (strcmp(tag, "command") == 0) {
          // Handle motion commands (e.g., w, a, s, d)
          char motion = value[0];  // Get the first character of the value
          switch (motion) {
              case 'W':  // Move forward
                  print("Command: Move Forward\n");
                  moveForward(speed);
                  break;

              case 'S':  // Move backward
                  print("Command: Move Backward\n");
                  moveBackward(speed);
                  break;

              case 'A':  // Turn left
                  print("Command: Turn Left\n");
                  turnLeft(speed);
                  break;

              case 'D':  // Turn right
                  print("Command: Turn Right\n");
                  turnRight(speed);
                  break;

              case 'X':  // Stop
                  print("Command: Stop\n");
                  stopMotors();
                  break;

              case 'Q':  // Reset
                  print("Command: Reset\n");
                  rightWheelCount = 0;
                  leftWheelCount = 0;
                  break;


              default:
                  print("Unknown motion command: %c\n", motion);
                  break;
          }
      } else {
          print("Unknown tag: %s\n", tag);
      }
  } else {
      print("Invalid command format: %s\n", command);
  }
}


void sendDataIfNeeded()
{
  if((mstime_get() - last_report_timestamp) > REPORT_PERIOD_MS)
  {
    // Create a JSON string with the encoder and IMU data
    dprint(usart, "{");
    dprint(usart, "\"leftWheelCount\": %d, ", leftWheelCount);
    dprint(usart, "\"rightWheelCount\": %d, ", rightWheelCount);
    dprint(usart, "\"ax\": %.2f, ", ax);
    dprint(usart, "\"ay\": %.2f, ", ay);
    dprint(usart, "\"az\": %.2f, ", az);
    dprint(usart, "\"gx\": %.2f, ", print_gx);
    dprint(usart, "\"gy\": %.2f, ", print_gy);
    dprint(usart, "\"gz\": %.2f", print_gz);
    dprint(usart, "}\n");
    
    last_report_timestamp = mstime_get();
  }    
}  
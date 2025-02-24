#ifndef BLE_ROBOT_H
#define BLE_ROBOT_H

/**
 * @file BLE_Robot.h
 * @brief Header file defining constants, pin assignments, and global variables for BLE-controlled robot.
 * @details Provides configuration for BLE services, motor/encoder pins, and shared variables for
 *          both WASD manual control and waypoint navigation modes.
 */

#include <ArduinoBLE.h>

// BLE Service and Characteristic UUIDs
#define ENC_SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define POSE_CHAR_UUID   "19B10001-E8F2-537E-4F6C-D104768A1215"
#define COMMAND_SERVICE_UUID "180A"
#define X_DIST_CHAR_UUID "0x2701"
#define Y_DIST_CHAR_UUID "0x2702"
#define WASD_CHAR_UUID   "0x2A00"



// Pin Definitions
#define ENC_LEFT_A  10
#define ENC_LEFT_B  11
#define ENC_RIGHT_A 5
#define ENC_RIGHT_B 6

// Motor A connections - Right
int enA = 2;
int in1 = 3;
int in2 = 4;

#define MOTOR_LEFT_PWM  7
#define MOTOR_LEFT_DIR_1  8
#define MOTOR_LEFT_DIR_2  9

#define MOTOR_RIGHT_PWM 2
#define MOTOR_RIGHT_DIR_1 3
#define MOTOR_RIGHT_DIR_2 4

#define LED_PIN LED_BUILTIN

// Robot Parameters
#define WHEEL_RADIUS 0.021
#define WHEEL_BASE   0.2
#define TICKS_PER_REV 48
#define MAX_SPEED    0.223
#define MAX_TURN_SPEED 1.0
#define K_THETA      1.5
#define K_V          1.0

// Global Variables
extern volatile long leftTicks;
extern volatile long rightTicks;
extern float xPos, yPos, theta;
extern BLEService encService;
extern BLEService commandService;
extern BLEFloatCharacteristic poseChar;
extern BLEFloatCharacteristic xDistChar;
extern BLEFloatCharacteristic yDistChar;
extern BLECharCharacteristic wasdChar;

#endif // BLE_ROBOT_H

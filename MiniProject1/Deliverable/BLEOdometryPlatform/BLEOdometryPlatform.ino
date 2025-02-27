/**
 * @file BLEOdometryPlatform.ino
 * @brief Control differential drive robot via BLE with WASDT commands on Arduino Nano 33 BLE.
 * @details WASDT manual control with 'Q' exit—sends "Theta: theta; X: x; Y: y" via BLE.
 *          Left: enA 7, in1 13, in2 8; Right: enB 2, in3 4, in4 3; Encoders: Left 10,11, Right 6,5.
 *          This Platform is the first mini project in a serie of 3 projects where 
 *          the team is tring to build a ship yard automation system for handling cargo.
 *          In this iterarioin a 2 wheeled robot mover around while being controlled by a BLE controller, and estimates its pose along the way using only necoder data.     
 */

#include <ArduinoBLE.h>
#include <Encoder.h>

#include "config.h"
#include "motorDriver.hpp"
#include "poseEstimator.hpp"

// Encoder pins
Encoder leftEnc(10, 11);  // Left encoder: Pin 10 (A), Pin 11 (B)
Encoder rightEnc(6, 5);   // Right encoder: Pin 6 (A), Pin 5 (B)


// Pose variables
float xPos = 0.0, yPos = 0.0, theta = 0.0;

// BLE setup
BLEService robotService("19B10000-E8F2-537E-4F6C-D104768A1214");  // Service UUID
BLECharCharacteristic commandChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);  // WASDTQ: 'W' (0x57), 'A' (0x41), 'S' (0x53), 'D' (0x44), 'T' (0x54), 'Q' (0x51)
BLEStringCharacteristic poseChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, 40);  // "Theta: theta; X: x; Y: y", max 40 chars
BLEUnsignedIntCharacteristic rotTimeChar("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // Comand to share and receive the rotation Time
BLEUnsignedIntCharacteristic linTimeChar("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // Comand to share and receive the linear Time
BLEUnsignedIntCharacteristic speedPWMChar("19B10006-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite); // Comand to share and receive the PWM set to the motors.


// Function to handle WASDT commands
void handleWASDT() {
  if (commandChar.written()) {
    char command = commandChar.value();
    Serial.print("Received: "); Serial.print(command); Serial.print(" (hex: "); Serial.print(command, HEX); Serial.println("");

    if (command == 'Q') {  // Exit on 'Q' (0x51)—stops robot, resets position
      setMotorSpeeds(0, 0);
      xPos = 0.0; yPos = 0.0; theta = 0.0;
      leftEnc.write(0); rightEnc.write(0);
      Serial.println("Exiting—position reset");
      updatePose(xPos, yPos, theta, leftEnc, rightEnc);
      Serial.println(xPos);
      Serial.println((yPos));

      // Combine theta, x, y into a single string with labels
      String poseStr = "Theta: " + String(theta, 2) + "; X: " + String(xPos, 2) + "; Y: " + String(yPos, 2);

      // Send combined string via BLE
      poseChar.writeValue(poseStr);

      // Display current theta, x, y on Serial for debugging
      Serial.print("Pose sent via BLE: "); Serial.println(poseStr);
      Serial.print("Ticks: "); Serial.print(leftEnc.read()); Serial.print(" | "); Serial.println(rightEnc.read());
      return;
    }

    unsigned int rot_time = rotTimeChar.value();
    unsigned int lin_time = linTimeChar.value();
    unsigned int speed_pwm = speedPWMChar.value();

    Serial.println(lin_time);
    Serial.println(rot_time);

    switch (command) {
      case 'W': setMotorSpeeds(speed_pwm, speed_pwm); delay(lin_time); break;  // Forward: 0x57
      case 'S': setMotorSpeeds(-speed_pwm, -speed_pwm); delay(lin_time); break;  // Backward: 0x53
      case 'A': setMotorSpeeds(-speed_pwm, speed_pwm); delay(rot_time); break;  // Left: 0x41
      case 'D': setMotorSpeeds(speed_pwm, -speed_pwm); delay(rot_time); break;  // Right: 0x44
      case 'T': setMotorSpeeds(0, 0); break;  // Stop: 0x54
      default: Serial.println("Invalid command"); return;
    }
    setMotorSpeeds(0, 0);
    updatePose(xPos, yPos, theta, leftEnc, rightEnc);
    Serial.println(xPos);
    Serial.println((yPos));

    // Combine theta, x, y into a single string with labels
    String poseStr = "Theta: " + String(theta, 2) + "; X: " + String(xPos, 2) + "; Y: " + String(yPos, 2);

    // Send combined string via BLE
    poseChar.writeValue(poseStr);

    // Display current theta, x, y on Serial for debugging
    Serial.print("Pose sent via BLE: "); Serial.println(poseStr);
    Serial.print("Ticks: "); Serial.print(leftEnc.read()); Serial.print(" | "); Serial.println(rightEnc.read());
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("Starting setup...");  // Debug
  

  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  setMotorSpeeds(0, 0);

  delay(1000);  // Give BLE time
  Serial.println("Before BLE begin...");  // Debug
  if (!BLE.begin()) {
    Serial.println("BLE failed!");
    while (1);
  }

  BLE.setDeviceName("Robot-BLE-DeadReckon");
  BLE.setLocalName("Team3FirstRobot");
  BLE.setAdvertisedService(robotService);
  robotService.addCharacteristic(commandChar);
  robotService.addCharacteristic(poseChar);
  robotService.addCharacteristic(rotTimeChar);
  robotService.addCharacteristic(linTimeChar);
  robotService.addCharacteristic(speedPWMChar);
  BLE.addService(robotService);
  
  commandChar.writeValue('T'); // Default: Stop (0x54)
  rotTimeChar.writeValue(ROTATION_TIME);
  linTimeChar.writeValue(ROTATION_TIME);
  speedPWMChar.writeValue(MAX_SPEED);
  poseChar.writeValue("Theta: 0.00; X: 0.00; Y: 0.00");  // Initial pose as string
  BLE.advertise();

  Serial.println("BLE ready - Connect via LightBlue, send WASDT commands");
  Serial.println("Commands: 'W' (0x57), 'A' (0x41), 'S' (0x53), 'D' (0x44), 'T' (0x54), 'Q' (0x51)");
  Serial.println("Read pose (0x19B10003) as 'Theta: theta; X: x; Y: y'");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to: "); Serial.println(central.address());
    
    while (central.connected()) {
      handleWASDT();
      delay(100);  // Give BLE stack time to breathe
    }
    Serial.print("Disconnected from: "); Serial.println(central.address());
    setMotorSpeeds(0, 0);
  }
  delay(100);  // Avoid tight loop when not connected
}

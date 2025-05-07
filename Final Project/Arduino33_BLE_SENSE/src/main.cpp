#include <ArduinoBLE.h>
#include <Encoder.h>
#include "config.h"
#include "motorDriver.hpp"
#include "poseEstimator.hpp"

// Encoder pins (as in your files)
Encoder leftEnc(10, 11);  // Left encoder: Pin 10 (A), Pin 11 (B)
Encoder rightEnc(6, 5);   // Right encoder: Pin 6 (A), Pin 5 (B)

// BLE configuration
#define BLE_DEVICE_NAME   "BLE_Arduino_Pandebono"
#define BLE_SERVICE_UUID  "0000ffe1-0000-1000-8000-00805f9b34fb"
#define BLE_CHAR_UUID_RX  "0000ffe1-0000-1000-8000-00805f9b34fb"

// BLE objects
BLEService robotService(BLE_SERVICE_UUID);
BLECharacteristic rxChar(BLE_CHAR_UUID_RX, BLEWrite | BLEWriteWithoutResponse, 32);

// Velocity command buffer
char cmdBuffer[32];

// Helper: Clamp value to [-max, max]
int clampPWM(int value, int maxPWM) {
  if (value > maxPWM) return maxPWM;
  if (value < -maxPWM) return -maxPWM;
  return value;
}

// Differential drive kinematics: convert (vx, vy, wz) to left/right PWM
void velocityToPWM(float vx, float vy, float wz, int &leftPWM, int &rightPWM) {
  // For differential drive, vy is ignored
  float v_left = vx - (WHEEL_BASE / 2.0f) * wz;
  float v_right = vx + (WHEEL_BASE / 2.0f) * wz;

  // Convert to PWM (simple proportional mapping)
  // Max robot speed (m/s) that maps to MAX_SPEED PWM
  const float MAX_ROBOT_SPEED = 0.22; // e.g., TurtleBot3 max speed ////////////////////////CHECK THIS
  leftPWM = clampPWM((int)(v_left / MAX_ROBOT_SPEED * MAX_SPEED), MAX_SPEED);
  rightPWM = clampPWM((int)(v_right / MAX_ROBOT_SPEED * MAX_SPEED), MAX_SPEED);
}

// Parse CSV command: "vx,vy,wz\n"
bool parseVelocityCommand(const char* cmd, float &vx, float &vy, float &wz) {
  char buf[32];
  strncpy(buf, cmd, sizeof(buf));
  buf[sizeof(buf)-1] = '\0'; // Ensure null-termination

  char *token = strtok(buf, ",");
  if (!token) return false;
  vx = atof(token);

  token = strtok(NULL, ",");
  if (!token) return false;
  vy = atof(token);

  token = strtok(NULL, ",");
  if (!token) return false;
  wz = atof(token);

  return true;
}


// Print BLE MAC address to Serial (ArduinoBLE)
void printBleMacToSerial() {
  Serial.print("BLE MAC: ");
  Serial.println(BLE.address());
}


void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  setMotorSpeeds(0, 0);

  // BLE setup
  if (!BLE.begin()) {
    Serial.println("BLE failed to start!");
    while (1);
  }
  BLE.setDeviceName(BLE_DEVICE_NAME);
  BLE.setLocalName(BLE_DEVICE_NAME);
  BLE.setAdvertisedService(robotService);
  robotService.addCharacteristic(rxChar);
  BLE.addService(robotService);
  BLE.advertise();

  printBleMacToSerial();

  Serial.print("BLE ready: ");
  Serial.println(BLE_DEVICE_NAME);
  Serial.println("Waiting for velocity commands (vx,vy,wz)...");
}

// Add at the top (global scope)
unsigned long ignoreUntil = 0; // Time (millis) until which input is ignored

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to: "); Serial.println(central.address());
    setMotorSpeeds(0, 0);

    while (central.connected()) {
      // Check for new BLE command
      if (rxChar.written()) {
        // Ignore input if within ignore period
        if (millis() < ignoreUntil) {
          // Optionally print debug info:
          // Serial.println("Ignoring input due to cooldown.");
          continue;
        }

        int len = rxChar.valueLength();
        if (len > 0 && (size_t)len < sizeof(cmdBuffer)) {
          memcpy(cmdBuffer, rxChar.value(), len);
          cmdBuffer[len] = '\0';

          float vx = 0, vy = 0, wz = 0;
          if (parseVelocityCommand(cmdBuffer, vx, vy, wz)) {
            int leftPWM = 0, rightPWM = 0;
            velocityToPWM(vx, vy, wz, leftPWM, rightPWM);
            setMotorSpeeds(leftPWM, rightPWM);

            Serial.print("Received: "); Serial.print(cmdBuffer);
            Serial.print(" -> vx: "); Serial.print(vx, 2);
            Serial.print(", vy: "); Serial.print(vy, 2);
            Serial.print(", wz: "); Serial.println(wz, 2);
            Serial.print("PWM: L="); Serial.print(leftPWM); Serial.print(" R="); Serial.println(rightPWM);

            // If command is exactly 0.0,0.0,0.0, start ignore period
            if (vx == 0.0f && vy == 0.0f && wz == 0.0f) {
              ignoreUntil = millis() + 1000; // Ignore for 1 second
              Serial.println("Entering 1s ignore period.");
            }
          } else {
            Serial.print("Invalid command: "); Serial.println(cmdBuffer);
            setMotorSpeeds(0, 0);
          }
        }
      }
      delay(10); // BLE stack breathing room
    }
    Serial.println("Disconnected.");
    setMotorSpeeds(0, 0);
  }
  delay(100); // Idle wait before next connection
}

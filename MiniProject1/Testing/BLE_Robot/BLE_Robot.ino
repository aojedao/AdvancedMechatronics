/**
 * @file BLE_Robot.ino
 * @brief Main sketch for BLE-controlled differential drive robot with WASD and waypoint modes.
 * @details Initializes BLE services, handles manual WASD control and autonomous waypoint navigation,
 *          updates robot pose using encoder data, and drives motors with rotate-then-move logic.
 * @author [Your Name]
 * @date [Today's Date]
 */

#include "BLE_Robot.h"
#include "EncoderControl.h"
#include "MotorControl.h"

const float angle_wiggle_room = 5.0;     // Adjust value as needed
const float distance_wiggle_room = 10.0; // Adjust value as needed

/** @brief BLE service instance for encoder data transmission. */
BLEService encService(ENC_SERVICE_UUID);
/** @brief BLE service instance for receiving command data. */
BLEService commandService(COMMAND_SERVICE_UUID);
/** @brief BLE characteristic for reading/writing encoder position data. */
BLEFloatCharacteristic poseChar(POSE_CHAR_UUID, BLERead | BLEWrite);
/** @brief BLE characteristic for receiving X-coordinate waypoints. */
BLEFloatCharacteristic xDistChar(X_DIST_CHAR_UUID, BLERead | BLEWrite);
/** @brief BLE characteristic for receiving Y-coordinate waypoints. */
BLEFloatCharacteristic yDistChar(Y_DIST_CHAR_UUID, BLERead | BLEWrite);
/** @brief BLE characteristic for receiving WASD control commands. */
BLECharCharacteristic wasdChar(WASD_CHAR_UUID, BLERead | BLEWrite);

/** @brief Current X position of the robot in meters, used in waypoint mode. */
float xPos = 0.0;
/** @brief Current Y position of the robot in meters, used in waypoint mode. */
float yPos = 0.0;
/** @brief Current orientation of the robot in radians, used in waypoint mode. */
float theta = 0.0;
/** @brief Accumulated left encoder ticks, updated by ISR. */
volatile long leftTicks = 0;
/** @brief Accumulated right encoder ticks, updated by ISR. */
volatile long rightTicks = 0;
/** @brief Target X waypoint received via BLE, in meters. */
float goalX = 0.0;
/** @brief Target Y waypoint received via BLE, in meters. */
float goalY = 0.0;
/** @brief Flag indicating if the robot is in waypoint following mode. */
bool followingWaypoints = false;
/** @brief Flag indicating if the robot is in rotation phase of waypoint navigation. */
bool rotating = false;

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud for debugging
  while (!Serial);     // Wait for serial connection

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  pinMode(MOTOR_LEFT_PWM, OUTPUT);
  pinMode(MOTOR_LEFT_DIR_1, OUTPUT);
  pinMode(MOTOR_LEFT_DIR_2, OUTPUT);
  
  pinMode(MOTOR_RIGHT_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_1, OUTPUT);
  pinMode(MOTOR_RIGHT_DIR_2, OUTPUT);

  if (!BLE.begin()) {
    Serial.println("BLE failed!");
    while (1);
  }

  BLE.setLocalName("Robot_Arduino");
  BLE.setAdvertisedService(encService);
  encService.addCharacteristic(poseChar);
  commandService.addCharacteristic(xDistChar);
  commandService.addCharacteristic(yDistChar);
  commandService.addCharacteristic(wasdChar);
  BLE.addService(encService);
  BLE.addService(commandService);
  poseChar.writeValue(0.0);
  xDistChar.writeValue(0.0);
  yDistChar.writeValue(0.0);
  wasdChar.writeValue('S');  // Default to stop
  BLE.advertise();

  setupEncoders();

  Serial.println("BLE Robot Ready");
}

void loop() {
  BLEDevice central = BLE.central();
  
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());
    digitalWrite(LED_PIN, HIGH);

    while (central.connected()) {
      if (followingWaypoints) {
        updatePose();  // Update pose in waypoint mode
        poseChar.writeValue(leftTicks);  // Send encoder data
      }

      if (wasdChar.written()) {
        char command = wasdChar.value();
        followingWaypoints = false;  // Disable waypoint mode
        switch (command) {
          case 'W':
            setMotorSpeeds(MAX_SPEED, MAX_SPEED);
            Serial.println("Moving Forward");
            break;
          case 'S':
            setMotorSpeeds(0, 0);
            Serial.println("Stopped");
            break;
          case 'A':
            setMotorSpeeds(-MAX_SPEED * 0.5, MAX_SPEED);
            Serial.println("Turning Left");
            break;
          case 'D':
            setMotorSpeeds(MAX_SPEED, -MAX_SPEED * 0.5);
            Serial.println("Turning Right");
            break;
          default:
            setMotorSpeeds(0, 0);
            Serial.println("Unknown Command");
            break;
        }
      }

      if (xDistChar.written() || yDistChar.written()) {
        goalX = xDistChar.value();
        goalY = yDistChar.value();
        followingWaypoints = true;
        rotating = true;  // Start in rotation phase
        Serial.print("New waypoint: (");
        Serial.print(goalX);
        Serial.print(", ");
        Serial.print(goalY);
        Serial.println(")");
      }

      if (followingWaypoints) {
        float distError = sqrt(pow(goalX - xPos, 2) + pow(goalY - yPos, 2));
        float targetAngle = atan2(goalY - yPos, goalX - xPos);
        float angleError = targetAngle - theta;
        if (abs(angleError) > PI) angleError -= 2 * PI * (angleError > 0 ? 1 : -1);

        if (rotating) {  // Rotation phase
          if (abs(angleError) > angle_wiggle_room) {
            float thetaDot = K_THETA * angleError;
            thetaDot = constrain(thetaDot, -MAX_TURN_SPEED, MAX_TURN_SPEED);
            float vL = -WHEEL_BASE * thetaDot / 2;
            float vR = WHEEL_BASE * thetaDot / 2;
            vL = constrain(vL, -MAX_SPEED, MAX_SPEED);
            vR = constrain(vR, -MAX_SPEED, MAX_SPEED);
            setMotorSpeeds(vL, vR);
            Serial.print("Rotating, angle off: ");
            Serial.println(angleError);
          } else {
            rotating = false;  // Switch to move phase
            setMotorSpeeds(0, 0);
            Serial.println("Rotation complete, switching to move");
          }
        } else {  // Move phase
          if (distError > distance_wiggle_room) {
            float vB = K_V * distError;
            vB = constrain(vB, 0, MAX_SPEED);
            float vL = vB;
            float vR = vB;
            setMotorSpeeds(vL, vR);
            Serial.print("Moving, distance off: ");
            Serial.println(distError);
          } else {
            followingWaypoints = false;
            setMotorSpeeds(0, 0);
            Serial.println("Waypoint reached!");
          }
        }
      }
    }

    Serial.print("Disconnected from: ");
    Serial.println(central.address());
    digitalWrite(LED_PIN, LOW);
    setMotorSpeeds(0, 0);
    followingWaypoints = false;
    rotating = false;
  }
}

void updatePose() {
  static long prevLeftTicks = 0, prevRightTicks = 0;
  long deltaLeft = leftTicks - prevLeftTicks;
  long deltaRight = rightTicks - prevRightTicks;
  prevLeftTicks = leftTicks;
  prevRightTicks = rightTicks;

  float leftDist = (deltaLeft * 2 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
  float rightDist = (deltaRight * 2 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
  float dist = (leftDist + rightDist) / 2;
  float dTheta = (rightDist - leftDist) / WHEEL_BASE;

  xPos += dist * cos(theta + dTheta / 2);
  yPos += dist * sin(theta + dTheta / 2);
  theta += dTheta;
  theta = atan2(sin(theta), cos(theta));
}

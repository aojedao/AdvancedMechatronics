/**
 * @file MotorControlWithModesFunctions.ino
 * @brief Control differential drive robot with two modes via BLE on Arduino Nano 33 BLE.
 * @details Mode 1: WSADT manual; Mode 2: X,Y navigation—non-blocking functions, 'Q' exits.
 *          Left: enA 7, in1 9, in2 8; Right: enB 2, in3 4, in4 3; Encoders: Left 10,11, Right 5,6.
 */

#include <ArduinoBLE.h>
#include <Encoder.h>

// Encoder pins
Encoder leftEnc(10, 11);
Encoder rightEnc(5, 6);

// Motor pins
#define enA 7   // Left PWM
#define in1 9   // Left DIR 1
#define in2 8   // Left DIR 2
#define enB 2   // Right PWM
#define in3 4   // Right DIR 1
#define in4 3   // Right DIR 2

#define MAX_SPEED 128       // Adjusted PWM—try 255 if needed
#define ROTATION_TIME 1220  // ~1220 ms for ~1560 ticks (1 rev)
#define K_V 1.0             // Distance gain
#define K_THETA 1.5         // Orientation gain
#define ANGLE_WIGGLE_ROOM 0.05  // rad (~2.86°)
#define DISTANCE_WIGGLE_ROOM 0.01  // m

// Robot parameters
const float WHEEL_RADIUS = 0.0485;  // m (~1 ft circumference)
const float WHEEL_BASE = 0.2;       // m (assumed—adjust)
const float TICKS_PER_REV = 1560;   // 19.5:1 × 80

// Pose variables
float xPos = 0.0, yPos = 0.0, theta = 0.0;
float goalX = 0.0, goalY = 0.0;
bool navigating = false, rotating = false;
bool xReceived = false, yReceived = false;  // Flags for X, Y input
char mode = '0';  // Default mode (0x30)

// BLE setup
BLEService robotService("19B10000-E8F2-537E-4F6C-D104768A1214");  // Service UUID
BLECharCharacteristic modeChar("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);    // Mode select: '1' (0x31) or '2' (0x32)
BLECharCharacteristic commandChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);  // WASDTQ: 'W' (0x57), 'A' (0x41), 'S' (0x53), 'D' (0x44), 'T' (0x54), 'Q' (0x51)
BLEFloatCharacteristic xChar("0x2701", BLERead | BLEWrite);                                       // X coordinate (e.g., 2.0 as 0x40000000, -2.0 as 0xC0000000)
BLEFloatCharacteristic yChar("0x2702", BLERead | BLEWrite);                                       // Y coordinate (e.g., 2.0 as 0x40000000, -2.0 as 0xC0000000)

void setMotorSpeeds(int leftPWM, int rightPWM) {
  if (leftPWM >= 0) { digitalWrite(in1, HIGH); digitalWrite(in2, LOW); }
  else { digitalWrite(in1, LOW); digitalWrite(in2, HIGH); leftPWM = -leftPWM; }
  analogWrite(enA, leftPWM);
  if (rightPWM >= 0) { digitalWrite(in3, HIGH); digitalWrite(in4, LOW); }
  else { digitalWrite(in3, LOW); digitalWrite(in4, HIGH); rightPWM = -rightPWM; }
  analogWrite(enB, rightPWM);
}

void updatePose() {
  static long prevLeftTicks = 0, prevRightTicks = 0;
  long deltaLeft = leftEnc.read() - prevLeftTicks;
  long deltaRight = rightEnc.read() - prevRightTicks;
  prevLeftTicks = leftEnc.read();
  prevRightTicks = rightEnc.read();

  float leftDist = (deltaLeft * 2 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
  float rightDist = (deltaRight * 2 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
  float dist = (leftDist + rightDist) / 2;
  float dTheta = (rightDist - leftDist) / WHEEL_BASE;

  xPos += dist * cos(theta + dTheta / 2);
  yPos += dist * sin(theta + dTheta / 2);
  theta += dTheta;
  theta = atan2(sin(theta), cos(theta));
}

void navigateToGoal() {
  if (!navigating) return;

  long leftTicks = leftEnc.read();
  long rightTicks = rightEnc.read();
  float distError = sqrt(pow(goalX - xPos, 2) + pow(goalY - yPos, 2));
  float targetAngle = atan2(goalY - yPos, goalX - xPos);
  float angleError = targetAngle - theta;
  if (abs(angleError) > PI) angleError -= 2 * PI * (angleError > 0 ? 1 : -1);
  

  if (!rotating && distError < DISTANCE_WIGGLE_ROOM) {
    setMotorSpeeds(0, 0);
    navigating = false;
    Serial.println("Reached goal!");
    Serial.print("Final Ticks: "); Serial.print(leftTicks); Serial.print(" | "); Serial.println(rightTicks);
    Serial.print("Pose: "); Serial.print(xPos); Serial.print(", "); Serial.print(yPos); Serial.print(", "); Serial.println(theta);
    return;
  }

  if (rotating) {  // Rotate phase
    Serial.println("Rotating");  // Print every loop while rotating
    Serial.print("Theta error (rad): "); Serial.println(angleError);  // Print theta error
    if (abs(angleError) > ANGLE_WIGGLE_ROOM) {
      // Use full speed with direction based on error sign
      int direction = (angleError > 0) ? 1 : -1;
      setMotorSpeeds(-direction * MAX_SPEED, direction * MAX_SPEED);  // Full speed opposite directions
    } else {
      setMotorSpeeds(0, 0);
      rotating = false;
      Serial.println("Rotation aligned—moving forward");
    }
  } else {  // Move phase
    Serial.println("Moving forward");  // Print every loop while moving
    float vB = K_V * distError;
    vB = constrain(vB, 0, MAX_SPEED);  // Forward only
    setMotorSpeeds(vB, vB);  // Straight
  }

  updatePose();
  Serial.print("Ticks: "); Serial.print(leftTicks); Serial.print(" | "); Serial.println(rightTicks);
  Serial.print("Pose: "); Serial.print(xPos); Serial.print(", "); Serial.print(yPos); Serial.print(", "); Serial.println(theta);
}

// Function to select mode
void selectMode() {
  if (modeChar.written()) {
    mode = modeChar.value();
    Serial.print("Mode set to: "); Serial.print(mode); Serial.print(" (hex: "); Serial.print(mode, HEX); Serial.println(")");
    if (mode == '1') Serial.println("Waiting for WSADT...");
    else if (mode == '2') {
      Serial.println("Waiting for X,Y...");
      xReceived = false; yReceived = false;  // Reset flags
      Serial.println("Please enter X value");
    }
    else {
      Serial.println("Invalid mode—send '1' (0x31) or '2' (0x32)");
      mode = '0';  // Reset to no mode
    }
  }
}

// Function to handle WASDT commands
void handleWASDT() {
  if (commandChar.written()) {
    char command = commandChar.value();
    Serial.print("Received: "); Serial.print(command); Serial.print(" (hex: "); Serial.print(command, HEX); Serial.println(")");

    if (command == 'Q') {  // Exit on 'Q' (0x51)
      Serial.println("Exiting WASDT mode—returning to mode selection");
      mode = '0';  // Reset mode to trigger selectMode
      return;
    }

    leftEnc.write(0); rightEnc.write(0);
    switch (command) {
      case 'W': setMotorSpeeds(MAX_SPEED, MAX_SPEED); delay(ROTATION_TIME); break;  // Forward: 0x57
      case 'S': setMotorSpeeds(-MAX_SPEED, -MAX_SPEED); delay(ROTATION_TIME); break;  // Backward: 0x53
      case 'A': setMotorSpeeds(-MAX_SPEED, MAX_SPEED); delay(ROTATION_TIME); break;  // Left: 0x41
      case 'D': setMotorSpeeds(MAX_SPEED, -MAX_SPEED); delay(ROTATION_TIME); break;  // Right: 0x44
      case 'T': setMotorSpeeds(0, 0); break;  // Stop: 0x54
      default: Serial.println("Invalid command"); return;
    }
    setMotorSpeeds(0, 0);
    updatePose();
    Serial.print("Ticks: "); Serial.print(leftEnc.read()); Serial.print(" | "); Serial.println(rightEnc.read());
    Serial.print("Pose: "); Serial.print(xPos); Serial.print(", "); Serial.print(yPos); Serial.print(", "); Serial.println(theta);
  }
}

// Function to handle X, Y coordinates
void handleXY() {
  if (commandChar.written()) {
    char command = commandChar.value();
    Serial.print("Received: "); Serial.print(command); Serial.print(" (hex: "); Serial.print(command, HEX); Serial.println(")");
    if (command == 'Q') {  // Exit on 'Q' (0x51)
      Serial.println("Exiting X,Y mode—returning to mode selection");
      mode = '0';  // Reset mode to trigger selectMode
      return;
    }
  }

  if (xChar.written()) {
    goalX = xChar.value();
    xReceived = true;
    Serial.print("X value entered: "); Serial.print(goalX); 
    Serial.print(" (hex: "); Serial.print(*(uint32_t*)&goalX, HEX); Serial.println(")");
    if (!yReceived) Serial.println("Please enter Y value");
  }
  if (yChar.written()) {
    goalY = yChar.value();
    yReceived = true;
    Serial.print("Y value entered: "); Serial.print(goalY); 
    Serial.print(" (hex: "); Serial.print(*(uint32_t*)&goalY, HEX); Serial.println(")");
  }
  if (xReceived && yReceived) {
    Serial.print("Moving to: "); Serial.print(goalX); Serial.print(", "); Serial.println(goalY);
    leftEnc.write(0); rightEnc.write(0);
    navigating = true; rotating = true;
  }
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);
  setMotorSpeeds(0, 0);

  if (!BLE.begin()) {
    Serial.println("BLE failed!");
    while (1);
  }

  BLE.setLocalName("Robot_Arduino");
  BLE.setAdvertisedService(robotService);
  robotService.addCharacteristic(modeChar);
  robotService.addCharacteristic(commandChar);
  robotService.addCharacteristic(xChar);
  robotService.addCharacteristic(yChar);
  BLE.addService(robotService);
  
  modeChar.writeValue('0');    // Default: No mode (0x30)
  commandChar.writeValue('T'); // Default: Stop (0x54)
  xChar.writeValue(0.0);       // Default: 0.0 (0x00000000)
  yChar.writeValue(0.0);       // Default: 0.0 (0x00000000)
  BLE.advertise();

  Serial.println("BLE ready - Connect via LightBlue, set mode (1: WSADT, 2: X,Y)");
  Serial.println("Mode: '1' (0x31) or '2' (0x32); Commands: 'W' (0x57), 'A' (0x41), 'S' (0x53), 'D' (0x44), 'T' (0x54), 'Q' (0x51)");
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to: "); Serial.println(central.address());
    
    while (central.connected()) {
      if (mode == '0') selectMode();
      else if (mode == '1') handleWASDT();
      else if (mode == '2') {
        handleXY();
        if (navigating) navigateToGoal();  // Handle navigation outside tight loop
      }
      delay(100);  // Give BLE stack time to breathe
    }
    Serial.print("Disconnected from: "); Serial.println(central.address());
    setMotorSpeeds(0, 0); mode = '0'; navigating = false; rotating = false; xReceived = false; yReceived = false;
  }
  delay(100);  // Avoid tight loop when not connected
}

/**
 * @file hopethisworks.ino
 * @brief Control differential drive robot via BLE with WASDT commands on Arduino Nano 33 BLE.
 * @details WASDT manual control with 'Q' exit—sends "Theta: theta; X: x; Y: y" via BLE.
 *          Left: enA 7, in1 9, in2 8; Right: enB 2, in3 4, in4 3; Encoders: Left 10,11, Right 6,5.
 */

#include <ArduinoBLE.h>
#include <Encoder.h>

//1883
//1832
// Encoder pins
Encoder leftEnc(10, 11);  // Left encoder: Pin 10 (A), Pin 11 (B)
Encoder rightEnc(6, 5);   // Right encoder: Pin 6 (A), Pin 5 (B)

// Motor pins
#define enA 7   // Left PWM
#define in1 9   // Left DIR 1
#define in2 8   // Left DIR 2

#define enB 2   // Right PWM
#define in3 4   // Right DIR 1
#define in4 3   // Right DIR 2

#define MAX_SPEED 128       // Adjusted PWM—try 255 if needed
#define ROTATION_TIME 1220  // ~1220 ms for ~1560 ticks (1 rev)
#define LINEAR_TIME 1220  // ~1220 ms for ~1560 ticks (1 rev)

// Robot parameters
const float WHEEL_RADIUS = 0.0485;  // m (~1 ft circumference)
const float WHEEL_BASE = 0.17
;       // m (assumed—adjust)
const float TICKS_PER_REV = 1560;   // 19.5:1 × 80

// Pose variables
float xPos = 0.0, yPos = 0.0, theta = 0.0;

// BLE setup
BLEService robotService("19B10000-E8F2-537E-4F6C-D104768A1214");  // Service UUID
BLECharCharacteristic commandChar("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);  // WASDTQ: 'W' (0x57), 'A' (0x41), 'S' (0x53), 'D' (0x44), 'T' (0x54), 'Q' (0x51)
BLEStringCharacteristic poseChar("19B10003-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite, 40);  // "Theta: theta; X: x; Y: y", max 40 chars
BLEUnsignedIntCharacteristic rotTimeChar("19B10004-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEUnsignedIntCharacteristic linTimeChar("19B10005-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEUnsignedIntCharacteristic speedPWMChar("19B10006-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);

void setMotorSpeeds(int leftPWM, int rightPWM) {
  if (leftPWM >= 0) { 
    digitalWrite(in1, HIGH);
     digitalWrite(in2, LOW); 
  }else{
    digitalWrite(in1, LOW); 
    digitalWrite(in2, HIGH);
  }
  
  if (rightPWM >= 0) { 
    digitalWrite(in3, HIGH); 
    digitalWrite(in4, LOW); 
  }else{ 
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }

  Serial.print("LeftPWM: ");Serial.println(leftPWM);
  Serial.print("rightPWM: ");Serial.println(rightPWM);

  analogWrite(enA, abs(leftPWM));
  analogWrite(enB, abs(rightPWM));
}

void updatePose() {
  static long deltaLeft = 0, deltaRight = 0;

  deltaLeft = leftEnc.readAndReset();
  deltaRight = rightEnc.readAndReset();
  

  float leftDist = (deltaLeft * 2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;
  float rightDist = (deltaRight * 2.0 * PI * WHEEL_RADIUS) / TICKS_PER_REV;

  float dist = (leftDist + rightDist) / 2.0;
  float dTheta = (rightDist - leftDist) / WHEEL_BASE;

  xPos += dist * cos(theta + dTheta / 2.0);
  yPos += dist * sin(theta + dTheta / 2.0);

  theta += dTheta;
  theta = atan2(sin(theta), cos(theta)); // Normalizer Theta between -pi and pi

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
      updatePose();
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
    updatePose();
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

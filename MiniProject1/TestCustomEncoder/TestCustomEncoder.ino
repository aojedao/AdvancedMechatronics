/**
 * @file EncoderTestWithCustom.ino
 * @brief Test sketch for custom encoder on Arduino Nano 33 BLE using CustomEncoder.h.
 * @details Prints left and right encoder tick counts to Serial Monitor when wheels are rotated manually.
 *          Left encoder on pins 10 (A) and 11 (B), right encoder on pins 5 (A) and 6 (B).
 */

#include "CustomEncoder.h"

void setup() {
  Serial.begin(9600);
  while (!Serial);

  setupEncoders();

  Serial.println("Custom Encoder Test Ready - Rotate wheels manually to see tick counts");
  Serial.println("LeftTicks | RightTicks");
}

void loop() {
  static long prevLeftTicks = 0;
  static long prevRightTicks = 0;

  if (leftTicks != prevLeftTicks || rightTicks != prevRightTicks) {
    Serial.print(leftTicks);
    Serial.print(" | ");
    Serial.println(rightTicks);

    prevLeftTicks = leftTicks;
    prevRightTicks = rightTicks;
  }

  delay(100);  // Avoid flooding Serial
}

#ifndef ENCODER_CONTROL_H
#define ENCODER_CONTROL_H
/**
 * @file EncoderControl.h
 * @brief Header file for encoder interrupt handling and setup.
 * @details Defines interrupt service routines (ISRs) to track encoder ticks and
 *          a setup function to configure encoder pins and interrupts.
 */
#include <Arduino.h>
#include "BLE_Robot.h"

void leftEncoderISR() {
  if (digitalRead(ENC_LEFT_B) == digitalRead(ENC_LEFT_A)) {
    leftTicks++;
  } else {
    leftTicks--;
  }
  Serial.print("Encoder left update:");
  Serial.println(rightTicks);
}

void rightEncoderISR() {
  if (digitalRead(ENC_RIGHT_B) == digitalRead(ENC_RIGHT_A)) {
    rightTicks++;
  } else {
    rightTicks--;
  }
  Serial.print("Encoder right update:");
  Serial.println(rightTicks);
}

void setupEncoders() {
  pinMode(ENC_LEFT_A, INPUT);
  pinMode(ENC_LEFT_B, INPUT);
  pinMode(ENC_RIGHT_A, INPUT);
  pinMode(ENC_RIGHT_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), rightEncoderISR, CHANGE);
  Serial.println("Encoders ready");
}

#endif // ENCODER_CONTROL_H

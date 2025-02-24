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
}

void rightEncoderISR() {
  if (digitalRead(ENC_RIGHT_B) == digitalRead(ENC_RIGHT_A)) {
    rightTicks++;
  } else {
    rightTicks--;
  }
}

void setupEncoders() {
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderISR, CHANGE);
}

#endif // ENCODER_CONTROL_H

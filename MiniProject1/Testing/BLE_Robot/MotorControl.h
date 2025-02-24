#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/**
 * @file MotorControl.h
 * @brief Header file for motor control functions.
 * @details Provides a function to set motor speeds based on desired velocities,
 *          converting them to PWM signals for motor drivers.
 */

#include "BLE_Robot.h"

void setMotorSpeeds(float vL, float vR) {
  int leftPWM = abs(vL) * 255 / MAX_SPEED;
  int rightPWM = abs(vR) * 255 / MAX_SPEED;
  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  digitalWrite(MOTOR_LEFT_DIR, vL >= 0 ? HIGH : LOW);
  analogWrite(MOTOR_LEFT_PWM, leftPWM);
  digitalWrite(MOTOR_RIGHT_DIR, vR >= 0 ? HIGH : LOW);
  analogWrite(MOTOR_RIGHT_PWM, rightPWM);
}

#endif // MOTOR_CONTROL_H

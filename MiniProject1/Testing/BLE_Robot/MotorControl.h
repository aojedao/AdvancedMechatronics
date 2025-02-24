#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/**
 * @file MotorControl.h
 * @brief Header file for motor control functions.
 * @details Provides a function to set motor speeds based on desired velocities,
 *          converting them to PWM signals for motor drivers.
 *
 * L298N Motor Driver Control Table
 * --------------------------------
 * The L298N driver controls the direction of a DC motor based on the logic levels
 * applied to its input pins (Input1 and Input2). Below is the truth table for the motor:
 *
 * Input1   Input2   Spinning Direction
 * ------------------------------------
 * Low(0)   Low(0)   Motor OFF (No rotation)
 * High(1)  Low(0)   Forward (Clockwise rotation)
 * Low(0)   High(1)  Backward (Counter-clockwise rotation)
 * High(1)  High(1)  Motor OFF (Brake mode)
 *
 * Notes:
 * - Input1 and Input2 are connected to the L298N driver's control pins.
 * - The motor direction depends on the voltage applied to its terminals:
 *   - Forward: Terminal A is HIGH, Terminal B is LOW.
 *   - Backward: Terminal A is LOW, Terminal B is HIGH.
 * - When both inputs are HIGH or LOW, the motor stops (brake or coast mode).
 * - Ensure the L298N driver is properly powered and connected to avoid damage.
 */

#include "BLE_Robot.h"

void setMotorSpeeds(float vL, float vR) {
  int leftPWM = abs(vL) * 255 / MAX_SPEED;
  int rightPWM = abs(vR) * 255 / MAX_SPEED;
  leftPWM = constrain(leftPWM, 0, 255);
  rightPWM = constrain(rightPWM, 0, 255);

  if (vL != 0)
  {
    if (vL > 0)
    {
      digitalWrite(MOTOR_LEFT_DIR_1, HIGH);
      digitalWrite(MOTOR_LEFT_DIR_2, LOW);
    }
    else
    {
      digitalWrite(MOTOR_LEFT_DIR_1, LOW);
      digitalWrite(MOTOR_LEFT_DIR_2, HIGH);
    }
  }
  else
  {
      digitalWrite(MOTOR_LEFT_DIR_1, LOW);
      digitalWrite(MOTOR_LEFT_DIR_2, LOW);
  }

  if (vR != 0)
  {
    if (vR > 0)
    {
      digitalWrite(MOTOR_RIGHT_DIR_1, HIGH);
      digitalWrite(MOTOR_RIGHT_DIR_2, LOW);
    }
    else
    {
      digitalWrite(MOTOR_RIGHT_DIR_1, LOW);
      digitalWrite(MOTOR_RIGHT_DIR_2, HIGH);
    }
  }
  else
  {
      digitalWrite(MOTOR_RIGHT_DIR_1, LOW);
      digitalWrite(MOTOR_RIGHT_DIR_2, LOW);
  }

  // Set velocity for each motor.
  analogWrite(MOTOR_LEFT_PWM, leftPWM);
  analogWrite(MOTOR_RIGHT_PWM, rightPWM);
}

#endif // MOTOR_CONTROL_H

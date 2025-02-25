#ifndef CUSTOM_ENCODER_H
#define CUSTOM_ENCODER_H

/**
 * @file CustomEncoder.h
 * @brief Custom quadrature encoder handling for Arduino Nano 33 BLE with full interrupt support.
 * @details Uses interrupts on both A and B channels to track encoder ticks for left and right wheels,
 *          matching Pololu #3499 (20 CPR) with ~19.5:1 gear ratio, ~1560 ticks/wheel revolution.
 */

/**
 * @brief Pin definitions for encoder channels (Nano 33 BLE compatible).
 * @details Left: Pins 10 (A), 11 (B); Right: Pins 5 (A), 6 (B) - all interrupt-capable.
 */
#define ENC_LEFT_A  10
#define ENC_LEFT_B  11
#define ENC_RIGHT_A 5
#define ENC_RIGHT_B 6

/**
 * @brief Global variables for encoder tick counts.
 * @details Volatile to ensure ISR updates are not optimized out.
 */
volatile long leftTicks = 0;
volatile long rightTicks = 0;

/**
 * @brief State variables for quadrature decoding.
 * @details Track previous A/B states to determine direction—updated in ISRs.
 */
volatile byte leftPrevState = 0;
volatile byte rightPrevState = 0;

// Lookup table for quadrature state transitions (4-bit: prevState << 2 | newState)
const int8_t quadTable[16] = {
  0,   // 0000 - No change
  1,   // 0001 - Clockwise
  -1,  // 0010 - Counterclockwise
  0,   // 0011 - Invalid
  -1,  // 0100 - Counterclockwise
  0,   // 0101 - No change
  0,   // 0110 - Invalid
  1,   // 0111 - Clockwise
  1,   // 1000 - Clockwise
  0,   // 1001 - Invalid
  0,   // 1010 - No change
  -1,  // 1011 - Counterclockwise
  0,   // 1100 - Invalid
  -1,  // 1101 - Counterclockwise
  1,   // 1110 - Clockwise
  0    // 1111 - No change
};

/**
 * @brief ISR for left encoder channel A interrupt.
 * @details Updates leftTicks based on full quadrature state transition (A and B).
 */
void leftEncoderAISR() {
  byte newState = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  leftTicks += quadTable[(leftPrevState << 2) | newState];
  leftPrevState = newState;
}

/**
 * @brief ISR for left encoder channel B interrupt.
 * @details Updates leftTicks based on full quadrature state transition (A and B).
 */
void leftEncoderBISR() {
  byte newState = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  leftTicks += quadTable[(leftPrevState << 2) | newState];
  leftPrevState = newState;
}

/**
 * @brief ISR for right encoder channel A interrupt.
 * @details Updates rightTicks based on full quadrature state transition (A and B).
 */
void rightEncoderAISR() {
  byte newState = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  rightTicks += quadTable[(rightPrevState << 2) | newState];
  rightPrevState = newState;
}

/**
 * @brief ISR for right encoder channel B interrupt.
 * @details Updates rightTicks based on full quadrature state transition (A and B).
 */
void rightEncoderBISR() {
  byte newState = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);
  rightTicks += quadTable[(rightPrevState << 2) | newState];
  rightPrevState = newState;
}

/**
 * @brief Configures encoder pins and attaches interrupts.
 * @details Sets pins as inputs with pull-ups, attaches ISRs to both A and B channels for full quadrature.
 */
void setupEncoders() {
  pinMode(ENC_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_LEFT_B, INPUT_PULLUP);
  pinMode(ENC_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_RIGHT_B, INPUT_PULLUP);

  // Initialize previous states
  leftPrevState = (digitalRead(ENC_LEFT_A) << 1) | digitalRead(ENC_LEFT_B);
  rightPrevState = (digitalRead(ENC_RIGHT_A) << 1) | digitalRead(ENC_RIGHT_B);

  // Attach interrupts to all A and B pins (Nano 33 BLE supports interrupts on all pins)
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), leftEncoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_LEFT_B), leftEncoderBISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), rightEncoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_B), rightEncoderBISR, CHANGE);
}

#endif // CUSTOM_ENCODER_H

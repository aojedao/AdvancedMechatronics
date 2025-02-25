/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder leftEnc(10, 11);
Encoder rightEnc(6, 5);


//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");
}

long oldPosition_left  = -999;
long oldPosition_right  = -999;

void loop() {
 long newPosition_left = leftEnc.read();
  long newPosition_right = rightEnc.read();

  
  if (newPosition_left != oldPosition_left || newPosition_right != oldPosition_right) {
    oldPosition_left = newPosition_left;
    Serial.print("l: ");
    Serial.print(oldPosition_left);
    Serial.print(" | r: ");
    oldPosition_right = newPosition_right;
    Serial.println(oldPosition_right);
  }
}

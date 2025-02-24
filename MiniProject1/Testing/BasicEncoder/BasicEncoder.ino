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
//Encoder leftEnc(5, 6);
//Encoder rightEnc(10, 11);


//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("Basic Encoder Test:");

  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(10, INPUT);
  pinMode(11, INPUT);
}

long oldPosition_left  = -999;
long oldPosition_right  = -999;

void loop() {
 // long newPosition_left = leftEnc.read();
  //long newPosition_right = rightEnc.read();

  Serial.print(digitalRead(5));
  Serial.print(" ");
    Serial.print(digitalRead(6));
  Serial.print(" ");
    Serial.print(digitalRead(10));
  Serial.print(" ");
      Serial.print(digitalRead(11));
  Serial.println(" ");
  

//  Serial.print(newPosition_left);
  //Serial.print(" ");
 // Serial.println(newPosition_right);

  /*
  if (newPosition_left != oldPosition_left) {
    oldPosition_left = newPosition_left;
    Serial.println(oldPosition_left);
  }

    if (newPosition_right != oldPosition_right) {
    oldPosition_right = newPosition_right;
    Serial.println(oldPosition_right);
  }*/
  //Serial.println("FUCK");
}

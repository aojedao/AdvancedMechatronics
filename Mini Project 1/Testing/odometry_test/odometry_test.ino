#include <Encoder.h>	//use the Encoder library

Encoder encoder(2,3);	//create the encoder (always use pins 2 and 3 on UNO)

long position = 0;	//current motor position (10 steps per revolution)
int rotation = 0;	//current number of revolutions from 0 point

void setup()
{
  Serial.begin(9600);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
}


void loop()
{
  //spin the motor
  analogWrite(5, 30);
  analogWrite(6,0);
  
  //get the current encoder position
 
  position = encoder.read() / 10;
  
  //calculate the number of complete rotations
  rotation = abs(position) / 10;
  
  //print results
  Serial.print("pos: ");
  Serial.println(position);
  Serial.print("rot: ");
  Serial.println(rotation);
}
#include <arduino.h>
#include "config.h"
#include "motorDriver.hpp"

void setMotorSpeeds(int leftPWM, int rightPWM) {
  if(leftPWM != 0){
    if (leftPWM > 0) { 
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW); 
    }else{
      digitalWrite(in1, LOW); 
      digitalWrite(in2, HIGH);
    }
  }
  else
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
    
  if(rightPWM != 0)
  {
    if (rightPWM > 0) { 
      digitalWrite(in3, HIGH); 
      digitalWrite(in4, LOW); 
    }else{ 
      digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);
    }
  }
  else
  {
    digitalWrite(in3, LOW); 
    digitalWrite(in4, LOW);
  }


  Serial.print("LeftPWM: ");Serial.println(leftPWM);
  Serial.print("rightPWM: ");Serial.println(rightPWM);

  analogWrite(enA, abs(leftPWM));
  analogWrite(enB, abs(rightPWM));
}
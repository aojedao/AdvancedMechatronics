//1A.

// C++ code
//
const byte push_button = 8;
const byte LED = 9;
void setup()
{
  pinMode(push_button, INPUT);//setup P8 as input pin
  pinMode(LED,OUTPUT);
  Serial.begin(9600);
}

void loop()
{
  int reading = digitalRead(push_button);
  if(reading) digitalWrite(LED,LOW);
  else digitalWrite(LED,HIGH);
}

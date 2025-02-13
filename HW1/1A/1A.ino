//using pin mode,digitalWrite() and delay function()
//Sketch uses 924 bytes (3%) of program storage space. Maximum is 30720 bytes.
//Global variables use 9 bytes (0%) of dynamic memory, leaving 2039 bytes for local variables. Maximum is 2048 bytes.

const byte led = 8;
void setup() {
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(led, !digitalRead(led));
  delay(500); 

}

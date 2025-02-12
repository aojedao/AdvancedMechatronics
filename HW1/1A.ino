//using pin mode,digitalWrite() and delay function()
//Sketch uses 924 bytes (3%) of program storage space. Maximum is 30720 bytes.
//Global variables use 9 bytes (0%) of dynamic memory, leaving 2039 bytes for local variables. Maximum is 2048 bytes.

const byte led = 8;
void setup() {
  // put your setup code here, to run once:
  DDRB |= (1<DDB0);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  PORTB ^= (1<<PB0);
  delay(500); 

}

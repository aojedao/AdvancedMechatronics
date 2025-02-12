
const int interval = 1000;
byte shiftRegister = 0b00000000;
int prev_toggle = 0;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  shiftRegister |= (1 << 0);  // Set bit 0
  shiftRegister |= (1 << 3);  // Set bit 3
  shiftRegister |= (1 << 6);  // Set bit 6

}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis() - prev_toggle > interval){
    shiftRegister ^= (1 << 4); //toggle shift register using XOR 
    prev_toggle = millis();
  }
  shiftRegister &= ~((1 << 2) | (1 << 5));
  Serial.println(shiftRegister);

}

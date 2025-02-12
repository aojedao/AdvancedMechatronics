
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  DDRB |= (1<<DDB0);
  PORTB ^= (1<<PORTB0);
}

void loop() {
  // put your main code here, to run repeatedly:
  char HL;
  while(true){
    Serial.print("Turn ON (H) or Turn OFF (L) the led : ");
    while(Serial.available() == 0){}
    HL = Serial.read();
    Serial.println(HL);
    if(HL == 'h' || HL == 'H'){
      PORTB |= (1<<PORTB0);
    }
    else if (HL == 'L' || HL == 'l'){
      PORTB &= ~(1<<PORTB0);
    }

    else Serial.println("Invalid Input");

  }

}

void setup() {
  // put your setup code here, to run once:
  DDRB |= ((1<<DDB0) | (1<<DDB1));//set led8 and led9 to output pins
  DDRD &= ~(1<<DDD7); //set pin7 to output push_button
  PORTD |= (1<<PORTD7); //pullup portd
  //PORTB ^= (1<<PORTB0); //when button is not pressed led8 glows and when pushbutton is pressed led8 off and led9 glows

}

void loop() {
  // put your main code here, to run repeatedly:
	PORTB = (PIND & (1<<PIND7))?(1<<PORTB0):(1<<PORTB1);
}

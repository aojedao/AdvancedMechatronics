//1B
void setup(){
  //set pin9 to input 
  DDRB |= (1<<PB1);//set pin9 as outputpin
  PORTB |= (1<<PB0);//pull up internal resistor to high
}

void loop(){
	PORTB = (PINB & (1<<PB0)) ? (PORTB | (1<<PB1)):(PORTB & ~(1<<PB1));
}


void setup() {
  // Configure buttons with pull-ups
  DDRD &= ~(1 << PD4); // P4 (input)
  PORTD |= (1 << PD4);
  DDRB &= ~(1 << PB2); // P10 (input)
  PORTB |= (1 << PB2);
  DDRC &= ~(1 << PC3); // A3 (input)
  PORTC |= (1 << PC3);
  
  // Configure LEDs (P5, P6, P7)
  DDRD |= (1 << PD5) | (1 << PD6) | (1 << PD7);
  
  // Enable pin change interrupts
  PCICR |= (1 << PCIE2) | (1 << PCIE0) | (1 << PCIE1);
  PCMSK2 |= (1 << PCINT20); // P4 (D4)
  PCMSK0 |= (1 << PCINT2);  // P10 (B2)
  PCMSK1 |= (1 << PCINT11); // A3 (C3)
  sei();
}

void loop(){
// Do nothing
}

ISR(PCINT2_vect) { // Port D (P4)
  if (!(PIND & (1 << PD4))) PORTD ^= (1 << PD5);
}

ISR(PCINT0_vect) { // Port B (P10)
  if (!(PINB & (1 << PB2))) PORTD ^= (1 << PD6);
}

ISR(PCINT1_vect) { // Port C (A3)
  if (!(PINC & (1 << PC3))) PORTD ^= (1 << PD7);
}
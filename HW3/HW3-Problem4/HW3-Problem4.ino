// Adv. Mech Spring 2025
// Team 3 Santiago Bernheim - Alejandro Ojeda - Nishant Pushparaju
// HW3 Problem 4

void setup() {
  DDRB |= (1 << DDB5); // Built-in LED (PB5) as output
  
  // Timer1 settings
  TCCR1A = 0; // Normal mode
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10); // CTC, prescaler 1024
  OCR1A = 7812; // 0.5s interval
  TIMSK1 = (1 << OCIE1A); // Enable interrupt
  sei(); // Enable global interrupts
}

void loop() {}

ISR(TIMER1_COMPA_vect) {
  PORTB ^= (1 << PB5); // Toggle LED
}
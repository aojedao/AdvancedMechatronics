// Adv. Mech Spring 2025
// Team 3 Santiago Bernheim - Alejandro Ojeda - Nishant Pushparaju
// HW3 Problem 2
// Blinking LED on Pin 4 using bit(), bitSet(), bitClear()

void setup() {
  DDRD |= bit(DDD4); // Set pin 4 (PORTD4) as output
}

void loop() {
  bitSet(PORTD, PD4); // Turn LED on
  delay(500);
  bitClear(PORTD, PD4); // Turn LED off
  delay(500);
}
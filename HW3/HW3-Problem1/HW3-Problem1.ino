
// Adv. Mech Spring 2025
// Team 3 Santiago Bernheim - Alejandro Ojeda - Nishant Pushparaju
// HW3 Problem 1
// Button Counter with interrupts.

volatile int count = 0;

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), isr, FALLING);
}

void loop() {
  // Empty loop; everything handled in ISR
}

void isr() {
  count++;
  Serial.println(count);
}
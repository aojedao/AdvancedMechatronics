// Adv. Mech Spring 2025
// Team 3 Santiago Bernheim - Alejandro Ojeda - Nishant Pushparaju
// HW3 Problem 5

volatile bool flag = false;
volatile unsigned long last = 0;

void setup() {
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), isr, FALLING);
  pinMode(8, OUTPUT);
}

void loop() {
  if (flag && (millis() - last >= 200)) {
    PORTB ^= (1 << PB0); // Toggle LED (PB0)
    flag = false;
    last = millis();
  }
}

void isr() {
  if (millis() - last >= 200) {
    flag = true;
  }
}
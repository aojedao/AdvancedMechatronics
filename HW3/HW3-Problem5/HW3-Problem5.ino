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
    last = millis(); // Save the last time that the led was toggled. to avoid toggling continuous 
  }
}

void isr() {
  // We are only allowing to set the flag if last time was change was 200miliseconds ago.
  // Debouncing is necessary  because of the mechanical contact the signal goes low multiple times for a single push.
  if (millis() - last >= 200) { 
    flag = true;
  }
}
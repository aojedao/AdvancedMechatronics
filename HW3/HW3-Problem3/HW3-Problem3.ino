// Adv. Mech Spring 2025
// Team 3 Santiago Bernheim - Alejandro Ojeda - Nishant Pushparaju
// HW3 Problem 3 - Serial communication using registers.

void setup() {
  // USART initialization
  UBRR0H = 0; // Baud rate 9600 for 16MHz
  UBRR0L = 103;
  UCSR0B |= (1 << TXEN0); // Enable transmitter
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data
}

void loop() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last >= 1000) {
    last = now;
    sendNumber(now);
  }
}

void sendNumber(unsigned long num) {
  char buffer[10];
  int i = 0;
  do {
    buffer[i++] = num % 10 + '0';
    num /= 10;
  } while (num > 0);
  
  for (int j = i - 1; j >= 0; j--) {
    while (!(UCSR0A & (1 << UDRE0))); // Wait for buffer
    UDR0 = buffer[j];
  }
}
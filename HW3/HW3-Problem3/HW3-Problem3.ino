// Adv. Mech Spring 2025
// Team 3 Santiago Bernheim - Alejandro Ojeda - Nishant Pushparaju
// HW3 Problem 3 - Serial communication using registers.

void setup() {
  // USART initialization
  UBRR0H = 0; // Set upper 4 bits of UBRR to 0
  UBRR0L = 103; // Set lower 8 bits of UBRR to 103 (9600 baud)
  UCSR0B |= (1 << TXEN0); // Enable transmitter
  UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00); // 8-bit data frame
}

void loop() {
  static unsigned long last = 0; // Store the last time we printed
  unsigned long now = millis(); // Get current runtime in milliseconds
  if (now - last >= 1000) { // Check if 1 second has passed
    last = now; // Update last print time
    sendNumber(now); // Send runtime to USART
  }
}

void sendNumber(unsigned long num) {
  char buffer[10]; // Buffer to store digits
  int i = 0;
  do {
    buffer[i++] = num % 10 + '0'; // Convert digit to ASCII
    num /= 10; // Move to the next digit
  } while (num > 0); // Repeat until all digits are processed
  
  for (int j = i - 1; j >= 0; j--) { // Send digits in reverse order
    while (!(UCSR0A & (1 << UDRE0))); // Wait for UDR0 to be empty
    UDR0 = buffer[j]; // Send digit
  }
}
const byte led = 8;
char HL;

void setup() {
  Serial.begin(9600);
  DDRB |= (1 << DDB0);        // Set pin 8 as output
  PORTB &= ~(1 << PORTB0);    // Initialize LED OFF (comment out if inverse logic)
  Serial.println("Enter H/h to turn ON or L/l to turn OFF:");
}

void loop() {
  if (Serial.available() > 0) {
    HL = Serial.read();
    
    // Skip line endings (ASCII 10 = \n, 13 = \r)
    if (HL == 10 || HL == 13) return;
    
    if (HL == 'H' || HL == 'h') {
      PORTB |= (1 << PORTB0);  // LED ON
      Serial.println("LED ON");
    } 
    else if (HL == 'L' || HL == 'l') {
      PORTB &= ~(1 << PORTB0); // LED OFF
      Serial.println("LED OFF");
    } 
    else {
      Serial.println("Invalid input! Use H/h or L/l for ON and OFF respectively.");
    }

    // Clear remaining characters in buffer
    while (Serial.available() > 0) Serial.read();
  }
}
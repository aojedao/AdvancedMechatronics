#include "PAR27979.h" // Assuming Parallax LCD uses this library (check your LCD documentation)
#include <math.h>

PAR27979 display(&Serial);

// Button connections
const int btn0 = 8; // Shift digit
const int btn1 = 9; // Increment digit
const int btn2 = 10; // Finalize input

// Variables
int digits[4] = {0, 0, 0, 0};
int currentDigit = 0;
bool inputMode = true;
unsigned long startTime;

void setup() {
  Serial.begin(9600); // Baud rate for LCD communication (check your LCD's datasheet. Parallax serial LCDs usually are 9600 or 19200)
  display.clearHome();
  
  display.print("Enter number:");

  // Set button pins as inputs with pull-up resistors
  pinMode(btn0, INPUT_PULLUP);
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
}

void loop() {
  if (inputMode) {
    handleInput();
  } else {
    displayResult();
  }
}

void handleInput() {
  if (digitalRead(btn0) == LOW) { // Shift digit
    delay(50); // Debouncing
    display.right();
  }

  if (digitalRead(btn1) == LOW) { // Increment digit
    delay(50); // Debouncing
    digits[currentDigit] = (digits[currentDigit] + 1) % 10;
  }
  display.print(digits[0]);
  /*
  if (digitalRead(btn2) == LOW) { // Finalize input
    delay(50); // Debouncing
    inputMode = false;
    startTime = millis();
  }
  */

  displayInput();
}

void displayInput() {
  /*
  for (int i = 0; i < 4; i++) {
    display.write(digits[i]);
    if (i == 1) display.write('.');
  }
  */
  //display.print(digits);
}

void displayResult() {
  float number = digits[0] * 10 + digits[1] + digits[2] * 0.1 + digits[3] * 0.01;
  float result = number;

  // Round to two decimal places
  result = round(result * 100.0) / 100.0;

  display.clearHome();
  display.print("Sqrt: ");
  display.print(result);

  if (millis() - startTime > 3000) {
    inputMode = true;
    display.clearHome();
    display.print("Enter number:");
    currentDigit = 0;
    for (int i = 0; i < 4; i++) digits[i] = 0; // Reset digits
  }
}

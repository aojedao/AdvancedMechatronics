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
  // Set button pins as inputs with pull-up resistors
  pinMode(btn0, INPUT_PULLUP);
  pinMode(btn1, INPUT_PULLUP);
  pinMode(btn2, INPUT_PULLUP);
  Serial.begin(9600); // Baud rate for LCD communication (check your LCD's datasheet. Parallax serial LCDs usually are 9600 or 19200)

  initializeScreen();
}

void loop() {
  if (inputMode) {
    handleInput();
  } else {
    displayResult();
  }
}

void handleInput() {

  if (digitalRead(btn2) == LOW) { // Finalize input
    delay(50); // Debouncing
    if (digitalRead(btn2) == LOW) { 
      inputMode = false;
      startTime = millis();
    }
  } else  if (digitalRead(btn0) == LOW) { // Shift digit
    delay(50); // Debouncing
    if (digitalRead(btn0) == LOW){
      currentDigit++;
      display.right();
      if (currentDigit > 3)
      {
        currentDigit = currentDigit % 4;
        for(int i = 0;  i< 5 ; i++) 
        {
            display.left();
        }
      } else if (currentDigit == 2)
      {
        display.right();
      }
      
      
      delay(100);
    }
  } else if (digitalRead(btn1) == LOW) { // Increment digit
    delay(50); // Debouncing
    if (digitalRead(btn1) == LOW) { 
      digits[currentDigit] = (digits[currentDigit] + 1) % 10;
      display.print(digits[currentDigit]);
      display.left();
      delay(100);
    }
  }
}

void displayResult() {
  float number = digits[0] * 10 + digits[1] + digits[2] * 0.1 + digits[3] * 0.01;
  float result = sqrt(number);

  // Round to two decimal places
  result = round(result * 100.0) / 100.0;

  display.clearHome();
  display.print("Sqrt: ");
  display.print(result);

  delay(5000);

  inputMode = true;
  currentDigit = 0;
  initializeScreen();
  for (int i = 0; i < 4; i++) digits[i] = 0; // Reset digits

}

void initializeScreen(){
  display.noSound();
  display.clearHome();
  display.print("Enter number:     00.00");
  for(int i = 0;  i< 5 ; i++) 
  {
      display.left();
  }
}

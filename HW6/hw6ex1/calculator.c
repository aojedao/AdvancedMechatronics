// calculator.c

#include "simpletools.h" // Include SimpleIDE library for terminal I/O
#include "functions.h"   // Include the functions defined in functions.c

int main() {
    int a, b, choice; // Variables to store user input
    float result;     // Variable to store the result of the calculation

    while(1)
    {
      // Prompt the user to enter two numbers
      print("Enter two numbers (e.g., 5 3):\n");
      scan("%d %d", &a, &b);
  
      // Prompt the user to choose an operation
      print("Choose an operation:\n");
      print("1 - Add\n");
      print("2 - Subtract\n");
      print("3 - Multiply\n");
      print("4 - Divide\n");
      scan("%d", &choice);
  
      // Perform the chosen operation
      switch (choice) {
          case 1:
              result = add(a, b); // Call the add function
              print("Result: %f\n", result);
              break;
          case 2:
              result = subtract(a, b); // Call the subtract function
              print("Result: %f\n", result);
              break;
          case 3:
              result = multiply(a, b); // Call the multiply function
              print("Result: %f\n", result);
              break;
          case 4:
              if (b != 0) {
                  result = divide(a, b); // Call the divide function
                  print("Result: %f\n", result);
              } else {
                  print("Error: Division by zero is not allowed.\n");
              }
              break;
          default:
              print("Invalid choice. Please select a valid operation (1-4).\n");
              break;
      }

    }    
    return 0; // End of program
}

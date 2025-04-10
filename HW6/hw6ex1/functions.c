// functions.c

#include "functions.h" // Include the header file

// Function definitions
int add(int a, int b) {
    return a + b;
}

int subtract(int a, int b) {
    return a - b;
}

int multiply(int a, int b) {
    return a * b;
}

float divide(int a, int b) {
    if (b != 0) {
        return (float)a / b; // Perform division and return a float
    } else {
        return 0; // Handle division by zero
    }
}

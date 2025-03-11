#include "simpletools.h"

int main() {
    DIRA |= (1 << 26) | (1 << 27); // Set P26 and P27 as outputs
    
    while(1) {
        OUTA |= (1 << 26) | (1 << 27); // Turn on LEDs
        print("DIRA: %b, OUTA: %b\n", DIRA, OUTA); // Print states
        pause(1000);                   // Wait for 1 second
        OUTA &= ~((1 << 26) | (1 << 27)); // Turn off LEDs
        print("DIRA: %b, OUTA: %b\n", DIRA, OUTA); // Print states
        pause(1000);                   // Wait for 1 second
    }
}
#include "simpletools.h"

int main() {
    DIRA |= (1 << 26) | (1 << 27); // Set P26 and P27 as outputs
    
    while(1) {
        OUTA |= (1 << 26) | (1 << 27); // Turn on LEDs
        pause(500);                     // Wait for 500 ms
        OUTA &= ~((1 << 26) | (1 << 27)); // Turn off LEDs
        pause(500);                     // Wait for 500 ms
    }
}
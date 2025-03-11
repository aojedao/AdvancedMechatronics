#include "simpletools.h"

int main() {
    set_direction(14, 0); // Set P14 as input (Button B1)
    set_direction(15, 0); // Set P15 as input (Button B2)
    set_direction(26, 1); // Set P26 as output (LED)
    set_direction(27, 1); // Set P27 as output (LED)
    
    while(1) {
        if (input(14)) {  // If Button B1 is pressed
            toggle(26);   // Toggle LED on P26
            pause(300);   // Debounce delay
        }
        if (input(15)) {  // If Button B2 is pressed
            toggle(27);   // Toggle LED on P27
            pause(300);   // Debounce delay
        }
    }
}
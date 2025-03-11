#include "simpletools.h"

int main() {
    set_direction(26, 1); // Set P26 as output
    set_direction(27, 1); // Set P27 as output
    
    while(1) {
        toggle(26);       // Toggle LED on P26
        toggle(27);       // Toggle LED on P27
        pause(500);       // Wait for 500 ms
    }
}
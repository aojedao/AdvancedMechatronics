#include "simpletools.h"

int main() {
    while(1) {
        high(26);          // Turn on LED on P26
        pause(200);        // Wait for 200 ms
        low(26);           // Turn off LED on P26
        pause(500);        // Wait for 500 ms
    }
} 
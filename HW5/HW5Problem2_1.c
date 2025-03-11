#include "simpletools.h"

int main() {
    while(1) {
        high(26);          // Turn on LED on P26
        high(27);          // Turn on LED on P27
        pause(500);        // Wait for 500 ms
        low(26);           // Turn off LED on P26
        low(27);           // Turn off LED on P27
        pause(500);        // Wait for 500 ms
    }
}
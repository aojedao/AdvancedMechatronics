#include "simpletools.h"
#include "servo.h"

int main() {
    set_direction(0, 0); // Set P0 as input (Button B0)
    set_direction(1, 0); // Set P1 as input (Button B1)
    
    int angle = 1500;    // Neutral position (1.5 ms pulse)
    
    while(1) {
        if (input(0)) {  // If Button B0 is pressed
            angle += 100; // Increase angle (clockwise)
            if (angle > 2000) angle = 2000; // Limit to 2000
            servo_set(14, angle); // Set servo angle
            pause(300); // Debounce delay
        }
        if (input(1)) {  // If Button B1 is pressed
            angle -= 100; // Decrease angle (counterclockwise)
            if (angle < 1000) angle = 1000; // Limit to 1000
            servo_set(14, angle); // Set servo angle
            pause(300); // Debounce delay
        }
    }
}
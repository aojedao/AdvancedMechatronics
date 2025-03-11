#include "simpletools.h"
#include "servo.h"

int main() {
    set_direction(0, 0); // Set P0 as input (Button B0)
    set_direction(1, 0); // Set P1 as input (Button B1)
    
    int angle = 90;      // Neutral position (90 degrees)
    
    while(1) {
        if (input(0)) {  // If Button B0 is pressed
            angle += 10; // Increase angle (clockwise)
            if (angle > 180) angle = 180; // Limit to 180
            servo_angle(14, angle); // Set servo angle
            pause(300); // Debounce delay
        }
        if (input(1)) {  // If Button B1 is pressed
            angle -= 10; // Decrease angle (counterclockwise)
            if (angle < 0) angle = 0; // Limit to 0
            servo_angle(14, angle); // Set servo angle
            pause(300); // Debounce delay
        }
    }
}
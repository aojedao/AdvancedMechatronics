#include "simpletools.h"

int main() {
    while(1) {
        int time = rc_time(7, 1); // Measure RC time on P7
        int angle = time / 20;    // Scale to degrees (adjust scaling factor as needed)
        print("Potentiometer Angle: %d degrees\n", angle);
        pause(1000);              // Wait for 1 second
    }
}
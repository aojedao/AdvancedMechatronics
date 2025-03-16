#include "simpletools.h"

int main() {
  set_direction(7,1);
    while(1) {
        high(7);
        pause(100);
        int time = rc_time(7, 1); // Measure RC time on P7
        int angle = time*180/249987;    // Scale to degrees (adjust scaling factor as needed)
        print("Potentiometer Angle: %d degrees\n", angle);
        pause(1000);              // Wait for 1 second
    }
}
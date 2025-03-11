#include "simpletools.h"

int main() {
    int on_time, off_time;
    
    print("Enter ON time (ms): ");
    scan("%d", &on_time);
    print("Enter OFF time (ms): ");
    scan("%d", &off_time);
    
    while(1) {
        high(26);          // Turn on LED on P26
        pause(on_time);    // Wait for user-defined ON time
        low(26);           // Turn off LED on P26
        pause(off_time);   // Wait for user-defined OFF time
    }
}
#include "simpletools.h"

volatile int counter = 1000;
int countArray[300];

void incrementCount() {
    for (int i = 0; i < 300; i++) {
        pause(1000); // Wait 1 second.
        counter++;
    }
}

int main() {
    cog_run(incrementCount, 128);

    for (int i = 0; i < 300; i++) {
        pause(1000); // Wait 1 second.
        countArray[i] = counter;
    }

    for (int i = 0; i < 300; i++) {
        print("Count[%d] = %d\n", i, countArray[i]);
    }

    return 0;
}

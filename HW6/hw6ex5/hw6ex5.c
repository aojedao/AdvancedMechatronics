#include "simpletools.h"

void blinkP26() {
    while (1) {
        high(26);
        pause(500);
        low(26);
        pause(500);
    }
}

void blinkP27() {
    while (1) {
        high(27);
        pause(250);
        low(27);
        pause(250);
    }
}

int main() {
    cog_run(blinkP26, 128);
    cog_run(blinkP27, 128);
    while (1);
    return 0;
}

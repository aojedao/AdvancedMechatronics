#include "simpletools.h"
#include "eeprom.h"

int main() {
    int freq, duration, addr = 0x8000;
    for (int i = 0; i < 5; i++) {
        print("Enter frequency and duration for tone %d:\n", i + 1);
        scan("%d %d", &freq, &duration);

        ee_putInt(freq, addr);
        addr += 2; // Increment address for next frequency.
        ee_putInt(duration, addr);
        addr += 2; // Increment address for next duration.
    }

    addr = 0x8000;
    for (int i = 0; i < 5; i++) {
        freq = ee_getInt(addr);
        addr += 2;
        duration = ee_getInt(addr);
        addr += 2;

        freqout(0, duration, freq);
    }
    return 0;
}

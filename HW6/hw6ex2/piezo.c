#include "simpletools.h"

#define BUZZER_PIN 8

#define N_TONES 5

int main() {
  while(1)
  {
    int freq, duration, addr = 0x8000;
    for (int i = 0; i < N_TONES; i++) {
        print("Enter frequency and duration for tone %d:\n", i + 1);
        scan("%d %d", &freq, &duration);

        ee_putInt(freq, addr);
        addr += 4; // Increment address for next frequency.
        ee_putInt(duration, addr);
        addr += 4; // Increment address for next duration.
    }

    addr = 0x8000;
    for (int i = 0; i < N_TONES; i++) {
        print("Getting values from EEPROM from %d\n", i);
        freq = ee_getInt(addr);
        addr += 4;
        duration = ee_getInt(addr);
        addr += 4;
        print("Values loaded from EPROM freq: %d duration: %d:\n", freq, duration);

        freqout(BUZZER_PIN, duration, freq);
    }
    print("Finished\n");
  }    
}

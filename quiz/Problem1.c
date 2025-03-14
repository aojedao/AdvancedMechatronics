#include "simpletools.h"

int main() {
    int mask = (0b0011);
    DIRA |= mask;
    pause(1000);
    
    while(1) {
      
      OUTA = (OUTA & (~mask)) | (((INA & (mask << 2)) >> 2)) & mask;
      unsigned char temp = ~((INA & (mask << 2))>>2);
      print("masked: %08b\n", temp);
      print("OUTA: %08b\n", OUTA);
      print("INA: %08b\n", INA);
      pause(1000);
    }
}
#include "simpletools.h"

// Encoder pins
#define RIGHT_A 11  
#define RIGHT_B 12  
#define LEFT_A  9
#define LEFT_B  10

// Global variables
//needs to be static volatile. Just volatile won't work. propgcc WTF is this buddy
static volatile int32_t rightWheelCount = 0;
static volatile int32_t leftWheelCount = 0;
static unsigned int encoderStack[128*2]; 
//need to see if cog_run() is better than this

//stolen from arduino https://github.com/PaulStoffregen/Encoder/blob/master/Encoder.h
//                           _______         _______       
//               Pin1 ______|       |_______|       |______ Pin1
// negative <---         _______         _______         __      --> positive
//               Pin2 __|       |_______|       |_______|   Pin2

		//	new	new	old	old
		//	pin2	pin1	pin2	pin1	Result
		//	----	----	----	----	------
		//	0	     0	    0	    0	  no movement
		//	0	     0	    0	    1	    +1
		//	0	     0	    1	    0	    -1
		//	0	     0	    1	    1	    +2  (assume pin1 edges only)
		//	0	     1	    0	    0	    -1
		//	0	     1	    0   	1	  no movement
		//	0	     1	    1	    0	    -2  (assume pin1 edges only)
		//	0	     1	    1	    1	    +1
		//	1	     0	    0	    0	    +1
		//	1	     0	    0	    1	    -2  (assume pin1 edges only)
		//	1	     0	    1	    0	  no movement
		//	1	     0     	1	    1	    -1
		//	1	     1	    0	    0	    +2  (assume pin1 edges only)
		//	1	     1	    0	    1	    -1
		//	1	     1	    1	    0	    +1
		//	1	     1	    1	    1	 no movement
static const int8_t decoderLookup[16] = {
  
    0,  +1, -1, +2, 
  
    -1, 0,  -2,+1, 
    
    +1, -2, 0,  -1, 
    
    +2, -1, +1, 0   
};

void encoderCog(void *par) {
    uint8_t lastRight = 0;
    uint8_t lastLeft = 0;
    
    while(1) {
      //getting the input values as xx
        uint8_t currRight = (input(RIGHT_B) << 1) | input(RIGHT_A);
        uint8_t currLeft = (input(LEFT_B) << 1) | input(LEFT_A);
        
      //left shifting them and adding the last input values
        uint8_t rightIndex = (currRight << 2) | lastRight;
        uint8_t leftIndex = (currLeft << 2) | lastLeft;
        
      //checking the lookup table
        rightWheelCount += decoderLookup[rightIndex];
        leftWheelCount += decoderLookup[leftIndex];
       //saving only the first two bits of the current value 
        lastRight = currRight & 0x03;  
        lastLeft = currLeft & 0x03;
        
        waitcnt(CNT + CLKFREQ/10000);  // 100μs delay
    }
}

int main() {
   
    clkset(_CLKMODE, _CLKFREQ);
    
    //set pins as input pins. this is not required but FANCY!!!!!!!!!
    set_direction(RIGHT_A, 0);
    set_direction(RIGHT_B, 0);
    set_direction(LEFT_A, 0);
    set_direction(LEFT_B, 0);
    
//YOU SOB, also cog_run does the same thing. better package on simple library. But does not free up memory for me if I don't use while loops.
// POOR
//need to think about overflow of values
//i have used int32_t but still
    cogstart(&encoderCog, NULL, encoderStack, sizeof(encoderStack));
    
    // Main loop
    while(1) {
        simpleterm_open();
        print("Right: %6d  Left: %6d\n", rightWheelCount, leftWheelCount);
        simpleterm_close();
        pause(100);
    }
}
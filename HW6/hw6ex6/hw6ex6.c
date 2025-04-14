#include "simpletools.h"

#define PING_PIN 8

// Global variables
volatile int distanceAvg = 0;
volatile int flag = 0;

// Function to measure distance using the PING))) sensor
int measurePingDistance(int pin) {
    int duration;
    int distance;

    low(pin);                // Ensure the pin is low
    pause(1);                // Wait for 1 ms
    high(pin);               // Send a high pulse
    pause(1);                // Wait for 1 ms
    low(pin);                // Set the pin low again

    duration = pulse_in(pin, 1); // Measure the duration of the echo pulse
    distance = duration / 58;    // Convert duration to distance in cm (speed of sound)

    return distance;
}

// Cog function to measure distance and calculate the 2-second average
void measureDistance() {
    int sum = 0, count = 0;

    while (1) {
        int distance = measurePingDistance(PING_PIN); // Measure distance on pin P5
        sum += distance;
        count++;

        if (count == 20) { // 2 seconds (20 * 100ms)
            distanceAvg = sum / 20; // Calculate average
            sum = 0;
            count = 0;
            flag = 1; // Set flag to indicate new data is available
        }

        pause(100); // Wait 100 ms before the next measurement
    }
}

// Cog function to print the updated average distance
void printDistance(void *par) {
    
    while (1) {
        if (flag) { // Check if new data is available
            simpleterm_open();
            print("Average Distance: %d cm\n", distanceAvg);
            simpleterm_close();
            flag = 0; // Reset flag
        }
    }
}

int main() {
    print("Starting");
  
    simpleterm_close();
    // Launch cogs for measuring and printing distance
    cog_run(measureDistance, 128);
    cog_run(printDistance, 128);
    
    

    while (1); // Keep the main program running
    return 0;
}

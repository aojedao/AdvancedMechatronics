/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools

#include <stdio.h>

float convertToCelsius(float fahrenheit) {
    return (fahrenheit - 32) * 5 / 9;
}

void displayTemperatures(float fahrenheit, float celsius) {
    printf("%.2f °F = %.2f °C\n", fahrenheit, celsius);
}

int main() {
    printf("Fahrenheit (°F)  Celsius (°C)\n");
    printf("-----------------------------\n");

    for (int f = 20; f <= 120; f++) {
        float c = convertToCelsius(f);
        displayTemperatures(f, c);
    }

    return 0;
}
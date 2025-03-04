/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools

#include <stdio.h>

float computeBMI(float weight, float height) {
    return (weight * 703) / (height * height);
}

int main() {
    
    pause(1000);
    
    float feet, inches, weight;
    float heightInches, bmi;

    // Prompt user for input
    printf("Enter height (feet): ");
    scanf("%f", &feet);
    printf("Enter height (inches): ");
    scanf("%f", &inches);

    printf("Enter weight (pounds): ");
    scanf("%f", &weight);

    // Convert height to inches
    heightInches = feet * 12 + inches;

    // Compute BMI
    bmi = computeBMI(weight, heightInches);

    // Print result
    printf("BMI: %.2f\n", bmi);

    return 0;
}
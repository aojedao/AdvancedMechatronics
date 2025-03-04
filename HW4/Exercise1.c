/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools

#include <stdio.h>

int main() {
    float angularVelocity, radius, duration;
    float linearVelocity, distance;

    // Prompt user for input
    pause(1000);
    printf("Enter angular velocity (rad/s): ");
    scanf("%f", &angularVelocity);

    printf("Enter wheel radius (m): ");
    scanf("%f", &radius);

    printf("Enter travel duration (s): ");
    scanf("%f", &duration);

    // Compute linear velocity and distance
    linearVelocity = angularVelocity * radius;
    distance = linearVelocity * duration;

    // Print results
    printf("Linear Velocity: %.2f m/s\n", linearVelocity);
    printf("Distance Traveled: %.2f m\n", distance);

    return 0;
}
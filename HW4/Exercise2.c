/*
  Blank Simple Project.c
  http://learn.parallax.com/propeller-c-tutorials 
*/
#include "simpletools.h"                      // Include simple tools

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#define SIZE 50

void generateRandomArray(int arr[], int size) {
    srand(time(0));
    for (int i = 0; i < size; i++) {
        arr[i] = rand() % 100;
    }
}

float computeAverage(int arr[], int size) {
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += arr[i];
    }
    return sum / size;
}

float computeStandardDeviation(int arr[], int size, float average) {
    float variance = 0;
    for (int i = 0; i < size; i++) {
        variance += pow(arr[i] - average, 2);
    }
    return sqrt(variance / size);
}

int main() {
    int arr[SIZE];
    generateRandomArray(arr, SIZE);

    float average = computeAverage(arr, SIZE);
    float stdDev = computeStandardDeviation(arr, SIZE, average);

    printf("Average: %.2f\n", average);
    printf("Standard Deviation: %.2f\n", stdDev);

    return 0;
}
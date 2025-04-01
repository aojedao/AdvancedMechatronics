#include "simpletools.h"  // Include SimpleTools library for UART and other utilities
#include "fdserial.h"     // Include Full Duplex Serial library for UART

// Define UART pins
#define RX_PIN 16  // Propeller RX pin
#define TX_PIN 15  // Propeller TX pin

// Function prototypes
void uartCommunication(void *par);  // Function to run UART communication in a cog
void handleCommand(char *command);
int getSensorData();

// Global variables
fdserial *uart;  // UART object

int main() {
  // Initialize UART in a separate cog
  cog_run(uartCommunication, 128);  // Run the UART communication function in a new cog

  while (1) {
    // Main loop can perform other tasks
    pause(1000);  // Placeholder for other tasks
  }
}

// Function to handle UART communication in a separate cog
void uartCommunication(void *par) {
  // Initialize UART
  uart = fdserial_open(RX_PIN, TX_PIN, 0, 115200);  // Open UART on RX=16, TX=15, Baud=115200

  char buffer[64];  // Buffer to store incoming commands
  int index = 0;    // Index for buffer

  while (1) {
    // Check if data is available from ESP32
    if (fdserial_rxReady(uart)) {
      char c = fdserial_rxChar(uart);  // Read a character from UART

      // If newline is received, process the command
      if (c == '\n') {
        buffer[index] = '\0';  // Null-terminate the string
        handleCommand(buffer);  // Process the command
        index = 0;  // Reset buffer index
      } else {
        buffer[index++] = c;  // Add character to buffer
        if (index >= 64) index = 0;  // Prevent buffer overflow
      }
    }

    // Send real-time data to ESP32
    int sensorData = getSensorData();  // Get sensor data
    dprint(uart, "DATA:%d\n", sensorData);  // Send data in "DATA:value" format
    pause(500);  // Wait 500 ms before sending the next data
  }
}

// Function to handle commands from ESP32
void handleCommand(char *command) {
  if (strcmp(command, "SET_PARAM:42") == 0) {
    // Example: Handle a specific command
    print("Received command: %s\n", command);  // Print to debug terminal
    // Perform some action based on the command
  } else {
    print("Unknown command: %s\n", command);  // Print unknown command
  }
}

// Function to simulate getting sensor data
int getSensorData() {
  static int value = 0;
  value += 1;  // Increment value for demonstration
  return value;  // Return the simulated sensor value
}
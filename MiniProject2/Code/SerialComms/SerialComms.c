#include "simpletools.h"  // Include the simple tools library
#include "simpletext.h"
int main() {
  // Define the RX and TX pins
  int rxPin = 14;  // Receive pin (P14)
  int txPin = 15;  // Transmit pin (P15)
  int baudRate = 115200;  // Baud rate

  serial *serial_connection;
  // Start the serial communication
  serial_connection = serial_open(rxPin, txPin,0, baudRate);

  char command[100];  // Buffer to store the incoming command
  int index = 0;      // Index for the command buffer

  print("Listening for commands...\n");  // Print initial message

  while (1) {

      int c = serial_rxChar(serial_connection);  // Read a character from the serial port
      print(c);
      //print("%s\n",c);
      if (c == '\n') {  // Check for end of command (newline character)
        command[index] = '\0';  // Null-terminate the string
        print("Received command: %s\n", command);  // Print the command
        index = 0;  // Reset index for the next command
      } else {
        if (index < sizeof(command) - 1) {  // Ensure buffer doesn't overflow
          command[index++] = c;  // Store the character in the buffer
        }
      }
    
  }

  return 0;  // End of program (though this will never be reached)
}
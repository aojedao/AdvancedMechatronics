#include "simpletools.h"  // Include basic functions like pause
#include "fdserial.h"     // Include the Full Duplex Serial library
#include "string.h"

//-----------------------------------------------------------
// THIS ONE IS ALREADY TESTED WITH ESP32 AND WORKS!!!!!!!!!!!!!
//-----------------------------------------------------------


// --- Define Serial Communication Parameters ---
#define RX_SERIAL_PIN 14 // Pin for receiving data (Connect to device's Tx)
#define TX_SERIAL_PIN 15  // Pin for transmitting data (Connect to device's Rx)
#define SERIAL_BAUD_RATE 115200 // Communication speed (must match connected device)

// Declare a pointer for the fdserial instance
fdserial *serial_connection;

int main() {
    // --- Initialize the Serial Connection ---
    serial_connection = fdserial_open(RX_SERIAL_PIN, TX_SERIAL_PIN, 0, SERIAL_BAUD_RATE);

    // --- Check for Initialization Errors ---
    if (serial_connection == NULL) {
        print("Error: Failed to open fdserial on P%d(Rx)/P%d(Tx).\n", RX_SERIAL_PIN, TX_SERIAL_PIN);
        while (1) { // Halt with LED blink on error
            high(26); pause(100); low(26); pause(100);
        }
    }

    // --- Ready Message (Optional) ---
    // You might want a simple message so you know the Propeller has started
    // If you don't want *any* output initiated by the Propeller, comment this out.
    dprint(serial_connection, "Serial Echo Ready (P%d/P%d @ %d bps)\n",
           RX_SERIAL_PIN, TX_SERIAL_PIN, SERIAL_BAUD_RATE);
    print("Ready to listen");


    // --- Main Loop: Check for and Echo Incoming Data ---
    while (1) {
        // Check if any character has been received (-1 if none)
        char received_data = fdserial_rxChar(serial_connection);
        

        if (received_data!=1 ) {
            // --- Character Received ---
            // Cast the received integer to a character
            int data_char = received_data;
            print("%c\n",data_char);

            // --- Directly Echo the Character ---
            // Send the exact character received back out the TX pin
            //fdserial_tx(serial_connection, data_char);

        } // End if(received_data != -1)

        // --- Small Delay ---
        // Important to allow other cogs (like fdserial's driver) to run
        pause(5); // Can potentially make this shorter for faster echo if needed

    } // End while(1)

    // Unreachable code
    // fdserial_close(serial_connection);
    // return 0;
}
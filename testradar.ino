#include <Arduino.h>

// --- HLK-LD1115H Radar Sensor Configuration ---
#define RADAR_SERIAL Serial2       // Use Serial2 for the radar (e.g., pins 16, 17) - CHECK YOUR BOARD!
#define RADAR_RX_PIN 16            // ESP32 pin connected to Radar TX
#define RADAR_TX_PIN 17            // ESP32 pin connected to Radar RX
#define RADAR_BAUD_RATE 115200     // Baud rate for the radar - !!! VERIFY FROM DATASHEET !!!

// --- Buffer for incoming radar UART data ---
char radarSerialBuffer[128];       // Buffer to hold incoming characters
int radarBufferIndex = 0;          // Current position in the buffer

// --- Variables to store parsed data ---
volatile float radar_distance_m = -1.0;  // Parsed distance in meters (-1.0 = invalid)
volatile float radar_velocity_mps = 0.0; // Parsed relative velocity in m/s

// =========================================================================
// SETUP FUNCTION
// =========================================================================
void setup() {
    // Initialize the main Serial port for debugging output (to your computer)
    Serial.begin(115200);
    while (!Serial); // Wait for Serial monitor connection (optional)
    Serial.println("\nESP32 HLK-LD1115H Radar Test Sketch");

    // Initialize the Hardware Serial port connected to the radar module
    // Uses specified pins (RX, TX) and configuration (8 data bits, No parity, 1 stop bit)
    RADAR_SERIAL.begin(RADAR_BAUD_RATE, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    Serial.printf("Attempting to listen to Radar on Serial%d (RX:%d, TX:%d) at %d baud.\n",
                  RADAR_SERIAL == Serial1 ? 1 : 2, RADAR_RX_PIN, RADAR_TX_PIN, RADAR_BAUD_RATE);

    // Simple check if the serial port initialization succeeded
    if (!RADAR_SERIAL) {
        Serial.println("Error: Could not begin Radar Serial port! Halting.");
        while(1); // Stop execution
    }

    Serial.println("Waiting for radar data...");
}

// =========================================================================
// FUNCTION TO READ AND PARSE RADAR DATA
// =========================================================================
void readAndParseRadar() {
    // Process all available characters from the radar serial port
    while (RADAR_SERIAL.available()) {
        char receivedChar = RADAR_SERIAL.read();

        // --- !!! IMPORTANT: PARSING LOGIC NEEDS TO MATCH YOUR DATASHEET !!! ---
        // This section contains PLACEHOLDER logic assuming a simple text protocol
        // where messages end with a newline character '\n'.
        // The format "D:dist_m,V:vel_mps\n" is HYPOTHETICAL.
        // You MUST replace this with logic based on the official HLK-LD1115H documentation.

        // Check for end-of-message character (e.g., newline)
        if (receivedChar == '\n') {
            radarSerialBuffer[radarBufferIndex] = '\0'; // Null-terminate the received string

            // --- Placeholder Parsing Attempt ---
            float parsed_dist = -1.0; // Temporary variables for parsing results
            float parsed_vel = 0.0;

            // Use sscanf to parse the buffer based on the *hypothetical* format "D:float,V:float"
            // Adjust the format string ("D:%f,V:%f") based on the ACTUAL protocol!
            int itemsParsed = sscanf(radarSerialBuffer, "D:%f,V:%f", &parsed_dist, &parsed_vel);

            if (itemsParsed == 2) { // Check if both expected values were found
                // Update global variables with parsed data
                // Consider adding locks (mutex/semaphore) or disabling interrupts
                // if these variables are accessed by ISRs/other tasks in a larger program.
                // For this simple test, direct volatile write is usually okay.
                radar_distance_m = parsed_dist;
                radar_velocity_mps = parsed_vel;

                // Print the successfully parsed data to the main Serial Monitor
                Serial.printf("RECEIVED => Dist: %.3f m, Vel: %.3f m/s\n", radar_distance_m, radar_velocity_mps);

            } else {
                // Parsing failed, print the raw buffer to help debugging the format
                Serial.print("Parse Error. Raw data: [");
                Serial.print(radarSerialBuffer);
                Serial.println("]");
                // Optionally invalidate global readings on error
                // radar_distance_m = -1.0;
                // radar_velocity_mps = 0.0;
            }
            // --- End Placeholder Parsing Attempt ---

            // Reset buffer for the next message
            radarBufferIndex = 0;
            memset(radarSerialBuffer, 0, sizeof(radarSerialBuffer)); // Clear the buffer contents

        } else if (receivedChar >= 32 && receivedChar <= 126) { // Store only printable ASCII chars
            // Add character to buffer if there's space (prevent overflow)
            if (radarBufferIndex < sizeof(radarSerialBuffer) - 1) {
                radarSerialBuffer[radarBufferIndex++] = receivedChar;
            } else {
                // Buffer overflow! Discard buffer and print warning.
                Serial.println("Radar buffer overflow!");
                radarBufferIndex = 0;
                memset(radarSerialBuffer, 0, sizeof(radarSerialBuffer)); // Clear the buffer
            }
        }
        // Ignore non-printable characters (like carriage return '\r' if present before '\n')
        // If the protocol is binary, this whole character-by-character approach needs rethinking.

        // --- !!! END OF DATASHEET-SPECIFIC PARSING LOGIC !!! ---
    }
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
    // Continuously check for and process incoming radar data
    readAndParseRadar();

    // Add a small delay. This prevents the loop from spinning extremely fast
    // when no serial data is available, yielding time to other potential tasks
    // (like background WiFi/BT) and reducing unnecessary CPU usage.
    delay(10); // Delay in milliseconds
}

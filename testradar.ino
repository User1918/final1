#include <Arduino.h>

// --- HLK-LD1115H Radar Sensor Configuration ---
#define RADAR_SERIAL Serial2       // Use Serial2 for the radar (check your board!)
#define RADAR_RX_PIN 16            // ESP32 pin connected to Radar TX
#define RADAR_TX_PIN 17            // ESP32 pin connected to Radar RX (optional but good practice)
#define RADAR_BAUD_RATE 115200     // Baud rate for the radar - !!! CHECK DATASHEET !!!

// --- Buffer for incoming radar UART data ---
char radarSerialBuffer[128]; // Increased buffer size for potentially longer messages
int radarBufferIndex = 0;

// --- Variables to store parsed data ---
float radar_distance_m = -1.0;  // Default to -1.0 to indicate no valid reading yet
float radar_velocity_mps = 0.0;

// =========================================================================
// SETUP FUNCTION
// =========================================================================
void setup() {
    // Initialize the main Serial port for debugging output (to your computer)
    Serial.begin(115200);
    while (!Serial); // Wait for Serial monitor to connect (optional)
    Serial.println("\nESP32 HLK-LD1115H Radar Test Sketch");

    // Initialize the Hardware Serial port connected to the radar module
    RADAR_SERIAL.begin(RADAR_BAUD_RATE, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    Serial.printf("Attempting to listen to Radar on Serial%d (RX:%d, TX:%d) at %d baud.\n",
                  RADAR_SERIAL == Serial1 ? 1 : 2, RADAR_RX_PIN, RADAR_TX_PIN, RADAR_BAUD_RATE);

    if (!RADAR_SERIAL) { // Check if Serial port was configured (simple check)
        Serial.println("Error: Could not begin Radar Serial port!");
        while(1); // Halt execution
    }
     Serial.println("Waiting for radar data...");
}

// =========================================================================
// FUNCTION TO READ AND PARSE RADAR DATA
// =========================================================================
void readAndParseRadar() {
    while (RADAR_SERIAL.available()) {
        char receivedChar = RADAR_SERIAL.read();

        // --- !!! IMPORTANT: PARSING LOGIC NEEDS TO MATCH YOUR DATASHEET !!! ---
        // This section assumes a simple text-based protocol ending with newline '\n'.
        // Common formats might be CSV, JSON, or a custom binary protocol.
        // You MUST replace the parsing logic below based on the HLK-LD1115H manual.

        // Example for a hypothetical format like "D:1.23,V:-0.45\n"
        if (receivedChar == '\n') { // End of message detected
            radarSerialBuffer[radarBufferIndex] = '\0'; // Null-terminate the string

            // Attempt to parse the received line
            float parsed_dist = -1.0;
            float parsed_vel = 0.0;

            // Use sscanf for the hypothetical format "D:dist,V:vel"
            // Adjust the format string "%f" and variable pointers based on the ACTUAL protocol!
            int itemsParsed = sscanf(radarSerialBuffer, "D:%f,V:%f", &parsed_dist, &parsed_vel);

            if (itemsParsed == 2) { // Check if both distance and velocity were successfully parsed
                radar_distance_m = parsed_dist;
                radar_velocity_mps = parsed_vel;
                Serial.printf("RECEIVED => Dist: %.3f m, Vel: %.3f m/s\n", radar_distance_m, radar_velocity_mps);
            } else {
                // Parsing failed - print the raw buffer for debugging
                Serial.print("Parse Error. Raw data: [");
                Serial.print(radarSerialBuffer);
                Serial.println("]");
                // Optionally invalidate readings
                // radar_distance_m = -1.0;
                // radar_velocity_mps = 0.0;
            }

            radarBufferIndex = 0; // Reset buffer index for the next message
            memset(radarSerialBuffer, 0, sizeof(radarSerialBuffer)); // Clear the buffer

        } else if (receivedChar >= 32 && receivedChar <= 126) { // Store printable characters
            // Add character to buffer if space available
            if (radarBufferIndex < sizeof(radarSerialBuffer) - 1) {
                radarSerialBuffer[radarBufferIndex++] = receivedChar;
            } else {
                // Buffer overflow - discard and reset
                Serial.println("Radar buffer overflow!");
                radarBufferIndex = 0;
                memset(radarSerialBuffer, 0, sizeof(radarSerialBuffer)); // Clear the buffer
            }
        }
        // Ignore non-printable characters except newline (or handle binary data differently)

        // --- !!! END OF DATASHEET-SPECIFIC PARSING LOGIC !!! ---
    }
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
    // Continuously check for and process incoming radar data
    readAndParseRadar();

    // Add a small delay to prevent the loop from running too fast
    // and consuming 100% CPU if no serial data is available.
    delay(10);
}
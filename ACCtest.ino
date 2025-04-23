#include <Arduino.h>
#include <Wire.h>

// --- TOF10120 Constants ---
const uint8_t TOF_I2C_ADDRESS = 0xA4; // Default address from datasheet
const uint16_t MIN_VALID_DISTANCE_TOF = 100; // Min valid distance (mm)
const uint16_t MAX_VALID_DISTANCE_TOF = 1800; // Max valid distance (mm)

// --- Function to configure sensor for I2C reading ---
bool configureTofForI2C_minimal() {
    Wire.beginTransmission(TOF_I2C_ADDRESS);
    Wire.write(0x09); // Register: Distance Sending Mode
    Wire.write(0x01); // Value: Host reads passively (I2C enabled)
    byte error = Wire.endTransmission(); // Send configuration command

    if (error == 0) {
        // Success code 0 means the sensor acknowledged the command
        Serial.println("Minimal Test: Configuration command acknowledged (OK).");
        delay(10); // Short delay after config
        return true;
    } else {
        // See https://www.arduino.cc/reference/en/language/functions/communication/wire/endtransmission/
        Serial.print("Minimal Test: Configuration command FAILED. Wire.endTransmission() error code: ");
        Serial.println(error);
        Serial.println("  Error Codes: 1=data too long, 2=recv addr NACK, 3=recv data NACK, 4=other error");
        return false;
    }
}

// --- Function to read distance ---
uint16_t readDistance_minimal() {
    byte error; // To store error codes

    // Step 1: Tell sensor which register we want to read (0x00 for real-time distance)
    Wire.beginTransmission(TOF_I2C_ADDRESS);
    Wire.write(0x00); // Register address for real-time distance LSB
    error = Wire.endTransmission(); // Send the register address
    if (error != 0) {
        Serial.print("Minimal Test: Failed to send read address. Error: ");
        Serial.println(error);
        return UINT16_MAX; // Return invalid marker
    }

    // Step 2: Wait a moment for sensor (Datasheet says >30us)
    delayMicroseconds(50); // 50us delay

    // Step 3: Request 2 bytes of data from the sensor
    uint8_t bytesReceived = Wire.requestFrom(TOF_I2C_ADDRESS, (uint8_t)2);

    // Step 4: Check if we received the expected number of bytes
    if (bytesReceived < 2) {
        Serial.print("Minimal Test: Failed to receive 2 bytes. Received: ");
        Serial.println(bytesReceived);
        // Clear the buffer if anything partial was received
        while(Wire.available()) { Wire.read(); }
        return UINT16_MAX; // Return invalid marker
    }

    // Step 5: Read the two bytes
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();

    // Step 6: Combine bytes into a 16-bit value
    uint16_t rawValue = (highByte << 8) | lowByte;

    // Step 7: Check if the value is within the sensor's valid range
    if (rawValue < MIN_VALID_DISTANCE_TOF || rawValue > MAX_VALID_DISTANCE_TOF) {
        Serial.print("Minimal Test: Raw value out of range (100-1800mm): ");
        Serial.println(rawValue);
        return UINT16_MAX; // Return invalid marker
    }

    // If all steps passed, return the valid distance
    return rawValue;
}

// --- Setup ---
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000) ; // Wait max 2s for serial monitor
    Serial.println("\n\nMinimal TOF10120 Test Sketch");

    // Initialize I2C using default SDA (21) and SCL (22) pins
    Wire.begin();
    Wire.setClock(100000); // Start with a slower I2C clock (100kHz) for stability test
    Serial.println("I2C Initialized (SDA=21, SCL=22, Clock=100kHz).");

    // Attempt to configure the sensor for I2C mode
    Serial.println("Attempting to configure TOF sensor...");
    if (!configureTofForI2C_minimal()) {
        Serial.println("******************************************************");
        Serial.println("Configuration FAILED. Check wiring, address (0xA4), power.");
        Serial.println("Halting execution.");
        Serial.println("******************************************************");
        while (1) { delay(100); } // Stop here if config fails
    } else {
        Serial.println("Configuration successful. Starting periodic reads...");
    }
}

// --- Loop ---
void loop() {
    Serial.print("Reading Distance... ");
    uint16_t distance = readDistance_minimal(); // Attempt to read

    if (distance == UINT16_MAX) {
        // This means readDistance_minimal() encountered an error or invalid value
        Serial.println("Result: INVALID / ERROR");
    } else {
        // Successfully read a valid distance
        Serial.print("Result: ");
        Serial.print(distance);
        Serial.println(" mm");
    }

    delay(500); // Wait half a second before the next reading attempt
}

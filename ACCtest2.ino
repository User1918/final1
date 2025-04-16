#include <Arduino.h>
// #include <ESP32Servo.h> // Consider using this library for non-blocking servo control on ESP32
// #include <Servo.h>      // Or this standard library for non-blocking servo control

// --- Motor and Servo Configuration ---
#define RPWM 2
#define LPWM 15
#define R_EN 0
#define L_EN 4
#define IS_R 25 // Analog input for Right Current Sense
#define IS_L 26 // Analog input for Left Current Sense

#define SERVO_PIN 13
#define MIN_PULSE_WIDTH 500    // Minimum pulse width (microseconds) for servo
#define MAX_PULSE_WIDTH 2500   // Maximum pulse width (microseconds) for servo
#define REFRESH_INTERVAL 20000 // Servo refresh interval (20000 us = 20ms) - Check your servo spec!

// Current limit threshold (needs calibration based on sensor & units)
// Assuming ACS712-style sensor where output voltage is proportional to current
// Example: If sensor is 100mV/A and Vref=3.3V, ADC_resolution=4095:
// Reading = (Current_mA / 1000) * 100mV * (4095 / 3300mV)
// So, Current_mA = Reading * (3300 / 4095) * (1000 / 100) = Reading * 8.058
// Define CURRENT_LIMIT in terms of the raw ADC reading for simplicity, or calculate properly.
#define RAW_CURRENT_LIMIT 87 // Example: Corresponds to ~700mA if sensitivity is ~8.06 mA/ADC unit
                               // YOU MUST ADJUST THIS based on your sensor and desired mA limit!

// --- RPM Encoder Configuration ---
#define HALL_SENSOR_PIN 3            // <<< CHOOSE an available interrupt pin (e.g., 3, 18, 19, etc.) NOT used above
const int PULSES_PER_REVOLUTION = 5; // Number of pulses from sensor per full revolution
const unsigned long ZERO_RPM_TIMEOUT_MS = 1000; // Consider RPM zero if no pulse received for this duration (ms)

// --- Global Variables ---
int32_t steering_angle = 90; // Target steering angle (center)
int32_t speed_motor = 0;     // Target motor speed (0-255 for analogWrite)

// RPM calculation variables ('volatile' for ISR access)
volatile unsigned long lastPulseTime_us = 0;
volatile unsigned long pulseInterval_us = 0;
volatile bool newPulseDetected = false;
volatile bool isFirstPulse = true;
float currentRPM = 0.0; // Calculated RPM

// --- RPM Interrupt Service Routine (ISR) ---
void IRAM_ATTR hallSensorISR() { // IRAM_ATTR recommended for ESP32
  unsigned long now_us = micros();
  if (!isFirstPulse) {
    pulseInterval_us = now_us - lastPulseTime_us;
    newPulseDetected = true;
  } else {
    isFirstPulse = false;
  }
  lastPulseTime_us = now_us;
}

// --- Setup Function ---
void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection
  Serial.println("System Initializing...");

  // Configure servo pin
  pinMode(SERVO_PIN, OUTPUT);
  // Consider initializing servo library here if using one

  // Configure motor pins
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(IS_R, INPUT); // Configure current sense pins as input
  pinMode(IS_L, INPUT);

  // Configure RPM sensor pin
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP); // Use pullup if sensor needs it

  // Attach RPM interrupt
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING); // Adjust FALLING/RISING as needed
  Serial.print("RPM detection attached to pin: "); Serial.println(HALL_SENSOR_PIN);

  // Initialize RPM timing
  lastPulseTime_us = micros();

  Serial.println("Setup Complete. Waiting for commands...");
}

// --- Motor Control Function ---
void control_motor(int speed) {
  // Read raw ADC values from current sensors
  int currentR_raw = analogRead(IS_R);
  int currentL_raw = analogRead(IS_L);

  // *** YOU NEED TO CALIBRATE THIS SECTION ***
  // Convert raw ADC reading to actual current (mA or A) based on your sensor's specifications
  // Example (replace with your actual calculation):
  // float currentR_mA = (float)currentR_raw * (3300.0 / 4095.0) * SENSOR_SCALING_FACTOR;
  // float currentL_mA = (float)currentL_raw * (3300.0 / 4095.0) * SENSOR_SCALING_FACTOR;
  // For now, we compare raw values directly to RAW_CURRENT_LIMIT
  // Serial.print("Raw Current R: "); Serial.print(currentR_raw);
  // Serial.print(" | Raw Current L: "); Serial.println(currentL_raw);


  // Check if current is within limits (using raw comparison for now)
  if (currentR_raw < RAW_CURRENT_LIMIT && currentL_raw < RAW_CURRENT_LIMIT) {
    digitalWrite(R_EN, HIGH); // Enable driver
    digitalWrite(L_EN, HIGH);

    // Ensure speed is within valid PWM range
    speed = constrain(speed, 0, 255);

    // Set motor speed (assuming forward direction only with LPWM)
    analogWrite(LPWM, speed);
    analogWrite(RPWM, 0); // Keep other direction PWM off
  } else {
    // Current limit exceeded, disable motor driver
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    analogWrite(LPWM, 0); // Ensure PWM is off
    analogWrite(RPWM, 0);
    // Serial.println("!!! CURRENT LIMIT EXCEEDED !!!");
  }
}

// --- Servo Control Function ---
// !!! WARNING: This manual servo control uses delayMicroseconds() which BLOCKS
// !!!          the loop. This will negatively impact serial reading and RPM
// !!!          detection responsiveness. Consider using a Servo library.
void control_servo() {
  // Constrain angle to safe limits (adjust as needed for your setup)
  int angle = constrain(steering_angle, 40, 130);

  // Map the angle (0-180 range expected by map) to the pulse width range
  int pulseWidth = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

  // Generate the servo pulse
  digitalWrite(SERVO_PIN, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(SERVO_PIN, LOW);

  // Wait for the refresh interval - THIS IS BLOCKING
  // Calculate remaining delay carefully
  if (REFRESH_INTERVAL > pulseWidth) {
      delayMicroseconds(REFRESH_INTERVAL - pulseWidth);
  } else {
      // Avoid negative delay if pulseWidth is somehow larger than interval
      delayMicroseconds(1); // Minimum delay
  }
}

// --- RPM Update Function ---
void updateRPM() {
  unsigned long interval_us_local = 0;
  unsigned long lastPulseTime_us_local = 0;
  bool newPulseFlag_local = false;

  // Safely check and read volatile variables
  noInterrupts();
  newPulseFlag_local = newPulseDetected;
  if (newPulseFlag_local) {
    interval_us_local = pulseInterval_us;
    newPulseDetected = false; // Reset flag
  }
  lastPulseTime_us_local = lastPulseTime_us;
  interrupts();

  // Calculate RPM if a new pulse was detected
  if (newPulseFlag_local) {
    if (interval_us_local > 0) {
      // Formula: RPM = 60,000,000 / (interval_us * pulses_per_revolution)
      currentRPM = 60000000.0 / ((float)interval_us_local * PULSES_PER_REVOLUTION);
    } else {
      currentRPM = 0.0; // Avoid division by zero
    }
  }

  // Check for Zero RPM Timeout
  unsigned long zero_rpm_timeout_us = ZERO_RPM_TIMEOUT_MS * 1000UL;
  if (micros() - lastPulseTime_us_local > zero_rpm_timeout_us) {
     if (!newPulseFlag_local) { // Check flag again to ensure no pulse arrived right before timeout check
        currentRPM = 0.0;
        // Optionally reset isFirstPulse after a long stop
        // noInterrupts(); isFirstPulse = true; interrupts();
    }
  }
}


// --- Main Loop ---
void loop() {
  // --- Serial Data Handling ---
  // Using the 10-byte format parsing found in the original loop
  static uint8_t serialBuffer[10];
  static int bufferIndex = 0;

  while (Serial.available()) {
    uint8_t byteRead = Serial.read();

    // Wait for start character '<'
    if (bufferIndex == 0 && byteRead != '<') {
      continue;
    }

    // Fill buffer
    if (bufferIndex < 10) {
        serialBuffer[bufferIndex++] = byteRead;
    } else {
        // Buffer overflow, reset
        bufferIndex = 0;
        continue;
    }


    // Check for complete packet (10 bytes received)
    if (bufferIndex == 10) {
      if (serialBuffer[9] == '>') { // Check end character
        // Decode 4-byte integer for angle and 4-byte integer for speed
        // Assuming little-endian transmission
        int32_t angle_received = *((int32_t*)&serialBuffer[1]);
        int32_t speed_received = *((int32_t*)&serialBuffer[5]);

        // Update global target values
        steering_angle = angle_received;
        speed_motor = speed_received; // Assuming speed is 0-255 range for PWM

        // Print received values for debugging
        Serial.print("Received Angle: "); Serial.print(steering_angle);
        Serial.print(" | Received Speed: "); Serial.println(speed_motor);

      } else {
        Serial.println("Packet Error: Incorrect end character!");
      }
      // Reset buffer index for next packet
      bufferIndex = 0;
    }
  } // End while Serial.available

  // --- Update Sensor Readings and Control Systems ---

  // Update RPM Calculation
  updateRPM();

  // Control Motor (includes current check)
  control_motor(speed_motor);

  // Control Servo (!!! WARNING: BLOCKING !!!)
  control_servo();

  // --- Optional: Print RPM periodically ---
  static unsigned long lastRpmPrintTime = 0;
  if (millis() - lastRpmPrintTime > 500) { // Print every 500ms
    lastRpmPrintTime = millis();
    Serial.print("Current RPM: ");
    Serial.println(currentRPM, 2); // Print with 2 decimal places
  }

} // End loop
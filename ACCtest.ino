#include <Arduino.h>

// --- Configuration ---
const int HALL_SENSOR_PIN = 2;       // Pin connected to the Hall effect sensor (must be an interrupt pin)
const int PULSES_PER_REVOLUTION = 5; // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< CHANGED FROM 4 to 5
const unsigned long ZERO_RPM_TIMEOUT_MS = 1000; // Consider RPM zero if no pulse received for this duration (in milliseconds)

// --- Global Variables ---
// 'volatile' is crucial for variables shared between ISR and main loop
volatile unsigned long lastPulseTime_us = 0;      // Timestamp of the last detected pulse (microseconds)
volatile unsigned long pulseInterval_us = 0;      // Time interval between the last two pulses (microseconds)
volatile bool newPulseDetected = false;         // Flag set by ISR when a new pulse arrives
volatile bool isFirstPulse = true;            // To handle the very first pulse correctly

// Calculated RPM
float currentRPM = 0.0;

// For non-blocking printing
unsigned long lastPrintTime_ms = 0;
const unsigned long PRINT_INTERVAL_MS = 200; // Print RPM every 200ms

// --- Interrupt Service Routine (ISR) ---
// This function is called automatically when the sensor pin state changes (FALLING edge)
// Keep ISRs as short and fast as possible!
void IRAM_ATTR hallSensorISR() { // IRAM_ATTR recommended for ESP32 for speed, harmless on AVR
  unsigned long now_us = micros(); // Get current time immediately

  // Calculate interval only after the first pulse has been detected
  if (!isFirstPulse) {
    pulseInterval_us = now_us - lastPulseTime_us;
    newPulseDetected = true; // Signal main loop to calculate RPM
  } else {
    isFirstPulse = false; // Next pulse will calculate an interval
  }

  lastPulseTime_us = now_us; // Record time of this pulse for the next interval calculation
}

// --- Setup Function ---
void setup() {
  Serial.begin(115200); // Start serial communication for output
  while (!Serial); // Wait for serial port to connect (needed for some boards like Leonardo)
  Serial.println("RPM Calculator Initialized");
  Serial.print("Hall Sensor Pin: "); Serial.println(HALL_SENSOR_PIN);
  Serial.print("Pulses per Revolution: "); Serial.println(PULSES_PER_REVOLUTION); // Will now print 5
  Serial.print("Zero RPM Timeout (ms): "); Serial.println(ZERO_RPM_TIMEOUT_MS);

  // Configure the sensor pin as input with an internal pull-up resistor
  // (Pull-up might be needed depending on your sensor type - common for open-collector outputs)
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);

  // Attach the interrupt
  // digitalPinToInterrupt(pin) converts pin number to interrupt number
  // hallSensorISR is the function to call
  // FALLING means trigger when the pin goes from HIGH to LOW (adjust if needed)
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);

  lastPulseTime_us = micros(); // Initialize last pulse time
}

// --- Main Loop ---
void loop() {
  // --- Local variables for safe reading of volatile data ---
  unsigned long interval_us_local = 0;
  unsigned long lastPulseTime_us_local = 0;
  bool newPulseFlag_local = false;

  // --- Safely check and read volatile variables ---
  noInterrupts(); // Temporarily disable interrupts to safely copy volatile variables
  newPulseFlag_local = newPulseDetected;
  if (newPulseFlag_local) {
    interval_us_local = pulseInterval_us;
    newPulseDetected = false; // Reset the flag after reading
  }
  lastPulseTime_us_local = lastPulseTime_us; // Read last pulse time for timeout check
  interrupts(); // Re-enable interrupts

  // --- Calculate RPM if a new pulse was detected ---
  if (newPulseFlag_local) {
    if (interval_us_local > 0) {
      // Calculate RPM:
      // Formula: RPM = 60,000,000.0 / (interval_microseconds * pulses_per_revolution)
      currentRPM = 60000000.0 / ( (float)interval_us_local * PULSES_PER_REVOLUTION ); // Calculation automatically uses 5 now
    } else {
      // Avoid division by zero
      currentRPM = 0.0;
    }
  }

  // --- Check for Zero RPM Timeout ---
  // Use micros() for timeout check for better consistency with ISR timing
  unsigned long zero_rpm_timeout_us = ZERO_RPM_TIMEOUT_MS * 1000UL; // Convert timeout to microseconds
  if (micros() - lastPulseTime_us_local > zero_rpm_timeout_us) {
    if (!newPulseFlag_local) { // Only set to zero if no *new* pulses were detected recently AND timeout exceeded
        currentRPM = 0.0;
        // Optionally reset isFirstPulse if you want the calculation
        // to restart cleanly after a long stop
        // noInterrupts();
        // isFirstPulse = true;
        // interrupts();
    }
  }

  // --- Print RPM periodically (non-blocking) ---
  unsigned long currentTime_ms = millis();
  if (currentTime_ms - lastPrintTime_ms >= PRINT_INTERVAL_MS) {
    lastPrintTime_ms = currentTime_ms;
    Serial.print("RPM: ");
    Serial.println(currentRPM, 2); // Print RPM with 2 decimal places
  }

  // --- Other tasks can be done here ---
  // delay(1);
}
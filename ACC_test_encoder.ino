#include <Arduino.h>
#include <math.h>
#include "esp_timer.h" // For ESP32 specific timer

// --- Pin Definitions --- (Keep these as they are)
#define RPWM 2
#define LPWM 15
#define R_EN 0
#define L_EN 4
#define IS_R 25
#define IS_L 26
#define SERVO_PIN 13         // Keep servo pin if needed
#define HALL_SENSOR_PIN 12

// --- Servo Constants --- (Keep these if servo is still used)
#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500
#define REFRESH_INTERVAL 20000
#define SERVO_MIN_ANGLE 40
#define SERVO_MAX_ANGLE 130

// --- Motor & Driver Constants --- (Keep these, calibrate CURRENT_LIMIT_MV)
#define CURRENT_LIMIT_MV 7000 // Example: Needs calibration!

// --- Encoder / RPM Calculation --- (Keep these)
const int PULSES_PER_REV = 5;
volatile unsigned long pulseCount = 0;
volatile float currentRPM = 0.0;        // Measured RPM
const unsigned long RPM_CALC_INTERVAL_MS = 50;
const unsigned long RPM_CALC_INTERVAL_US = RPM_CALC_INTERVAL_MS * 1000;
volatile unsigned long lastRpmCalcTime = 0;
volatile unsigned long lastRpmCalcPulseCount = 0;
esp_timer_handle_t rpm_timer_handle;

// --- Stop Detection Timer --- (Keep this, optional but useful)
const int STOP_CHECK_INTERVAL_US = 1000000;
volatile unsigned long lastCheckedPulseCount = 0;
bool motorRunning = false; // Flag set by setMotorPWM
esp_timer_handle_t stop_timer_handle;

// --- Serial Communication Buffer ---
String serialDataIn = ""; // Buffer for incoming serial data

// --- Global State Variables ---
int commandedPWM = 0;       // The PWM value received via Serial (0-255)
int32_t steering_angle = 90; // Keep steering angle if needed

// =========================================================================
// INTERRUPT SERVICE ROUTINE (ISR) - Hall Sensor / Encoder (Keep as is)
// =========================================================================
void IRAM_ATTR hallSensorISR() {
    pulseCount++;
}

// =========================================================================
// TIMER CALLBACK - RPM Calculation (Keep as is)
// =========================================================================
void IRAM_ATTR rpm_timer_callback(void *arg) {
    // ... (Keep the existing code for rpm_timer_callback) ...
    unsigned long currentTime_us = micros();
    unsigned long currentPulseReading;
    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();
    unsigned long deltaTime_us = currentTime_us - lastRpmCalcTime;
    unsigned long deltaPulses = currentPulseReading - lastRpmCalcPulseCount;
    float calculatedRPM = 0.0;
    if (deltaTime_us > 0 && deltaPulses > 0) {
        float pulses_per_second = (float)deltaPulses * 1000000.0 / (float)deltaTime_us;
        float rps = pulses_per_second / (float)PULSES_PER_REV;
        calculatedRPM = rps * 60.0;
    } else if (deltaTime_us > RPM_CALC_INTERVAL_US * 2) {
         if (commandedPWM < 5) { // If commanded PWM is very low, allow forcing RPM to 0
              calculatedRPM = 0.0;
         }
    }
    noInterrupts();
    currentRPM = calculatedRPM;
    interrupts();
    lastRpmCalcTime = currentTime_us;
    lastRpmCalcPulseCount = currentPulseReading;
}

// =========================================================================
// TIMER CALLBACK - Stop Detection (Keep as is)
// =========================================================================
void IRAM_ATTR stop_timer_callback(void *arg) {
   // ... (Keep the existing code for stop_timer_callback) ...
    unsigned long currentPulseReading;
    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();
    // Force RPM to 0 if motor was running but pulses stopped
    if (motorRunning && currentPulseReading == lastCheckedPulseCount) {
        noInterrupts();
        currentRPM = 0.0;
        interrupts();
    }
    lastCheckedPulseCount = currentPulseReading;
}

// =========================================================================
// FUNCTION: setMotorPWM (Keep as is)
// Applies the given PWM value (0-255) to the motor driver.
// =========================================================================
void setMotorPWM(int pwmValue) {
   // ... (Keep the existing code for setMotorPWM, including current check) ...
   // This function now directly controls the motor based on commandedPWM
    pwmValue = constrain(pwmValue, 0, 255);
    // currentPWM = pwmValue; // Don't need this global anymore unless displayData uses it

    uint16_t raw_IS_R = analogRead(IS_R);
    float voltage_IS_R_mV = raw_IS_R * (3300.0 / 4095.0);

    if (voltage_IS_R_mV < CURRENT_LIMIT_MV ) {
        digitalWrite(R_EN, HIGH);
        digitalWrite(L_EN, HIGH);
        analogWrite(LPWM, pwmValue);
        digitalWrite(RPWM, LOW);
        motorRunning = (pwmValue > 5); // Consider motor running if PWM > threshold
    } else {
        Serial.println("!!! CURRENT LIMIT EXCEEDED !!!");
        digitalWrite(R_EN, LOW);
        digitalWrite(L_EN, LOW);
        analogWrite(LPWM, 0);
        digitalWrite(RPWM, LOW);
        motorRunning = false;
        // No integral to reset anymore
    }
}

// =========================================================================
// FUNCTION: control_servo (Keep as is, if needed)
// =========================================================================
void control_servo() {
   // ... (Keep the existing code for control_servo, if using steering) ...
    static unsigned long lastServoUpdate = 0;
    unsigned long now = micros();

    if (now - lastServoUpdate >= REFRESH_INTERVAL) {
        lastServoUpdate = now;
        int constrainedAngle = constrain(steering_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        int pulseWidth = map(constrainedAngle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(pulseWidth);
        digitalWrite(SERVO_PIN, LOW);
    }
}

// =========================================================================
// FUNCTION: parseSimpleCommand (NEW - handles simple PWM command)
// Parses commands like "<pwm_value>" or "<pwm_value,angle>"
// Updates global commandedPWM and steering_angle
// =========================================================================
void parseSimpleCommand(String data) {
    data.trim(); // Remove potential whitespace
    if (!data.startsWith("<") || !data.endsWith(">")) {
        Serial.println("Invalid format: Missing < or >");
        return;
    }
    // Remove < and >
    data = data.substring(1, data.length() - 1);

    // Check if there's a comma for angle data
    int commaIndex = data.indexOf(',');

    if (commaIndex != -1) {
        // Format <pwm,angle>
        String pwmStr = data.substring(0, commaIndex);
        String angleStr = data.substring(commaIndex + 1);
        int pwmVal = pwmStr.toInt();
        int angleVal = angleStr.toInt();

        // Validate PWM
        if (pwmVal >= 0 && pwmVal <= 255) {
            commandedPWM = pwmVal;
            Serial.print("Received PWM: "); Serial.print(commandedPWM);
        } else {
            Serial.print("Invalid PWM value: "); Serial.println(pwmStr);
        }
        // Validate and set Angle (optional)
        if (angleVal >= 0 && angleVal <= 180) { // Or your specific angle limits
             steering_angle = angleVal;
             Serial.print(" | Received Angle: "); Serial.println(steering_angle);
        } else {
             Serial.print(" | Invalid Angle value: "); Serial.println(angleStr);
        }

    } else {
        // Format <pwm> - Only PWM value provided
        int pwmVal = data.toInt();
        if (pwmVal >= 0 && pwmVal <= 255) {
            commandedPWM = pwmVal;
            Serial.print("Received PWM: "); Serial.println(commandedPWM);
        } else {
            Serial.print("Invalid PWM value: "); Serial.println(data);
        }
    }
}


// =========================================================================
// FUNCTION: displayData (MODIFIED)
// Prints Commanded PWM and Measured RPM.
// =========================================================================
void displayData() {
    static unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL_MS = 100; // Print data every 100ms

    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();

        float measuredRPM;
        // Atomically read currentRPM
        noInterrupts();
        measuredRPM = currentRPM;
        interrupts();

        // Print values in a format suitable for Arduino Serial Plotter
        Serial.print("CommandPWM:");
        Serial.print(commandedPWM);
        Serial.print(","); // Comma separator for plotter
        Serial.print("MeasuredRPM:");
        Serial.println(measuredRPM);
    }
}


// =========================================================================
// SETUP FUNCTION (Simplified)
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Motor RPM Reader Initializing...");
    Serial.println("Send PWM command like:  or <100,90>"); // Instructions

    // --- Pin Modes ---
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(IS_R, INPUT);
    pinMode(IS_L, INPUT);
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);

    // --- Initial State ---
    digitalWrite(R_EN, LOW); // Start disabled
    digitalWrite(L_EN, LOW);
    analogWrite(LPWM, 0);
    digitalWrite(RPWM, LOW);
    setMotorPWM(0); // Ensure motor is stopped
    Serial.println("Motor pins initialized.");

    // --- Attach Interrupt ---
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);
    Serial.println("Hall sensor interrupt attached.");

    // --- Initialize Timers & State ---
    lastRpmCalcTime = micros();
    noInterrupts();
    lastRpmCalcPulseCount = pulseCount;
    lastCheckedPulseCount = pulseCount;
    interrupts();
    Serial.println("State variables initialized.");

    // --- Setup ESP32 Timers --- (Keep these for RPM reading)
    esp_timer_create_args_t rpm_timer_args = { .callback = &rpm_timer_callback, .name = "rpm_calc"};
    esp_timer_create(&rpm_timer_args, &rpm_timer_handle);
    esp_timer_start_periodic(rpm_timer_handle, RPM_CALC_INTERVAL_US);
    Serial.println("RPM calculation timer started.");

    esp_timer_create_args_t stop_timer_args = { .callback = &stop_timer_callback, .name = "motor_stop_check"};
    esp_timer_create(&stop_timer_args, &stop_timer_handle);
    esp_timer_start_periodic(stop_timer_handle, STOP_CHECK_INTERVAL_US);
    Serial.println("Stop detection timer started.");

    Serial.println("Setup Complete. Waiting for commands...");
}

// =========================================================================
// MAIN LOOP (Simplified)
// =========================================================================
void loop() {
    // --- Process Incoming Serial Data ---
    while (Serial.available() > 0) {
        char receivedChar = (char)Serial.read();
        if (receivedChar == '<') {
            serialDataIn = ""; // Start new message
            serialDataIn += receivedChar; // Keep start character if needed, or omit
        } else if (receivedChar == '>') {
            if (serialDataIn.startsWith("<")) { // Make sure we started correctly
                 serialDataIn += receivedChar; // Add end character
                 parseSimpleCommand(serialDataIn); // Parse the complete message
                 serialDataIn = ""; // Reset buffer
            }
        } else if (serialDataIn.startsWith("<") && serialDataIn.length() < 20) { // Prevent overflow, allow for <pwm,angle>
            serialDataIn += receivedChar;
        }
         // Handle potential timeout or buffer clear if '>' not received? Optional.
    }

    // --- Apply Commanded PWM ---
    // Apply the latest commanded PWM value to the motor
    // Could alternatively only call this when commandedPWM changes,
    // but calling periodically is fine.
    setMotorPWM(commandedPWM);

    // --- Servo Control --- (Keep if needed)
    control_servo();

    // --- Display Data ---
    displayData(); // Display data periodically

    // No PID calculation needed in the loop anymore
    // Add a small delay if loop runs too fast without PID checks,
    // though timer callbacks and serial checks usually add enough delay.
    // delay(1); // Optional small delay
}
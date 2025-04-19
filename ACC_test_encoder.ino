#include <Arduino.h>
#include <math.h>
#include "esp_timer.h" // For ESP32 specific timer
// #include <string.h> // Not needed for this version

// --- Pin Definitions ---
#define RPWM 2
#define LPWM 15
#define R_EN 0
#define L_EN 4
#define IS_R 25
#define IS_L 26
// #define SERVO_PIN 13 // Servo pin commented out
#define HALL_SENSOR_PIN 12

// --- Servo Constants --- (Optional - commented out)
// #define MIN_PULSE_WIDTH 500
// #define MAX_PULSE_WIDTH 2500
// #define REFRESH_INTERVAL 20000 // 50Hz
// #define SERVO_MIN_ANGLE 40
// #define SERVO_MAX_ANGLE 130

// --- Motor & Driver Constants ---
// !!! MUST CALIBRATE CURRENT_LIMIT_MV FOR YOUR BTS7960 BOARD !!!
#define CURRENT_LIMIT_MV 7000 // Example: Needs calibration!

// --- Encoder / RPM Calculation ---
const int PULSES_PER_REV = 5;
volatile unsigned long pulseCount = 0; // DEFINED ONCE
volatile float currentRPM = 0.0;       // Measured RPM from encoder
const unsigned long RPM_CALC_INTERVAL_MS = 50; // Update RPM calculation every 50ms
const unsigned long RPM_CALC_INTERVAL_US = RPM_CALC_INTERVAL_MS * 1000;
volatile unsigned long lastRpmCalcTime = 0;
volatile unsigned long lastRpmCalcPulseCount = 0;
esp_timer_handle_t rpm_timer_handle;

// --- Stop Detection Timer ---
const int STOP_CHECK_INTERVAL_US = 1000000; // 1 second
volatile unsigned long lastCheckedPulseCount = 0;
bool motorRunning = false; // Flag set by setMotorPWM
esp_timer_handle_t stop_timer_handle;

// --- REMOVED Serial Communication Buffer ---
// String serialDataIn = ""; // No longer needed

// --- Global State Variables ---
int commandedPWM = 0;        // The PWM value SET IN SETUP() to apply
// int32_t steering_angle = 90; // Optional: Keep steering angle if needed (Commented out)

// --- REMOVED Variables ---
// float RPM_setpoint = 0.0; // No RPM setpoint in this version
// int32_t received_speed = 0; // No speed received
// float integral = 0.0; // No PID
// float previousErrorRPM = 0.0; // No PID
// unsigned long lastPIDRunTime = 0; // No PID
// int currentPWM = 0; // Renamed to commandedPWM or read directly

// =========================================================================
// INTERRUPT SERVICE ROUTINE (ISR) - Hall Sensor / Encoder
// =========================================================================
void IRAM_ATTR hallSensorISR() {
    pulseCount++;
}

// =========================================================================
// TIMER CALLBACK - RPM Calculation
// =========================================================================
void IRAM_ATTR rpm_timer_callback(void *arg) {
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
    } else if (deltaTime_us > RPM_CALC_INTERVAL_US * 2) { // If no pulses for a while
         // Force RPM to 0 if commanded PWM is very low (motor should be stopped)
         // Use the globally set commandedPWM here
         if (commandedPWM < 5) { // Adjust threshold if needed
              calculatedRPM = 0.0;
         }
    }

    noInterrupts();
    currentRPM = calculatedRPM; // Update the global measured RPM
    interrupts();

    lastRpmCalcTime = currentTime_us;
    lastRpmCalcPulseCount = currentPulseReading;
}

// =========================================================================
// TIMER CALLBACK - Stop Detection
// =========================================================================
void IRAM_ATTR stop_timer_callback(void *arg) {
    unsigned long currentPulseReading;
    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();

    // Force RPM to 0 if motor was running but pulses stopped for 1 sec
    if (motorRunning && currentPulseReading == lastCheckedPulseCount) {
        noInterrupts();
        currentRPM = 0.0;
        interrupts();
    }
    lastCheckedPulseCount = currentPulseReading;
}

// =========================================================================
// FUNCTION: setMotorPWM (Applies PWM to driver)
// =========================================================================
void setMotorPWM(int pwmValue) {
    pwmValue = constrain(pwmValue, 0, 255);

    uint16_t raw_IS_R = analogRead(IS_R);
    float voltage_IS_R_mV = raw_IS_R * (3300.0 / 4095.0);

    // Check current limit (NEEDS CALIBRATION)
    if (voltage_IS_R_mV < CURRENT_LIMIT_MV ) {
        digitalWrite(R_EN, HIGH);
        digitalWrite(L_EN, HIGH);
        analogWrite(LPWM, pwmValue);
        digitalWrite(RPWM, LOW);
        motorRunning = (pwmValue > 5); // Motor considered running if PWM > ~0
    } else {
        Serial.println("!!! CURRENT LIMIT EXCEEDED !!!");
        digitalWrite(R_EN, LOW);
        digitalWrite(L_EN, LOW);
        analogWrite(LPWM, 0);
        digitalWrite(RPWM, LOW);
        motorRunning = false;
        // No integral to reset
    }
}

// =========================================================================
// FUNCTION: control_servo (Optional - Commented out)
// =========================================================================
/*
void control_servo() {
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
*/

// =========================================================================
// FUNCTION: parseSimpleCommand (REMOVED - No longer needed)
// =========================================================================
// void parseSimpleCommand(String data) { ... } // Function definition removed

// =========================================================================
// FUNCTION: displayData (Prints Commanded PWM and Measured RPM)
// =========================================================================
void displayData() {
    static unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL_MS = 200; // Print data every 200ms for readability

    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();
        float measuredRPM;
        noInterrupts();
        measuredRPM = currentRPM; // Read the latest calculated RPM
        interrupts();

        // Print values clearly labelled for data collection
        // commandedPWM now shows the value set in setup()
        Serial.print("SetPWM: ");
        Serial.print(commandedPWM);
        Serial.print("\t MeasuredRPM: "); // Use tab for spacing
        Serial.println(measuredRPM);

        // Optional: Use plotter format if preferred
        // Serial.print("SetPWM:"); Serial.print(commandedPWM);
        // Serial.print(",MeasuredRPM:"); Serial.println(measuredRPM);
    }
}

// =========================================================================
// SETUP FUNCTION
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("========================================");
    Serial.println("ESP32 Motor Characterization Tool");
    Serial.println("Purpose: Measure RPM for a FIXED PWM input.");
    Serial.println("PWM value is set directly in the setup() function.");
    // Serial.println("Send commands via Serial Monitor like:"); // Removed
    // Serial.println("  <pwm_value>          (e.g., )"); // Removed
    // Serial.println("  <pwm_value,angle>    (e.g., <150,90>)"); // Removed
    Serial.println("========================================");

    // --- Pin Modes ---
    // pinMode(SERVO_PIN, OUTPUT); digitalWrite(SERVO_PIN, LOW); // Servo pin commented out
    pinMode(RPWM, OUTPUT); pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT); pinMode(L_EN, OUTPUT);
    pinMode(IS_R, INPUT); pinMode(IS_L, INPUT);
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);

    // --- Initial State ---
    digitalWrite(R_EN, LOW); digitalWrite(L_EN, LOW);
    analogWrite(LPWM, 0); digitalWrite(RPWM, LOW);
    // setMotorPWM(0); // Set PWM below

    // **********************************************************
    // ***** SET YOUR DESIRED FIXED PWM VALUE HERE (0-255) ******
    commandedPWM = 150; // <<< CHANGE THIS VALUE FOR TESTING <<<
    // **********************************************************
    Serial.print("Setting fixed PWM to: "); Serial.println(commandedPWM);

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

    // --- Setup ESP32 Timers ---
    esp_timer_create_args_t rpm_timer_args = { .callback = &rpm_timer_callback, .name = "rpm_calc"};
    esp_timer_create(&rpm_timer_args, &rpm_timer_handle);
    esp_timer_start_periodic(rpm_timer_handle, RPM_CALC_INTERVAL_US);
    Serial.println("RPM calculation timer started.");

    esp_timer_create_args_t stop_timer_args = { .callback = &stop_timer_callback, .name = "motor_stop_check"};
    esp_timer_create(&stop_timer_args, &stop_timer_handle);
    esp_timer_start_periodic(stop_timer_handle, STOP_CHECK_INTERVAL_US);
    Serial.println("Stop detection timer started.");

    Serial.println("\nSetup Complete. Running motor at fixed PWM.");
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
    // --- Check for and Process Incoming Serial Commands ---
    // REMOVED - No serial input processing in this version
    /*
    while (Serial.available() > 0) {
        // ... Serial handling code removed ...
    }
    */

    // --- Apply Commanded PWM to Motor ---
    // Directly use the PWM value set in setup()
    setMotorPWM(commandedPWM);

    // --- Control Servo (Optional - Commented out) ---
    // control_servo();

    // --- Display Data ---
    // Periodically print the fixed PWM and the measured RPM
    displayData();

    // No PID calculations here
    // delay(1); // Small delay can sometimes help stability if loop is too tight
}

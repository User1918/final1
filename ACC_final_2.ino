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
#define SERVO_PIN 13
#define HALL_SENSOR_PIN 12

// --- Servo Constants --- (Keep these)
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
volatile float currentRPM = 0.0;
const unsigned long RPM_CALC_INTERVAL_MS = 50;
const unsigned long RPM_CALC_INTERVAL_US = RPM_CALC_INTERVAL_MS * 1000;
volatile unsigned long lastRpmCalcTime = 0;
volatile unsigned long lastRpmCalcPulseCount = 0;
esp_timer_handle_t rpm_timer_handle;

// --- PID Controller ---
float RPM_setpoint = 0.0; // Desired speed (RPM)

// *** NEW PID GAINS - MUST BE RETUNED ***
// These gains now relate RPM error to an RPM-based control effort
float Kp = 0.5;  // Example starting value - TUNE ME!
float Ki = 0.1;  // Example starting value - TUNE ME!
float Kd = 0.005; // Example starting value - TUNE ME!

// --- PID Timing & State Variables ---
const unsigned long PID_INTERVAL_MS = 10;
const float PID_SAMPLE_TIME_S = (float)PID_INTERVAL_MS / 1000.0;
float integral = 0.0;           // PID integral state
float previousErrorRPM = 0.0;   // Store previous error in RPM
unsigned long lastPIDRunTime = 0;
int currentPWM = 0; // Stores the last calculated PWM value

// *** NEW: RPM-to-PWM Transformation Constants - MUST BE TUNED ***
// These map the target RPM effort calculated by PID to a PWM value.
// PWM = RPM_TO_PWM_SLOPE * targetEffort_RPM + RPM_TO_PWM_OFFSET
// Needs experimental determination for your specific motor+load!
const float RPM_TO_PWM_SLOPE = 2.0;  // Example: 2 PWM units per RPM - TUNE ME!
const float RPM_TO_PWM_OFFSET = 20.0; // Example: PWM needed to start motion - TUNE ME!
const float MAX_RPM_ESTIMATE = 150.0; // Example: Estimated max possible RPM for anti-windup - TUNE ME!


// --- Stop Detection Timer --- (Keep these)
const int STOP_CHECK_INTERVAL_US = 1000000;
volatile unsigned long lastCheckedPulseCount = 0;
bool motorRunning = false;
esp_timer_handle_t stop_timer_handle;

// --- Serial Communication Buffer --- (Keep these)
const int SERIAL_BUFFER_SIZE = 10;
byte serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// --- Global State Variables --- (Keep these)
int32_t steering_angle = 90;
int32_t received_speed = 0;

// =========================================================================
// INTERRUPT SERVICE ROUTINE (ISR) - Hall Sensor / Encoder
// =========================================================================
void IRAM_ATTR hallSensorISR() {
    pulseCount++;
}

// =========================================================================
// TIMER CALLBACK - RPM Calculation (Keep this function as is)
// =========================================================================
void IRAM_ATTR rpm_timer_callback(void *arg) {
    // ... (Keep the existing code for rpm_timer_callback) ...
     unsigned long currentTime_us = micros();
    unsigned long currentPulseReading;

    // Atomically read pulseCount
    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();

    unsigned long deltaTime_us = currentTime_us - lastRpmCalcTime;
    unsigned long deltaPulses = currentPulseReading - lastRpmCalcPulseCount;

    float calculatedRPM = 0.0;
    if (deltaTime_us > 0 && deltaPulses > 0) { // Calculate only if time and pulses have changed
        float pulses_per_second = (float)deltaPulses * 1000000.0 / (float)deltaTime_us;
        float rps = pulses_per_second / (float)PULSES_PER_REV; // Revolutions per second
        calculatedRPM = rps * 60.0;                            // Revolutions per minute
    } else if (deltaTime_us > RPM_CALC_INTERVAL_US * 2) { // If no pulses for a while, assume 0 RPM
         // Check if motor *should* be running based on setpoint before forcing 0
         if (abs(RPM_setpoint) < 0.1) { // Check if setpoint is effectively zero
              calculatedRPM = 0.0;
         }
    }

    // Atomically update currentRPM
    noInterrupts();
    currentRPM = calculatedRPM; // No filter for now
    interrupts();

    // Store values for next calculation
    lastRpmCalcTime = currentTime_us;
    lastRpmCalcPulseCount = currentPulseReading;
}


// =========================================================================
// TIMER CALLBACK - Stop Detection (Keep this function as is)
// =========================================================================
void IRAM_ATTR stop_timer_callback(void *arg) {
   // ... (Keep the existing code for stop_timer_callback) ...
    unsigned long currentPulseReading;

    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();

    // If the motor is supposed to be running (setpoint > 0) but pulses haven't changed
    if (motorRunning && currentPulseReading == lastCheckedPulseCount) {
        noInterrupts();
        currentRPM = 0.0; // Force measured RPM to zero
        interrupts();
    }
    lastCheckedPulseCount = currentPulseReading; // Update for next check
}

// =========================================================================
// FUNCTION: setMotorPWM (Keep this function as is)
// Applies the given PWM value (0-255) to the motor driver.
// =========================================================================
void setMotorPWM(int pwmValue) {
   // ... (Keep the existing code for setMotorPWM, including current check) ...
    pwmValue = constrain(pwmValue, 0, 255);
    currentPWM = pwmValue; // Store the value being applied

    uint16_t raw_IS_R = analogRead(IS_R);
    float voltage_IS_R_mV = raw_IS_R * (3300.0 / 4095.0);

    // Placeholder for actual current check - NEEDS CALIBRATION
    if (voltage_IS_R_mV < CURRENT_LIMIT_MV ) {
        digitalWrite(R_EN, HIGH);
        digitalWrite(L_EN, HIGH);
        analogWrite(LPWM, pwmValue);
        digitalWrite(RPWM, LOW);
        motorRunning = (pwmValue > 0);
    } else {
        // Current limit exceeded! Stop the motor immediately
        Serial.println("!!! CURRENT LIMIT EXCEEDED !!!");
        digitalWrite(R_EN, LOW);
        digitalWrite(L_EN, LOW);
        analogWrite(LPWM, 0);
        digitalWrite(RPWM, LOW);
        motorRunning = false;
        integral = 0; // Reset integral term on fault
        currentPWM = 0;
    }
}

// =========================================================================
// FUNCTION: calculatePID_RPM_Output (NEW PID Logic)
// Calculates the required control effort (scaled in RPM units).
// Input: setpointRPM, measuredRPM
// Output: float targetEffort_RPM (unconstrained)
// =========================================================================
float calculatePID_RPM_Output(float setpointRPM, float measuredRPM) {
    // --- Calculate Error (in RPM) ---
    float errorRPM = setpointRPM - measuredRPM;

    // --- Integral Term (with anti-windup) ---
    // Basic anti-windup: Clamp the integral term itself.
    // Estimate max contribution needed from integral (e.g., enough to reach max RPM)
    float maxIntegralContribution_RPM = MAX_RPM_ESTIMATE; // Example limit - TUNE ME!
    float maxIntegral = (Ki > 0.001) ? maxIntegralContribution_RPM / Ki : 1e9; // Avoid division by zero

    // Only integrate if not already saturated (more advanced anti-windup could be added here)
    // For now, just accumulate and clamp later
    integral += errorRPM * PID_SAMPLE_TIME_S;
    integral = constrain(integral, -maxIntegral, maxIntegral); // Apply clamping

    // --- Derivative Term ---
    float derivative = 0;
    if (PID_SAMPLE_TIME_S > 0) {
        derivative = (errorRPM - previousErrorRPM) / PID_SAMPLE_TIME_S;
    }

    // --- PID Calculation (Output is RPM-based effort) ---
    float p_term = Kp * errorRPM;
    float i_term = Ki * integral;
    float d_term = Kd * derivative;
    float pidOutput_RPM = p_term + i_term + d_term;

    // --- Update State for Next Iteration ---
    previousErrorRPM = errorRPM; // Store RPM error for next derivative calculation

    // --- Return calculated effort (RPM units) ---
    return pidOutput_RPM;
}

// =========================================================================
// FUNCTION: transformRPMtoPWM (NEW Transformation Logic)
// Converts the target RPM effort from PID into a PWM value (0-255).
// Input: targetEffort_RPM (float)
// Output: int pwmValue (0-255)
// =========================================================================
int transformRPMtoPWM(float targetEffort_RPM) {
    // Apply linear transformation: PWM = m * RPM + c
    // Use the tuned constants RPM_TO_PWM_SLOPE and RPM_TO_PWM_OFFSET
    float calculatedPWM_f = RPM_TO_PWM_SLOPE * targetEffort_RPM + RPM_TO_PWM_OFFSET;

    // Constrain the result to the valid PWM range
    int pwmOut = constrain((int)round(calculatedPWM_f), 0, 255);

    return pwmOut;
}


// =========================================================================
// FUNCTION: control_servo (Keep this function as is)
// =========================================================================
void control_servo() {
   // ... (Keep the existing code for control_servo) ...
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
// FUNCTION: parseSerialData (Keep this function as is)
// =========================================================================
void parseSerialData() {
   // ... (Keep the existing code for parseSerialData) ...
    if (serialBuffer[0] == '<' && serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') {
        steering_angle = *((int32_t*)&serialBuffer[1]);
        received_speed = *((int32_t*)&serialBuffer[5]);
        RPM_setpoint = (float)received_speed;
    } else {
        Serial.println("Invalid serial packet received.");
    }
    serialBufferIndex = 0;
}

// =========================================================================
// FUNCTION: displayData (Keep this function as is)
// =========================================================================
void displayData() {
   // ... (Keep the existing code for displayData) ...
    static unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL_MS = 100; // Print data every 100ms

    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();
        float measuredRPM;
        noInterrupts();
        measuredRPM = currentRPM;
        interrupts();

        Serial.print("SetpointRPM:");
        Serial.print(RPM_setpoint);
        Serial.print(","); // Comma separator for plotter
        Serial.print("MeasuredRPM:");
        Serial.print(measuredRPM);
        Serial.print(",");
        Serial.print("PWM:"); // Also plot PWM now
        Serial.println(currentPWM);
    }
}


// =========================================================================
// SETUP FUNCTION (Keep mostly as is)
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Motor Control Initializing (RPM PID Output)...");

    // --- Pin Modes --- (Keep setup)
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(IS_R, INPUT);
    pinMode(IS_L, INPUT);
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);

    // --- Initial State --- (Keep setup)
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    analogWrite(LPWM, 0);
    digitalWrite(RPWM, LOW);
    setMotorPWM(0);

    // --- Attach Interrupt --- (Keep setup)
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);

    // --- Initialize Timers & State --- (Keep setup)
    lastRpmCalcTime = micros();
    lastPIDRunTime = millis();
    noInterrupts();
    lastRpmCalcPulseCount = pulseCount;
    lastCheckedPulseCount = pulseCount;
    interrupts();
    Serial.println("State variables initialized.");

    // --- Setup ESP32 Timers --- (Keep setup)
    esp_timer_create_args_t rpm_timer_args = { .callback = &rpm_timer_callback, .name = "rpm_calc"};
    esp_timer_create(&rpm_timer_args, &rpm_timer_handle);
    esp_timer_start_periodic(rpm_timer_handle, RPM_CALC_INTERVAL_US);

    esp_timer_create_args_t stop_timer_args = { .callback = &stop_timer_callback, .name = "motor_stop_check"};
    esp_timer_create(&stop_timer_args, &stop_timer_handle);
    esp_timer_start_periodic(stop_timer_handle, STOP_CHECK_INTERVAL_US);

    Serial.println("Setup Complete. Waiting for data...");
}

// =========================================================================
// MAIN LOOP (Updated PID Flow)
// =========================================================================
void loop() {
    // --- Process Incoming Serial Data --- (Keep as is)
    while (Serial.available() > 0) {
        // ... (Keep existing serial parsing logic) ...
         uint8_t byteRead = Serial.read();
        if (serialBufferIndex == 0 && byteRead != '<') {
            continue; // Wait for start marker
        }
        if (serialBufferIndex < SERIAL_BUFFER_SIZE) {
            serialBuffer[serialBufferIndex++] = byteRead;
        }
        if (serialBufferIndex == SERIAL_BUFFER_SIZE) {
            if (serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') {
                parseSerialData(); // Updates RPM_setpoint and steering_angle
            } else {
                 Serial.println("Serial Frame Error: No end marker.");
            }
             serialBufferIndex = 0; // Always reset index
        }
    }


    // --- PID Control Loop (Runs periodically) ---
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDRunTime >= PID_INTERVAL_MS) {
        lastPIDRunTime = currentMillis; // Update time of this PID run

        // Get current speed measurement
        float measuredRPM;
        noInterrupts();
        measuredRPM = currentRPM;
        interrupts();

        // --- NEW PID FLOW ---
        // 1. Reset integral if setpoint is zero (helps prevent windup at stop)
        if (abs(RPM_setpoint) < 0.1) {
             integral = 0.0;
             previousErrorRPM = 0.0; // Reset previous error as well
        }

        // 2. Calculate desired control effort in RPM units using PID
        float targetEffort_RPM = calculatePID_RPM_Output(RPM_setpoint, measuredRPM);

        // 3. Transform the RPM-based effort into a PWM value
        int targetPWM = transformRPMtoPWM(targetEffort_RPM);

        // 4. Apply the calculated PWM to the motor
        setMotorPWM(targetPWM);
        // --- End of NEW PID FLOW ---
    }

    // --- Servo Control --- (Keep as is)
    control_servo();

    // --- Display Data --- (Keep as is - now plots PWM too)
    displayData();
}
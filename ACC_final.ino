#include <Arduino.h>
#include <math.h>
#include "esp_timer.h" // For ESP32 specific timer

// --- Pin Definitions ---
#define RPWM 2       // Motor Right PWM
#define LPWM 15      // Motor Left PWM
#define R_EN 0       // Motor Right Enable
#define L_EN 4       // Motor Left Enable
#define IS_R 25      // Motor Right Current Sense
#define IS_L 26      // Motor Left Current Sense

#define SERVO_PIN 13 // Steering Servo Pin

#define HALL_SENSOR_PIN 12 // Encoder Input Pin

// --- Servo Constants ---
#define MIN_PULSE_WIDTH 500  // Minimum pulse width for servo (microseconds)
#define MAX_PULSE_WIDTH 2500 // Maximum pulse width for servo (microseconds)
#define REFRESH_INTERVAL 20000 // Servo refresh interval (microseconds, 50Hz) - Adjusted from original
#define SERVO_MIN_ANGLE 40   // Minimum allowed steering angle
#define SERVO_MAX_ANGLE 130  // Maximum allowed steering angle

// --- Motor & Driver Constants ---
#define CURRENT_LIMIT_MV 7000 // Example: Set a limit in mV corresponding to max current.
                              // **IMPORTANT**: This value needs calibration based on your BTS7960 board's
                              // IS pin output (V/A) and the desired current limit (mA).
                              // The original '7000' seemed like mA, but the reading is analog voltage.
                              // Assuming 100A max -> 7A continuous for BTS7960.
                              // The IS pin might output e.g. 8.5mV/A or similar. Calibrate this!

// --- Encoder / RPM Calculation ---
const int PULSES_PER_REV = 5;           // Pulses from Hall sensor per motor revolution
volatile unsigned long pulseCount = 0;  // Incremented by ISR
volatile float currentRPM = 0.0;        // Calculated RPM (updated by timer)
const unsigned long RPM_CALC_INTERVAL_MS = 50; // How often RPM is calculated
const unsigned long RPM_CALC_INTERVAL_US = RPM_CALC_INTERVAL_MS * 1000;
volatile unsigned long lastRpmCalcTime = 0;
volatile unsigned long lastRpmCalcPulseCount = 0;
esp_timer_handle_t rpm_timer_handle;

// --- PID Controller ---
float RPM_setpoint = 0.0; // Desired speed (RPM), updated from Serial
float Kp = 1.0;           // Proportional gain
float Ki = 0.5;           // Integral gain
float Kd = 0.01;          // Derivative gain

// --- PID Timing & State Variables ---
const unsigned long PID_INTERVAL_MS = 10;     // How often PID calculation runs
const float PID_SAMPLE_TIME_S = (float)PID_INTERVAL_MS / 1000.0; // PID sample time in seconds
float previousError = 0.0;
float integral = 0.0;
unsigned long lastPIDRunTime = 0; // For timing PID execution in loop()
int currentPWM = 0; // Stores the last calculated PWM value

// --- Linear Feedforward/Error Transformation Constants (Used inside PID) ---
// These constants transform the error (RPM) before PID calculation.
// Ensure they are tuned for RPM units if error is in RPM.
// PWM = a * error_rpm + b  <- This is how they are used in calculatePID
const float a = 80/339; // Example: Adjust scaling factor (Needs tuning)
const float b = 180/113; // Example: Adjust offset (Needs tuning)
                     // Original values: 80.0/339.0 and 180.0/113.0 might need re-evaluation.


// --- Stop Detection Timer ---
const int STOP_CHECK_INTERVAL_US = 1000000; // Check every 1 second
volatile unsigned long lastCheckedPulseCount = 0;
bool motorRunning = false; // Flag to indicate if motor is intended to be running
esp_timer_handle_t stop_timer_handle;

// --- Serial Communication Buffer ---
const int SERIAL_BUFFER_SIZE = 10; // < + 4 bytes angle + 4 bytes speed + >
byte serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// --- Global State Variables ---
int32_t steering_angle = 90; // Desired steering angle (0-180, center likely 90), updated from Serial
int32_t received_speed = 0;  // Temporary storage for speed received via Serial

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
         // Otherwise, keep the last known RPM or let the stop timer handle it
    }


    // Atomically update currentRPM
    noInterrupts();
    // Basic low-pass filter (optional, adjust alpha for more/less smoothing)
    // float alpha = 0.8;
    // currentRPM = alpha * currentRPM + (1.0 - alpha) * calculatedRPM;
    currentRPM = calculatedRPM; // No filter for now
    interrupts();

    // Store values for next calculation
    lastRpmCalcTime = currentTime_us;
    lastRpmCalcPulseCount = currentPulseReading;
}


// =========================================================================
// TIMER CALLBACK - Stop Detection
// Checks if pulses have stopped accumulating when motor should be running
// =========================================================================
void IRAM_ATTR stop_timer_callback(void *arg) {
    unsigned long currentPulseReading;

    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();

    // If the motor is supposed to be running (setpoint > 0) but pulses haven't changed
    if (motorRunning && currentPulseReading == lastCheckedPulseCount) {
        noInterrupts();
        currentRPM = 0.0; // Force measured RPM to zero
        interrupts();
         // Optional: Could also trigger an error flag here
    }
    lastCheckedPulseCount = currentPulseReading; // Update for next check
}

// =========================================================================
// FUNCTION: setMotorPWM
// Applies the given PWM value to the motor driver.
// Includes basic current monitoring (NEEDS CALIBRATION).
// Assumes positive PWM drives motor forward.
// =========================================================================
void setMotorPWM(int pwmValue) {
    // Constrain PWM to valid range
    pwmValue = constrain(pwmValue, 0, 255);
    currentPWM = pwmValue; // Store the value being applied

    // --- Current Monitoring ---
    // **WARNING:** This requires calibration! Read voltage from IS pins,
    // convert it to current based on your BTS7960 board's specifics.
    // The relationship is usually I = V_is / K, where K depends on sense resistor.
    // For example, if K = 0.0085 V/A (8.5mV/A) and ADC is 12-bit (0-4095) on 3.3V ref:
    // V_is = analogRead(IS_pin) * (3300.0 / 4095.0); // Voltage in mV
    // Current_mA = V_is / 0.0085; // If K is 8.5mV/A = 0.0085 V/A

    uint16_t raw_IS_R = analogRead(IS_R);
    uint16_t raw_IS_L = analogRead(IS_L); // Read L IS pin as well if used for braking/reverse

    // Convert raw ADC reading to millivolts (assuming 3.3V ref, 12-bit ADC)
    float voltage_IS_R_mV = raw_IS_R * (3300.0 / 4095.0);
    // float voltage_IS_L_mV = raw_IS_L * (3300.0 / 4095.0); // If needed

    // Placeholder for actual current check - using voltage directly for now
    // Replace CURRENT_LIMIT_MV with a calibrated value.
    if (voltage_IS_R_mV < CURRENT_LIMIT_MV /* && voltage_IS_L_mV < CURRENT_LIMIT_MV */) {
        // Current is within limits, enable driver
        digitalWrite(R_EN, HIGH);
        digitalWrite(L_EN, HIGH); // Enable both for BTS7960

        // Apply PWM (Forward direction assumed)
        // To drive forward with BTS7960, typically:
        // RPWM = PWM signal, LPWM = LOW (or vice versa depending on wiring)
        analogWrite(LPWM, pwmValue); // Send PWM signal to Left PWM input
        digitalWrite(RPWM, LOW);    // Keep Right PWM input LOW for forward

        motorRunning = (pwmValue > 0); // Update motor running state
    } else {
        // Current limit exceeded! Stop the motor immediately
        Serial.println("!!! CURRENT LIMIT EXCEEDED !!!");
        digitalWrite(R_EN, LOW); // Disable driver
        digitalWrite(L_EN, LOW); // Disable driver
        analogWrite(LPWM, 0);    // Ensure PWM is off
        digitalWrite(RPWM, LOW);
        motorRunning = false;
        integral = 0; // Reset integral term on fault
        currentPWM = 0;
    }
}

// =========================================================================
// FUNCTION: calculatePID
// Calculates the required PWM using a FIXED sample time based on RPM error.
// Includes the linear transformation `a*error + b` as per original code.
// Returns the calculated PWM value (0-255).
// =========================================================================
int calculatePID(float setpointRPM, float measuredRPM) {
    // --- Calculate Error (in RPM) ---
    float errorRPM = setpointRPM - measuredRPM;

    // --- Linear transformation (applied to error, as in original code) ---
    // This step might need tuning or reconsideration based on desired behavior.
    // It scales the error before it's used in the PID calculation.
    float transformedError = a * errorRPM + b;
    // Serial.print(" ErrorRPM: "); Serial.print(errorRPM); // Debug
    // Serial.print(" TransformedError: "); Serial.print(transformedError); // Debug

    // --- Integral Term (using fixed sample time) ---
    integral += transformedError * PID_SAMPLE_TIME_S;

    // --- Anti-windup (optional but recommended) ---
    // Constrain the integral term to prevent excessive buildup
    // float maxIntegral = 255.0 / Ki; // Example limit, adjust as needed
    // integral = constrain(integral, -maxIntegral, maxIntegral);


    // --- Derivative Term (using fixed sample time) ---
    float derivative = 0;
    // Check if sample time is valid (should always be true if > 0)
    if (PID_SAMPLE_TIME_S > 0) {
        // Use change in transformed error for derivative calculation
        derivative = (transformedError - previousError) / PID_SAMPLE_TIME_S;
    }

    // --- PID Calculation ---
    // Calculates the total control output based on transformed error
    float pidOutput = Kp * transformedError + Ki * integral + Kd * derivative;
    // Serial.print(" P: "); Serial.print(Kp * transformedError); // Debug
    // Serial.print(" I: "); Serial.print(Ki * integral); // Debug
    // Serial.print(" D: "); Serial.print(Kd * derivative); // Debug
    // Serial.print(" PID_Out: "); Serial.print(pidOutput); // Debug


    // --- Update State for Next Iteration ---
    previousError = transformedError; // Store transformed error for next derivative calculation

    // --- Constrain and Return PWM ---
    // The PID output is directly used as PWM signal strength
    int pwmOut = constrain((int)round(pidOutput), 0, 255); // Round and constrain
    // Serial.print(" PWM_Out: "); Serial.println(pwmOut); // Debug
    return pwmOut;
}


// =========================================================================
// FUNCTION: control_servo
// Sends pulses to the servo based on the global `steering_angle`.
// =========================================================================
void control_servo() {
    static unsigned long lastServoUpdate = 0;
    unsigned long now = micros();

    // Refresh servo periodically to maintain position
    if (now - lastServoUpdate >= REFRESH_INTERVAL) {
        lastServoUpdate = now;

        // Constrain angle to safe limits
        int constrainedAngle = constrain(steering_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        // Map angle (0-180 range assumed for mapping) to pulse width
        int pulseWidth = map(constrainedAngle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

        // Send the pulse
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(pulseWidth); // Keep pin HIGH for calculated duration
        digitalWrite(SERVO_PIN, LOW);
        // No need for the second delayMicroseconds if using periodic updates
    }
     // If not using ESP32Servo library, this manual pulsing needs to be done periodically.
     // The REFRESH_INTERVAL handles this timing.
}


// =========================================================================
// FUNCTION: parseSerialData
// Processes bytes from the serial buffer to extract angle and speed.
// Expected format: '<'[4-byte angle][4-byte speed]'>' (Total 10 bytes)
// Updates global `steering_angle` and `received_speed`.
// =========================================================================
void parseSerialData() {
    if (serialBuffer[0] == '<' && serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') {
        // Extract angle (bytes 1-4) - assumes little-endian from Pi
        steering_angle = *((int32_t*)&serialBuffer[1]);

        // Extract speed (bytes 5-8) - assumes little-endian from Pi
        received_speed = *((int32_t*)&serialBuffer[5]);

        // Update the RPM setpoint with the received speed
        // Add any scaling or unit conversion if needed (e.g., if Pi sends rad/s)
        RPM_setpoint = (float)received_speed; // Assuming Pi sends desired RPM directly

        // Serial.print("Received Angle: "); Serial.print(steering_angle);
        // Serial.print(" | Received Speed (RPM): "); Serial.println(RPM_setpoint);
    } else {
        Serial.println("Invalid serial packet received.");
    }
    // Reset buffer index for next packet
    serialBufferIndex = 0;
}

// =========================================================================
// FUNCTION: displayData
// Prints Setpoint RPM and Measured RPM for plotting/debugging.
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
        Serial.print("SetpointRPM:");
        Serial.print(RPM_setpoint);
        Serial.print(","); // Comma separator for plotter
        Serial.print("MeasuredRPM:");
        Serial.println(measuredRPM);

        // Optional: Print other values for debugging
        // Serial.print(" | PWM:"); Serial.print(currentPWM);
        // Serial.print(" | Angle:"); Serial.print(steering_angle);
        // Serial.println();
    }
}


// =========================================================================
// SETUP FUNCTION
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Motor Control Initializing...");

    // --- Pin Modes ---
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW); // Start servo pin low

    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(IS_R, INPUT);
    pinMode(IS_L, INPUT);

    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP); // Enable pull-up for Hall sensor

    // --- Initial State ---
    digitalWrite(R_EN, LOW); // Start with motor driver disabled
    digitalWrite(L_EN, LOW);
    analogWrite(LPWM, 0);
    digitalWrite(RPWM, LOW);
    setMotorPWM(0); // Ensure motor is stopped initially
    Serial.println("Motor pins initialized.");

    // --- Attach Interrupt ---
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING); // Or RISING/CHANGE depending on sensor
    Serial.println("Hall sensor interrupt attached.");

    // --- Initialize Timers & State ---
    lastRpmCalcTime = micros();
    lastPIDRunTime = millis();
    noInterrupts();
    lastRpmCalcPulseCount = pulseCount;
    lastCheckedPulseCount = pulseCount;
    interrupts();
    Serial.println("State variables initialized.");

    // --- Setup ESP32 Timers ---
    // RPM Calculation Timer
    esp_timer_create_args_t rpm_timer_args = {
        .callback = &rpm_timer_callback,
        .name = "rpm_calc"
    };
    esp_timer_create(&rpm_timer_args, &rpm_timer_handle);
    esp_timer_start_periodic(rpm_timer_handle, RPM_CALC_INTERVAL_US);
    Serial.println("RPM calculation timer started.");

    // Stop Detection Timer
    esp_timer_create_args_t stop_timer_args = {
        .callback = &stop_timer_callback,
        .name = "motor_stop_check"
    };
    esp_timer_create(&stop_timer_args, &stop_timer_handle);
    esp_timer_start_periodic(stop_timer_handle, STOP_CHECK_INTERVAL_US);
    Serial.println("Stop detection timer started.");

    Serial.println("Setup Complete. Waiting for data...");
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
    // --- Process Incoming Serial Data ---
    while (Serial.available() > 0) {
        uint8_t byteRead = Serial.read();

        if (serialBufferIndex == 0 && byteRead != '<') {
            continue; // Wait for start marker
        }

        if (serialBufferIndex < SERIAL_BUFFER_SIZE) {
            serialBuffer[serialBufferIndex++] = byteRead;
        }

        // If buffer full, check end marker and parse
        if (serialBufferIndex == SERIAL_BUFFER_SIZE) {
            if (serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') {
                parseSerialData(); // Updates RPM_setpoint and steering_angle
            } else {
                // Invalid packet, reset index
                serialBufferIndex = 0;
                 Serial.println("Serial Frame Error: No end marker.");
            }
             serialBufferIndex = 0; // Always reset index after processing or error
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

        // Calculate the target PWM using the PID function
        int targetPWM = calculatePID(RPM_setpoint, measuredRPM);

        // Apply the calculated PWM to the motor
        setMotorPWM(targetPWM);
    }

    // --- Servo Control ---
    // control_servo needs to be called frequently to maintain position
    // if not using a servo library that handles background updates.
    control_servo(); // Call this on every loop iteration

    // --- Display Data ---
    displayData(); // Display data periodically for plotting/debugging
}
#include <Arduino.h>
#include <math.h>
#include "esp_timer.h"
#include <Wire.h>

// --- Pin Definitions ---
#define RPWM 2         // Right PWM (Might need configuration for ESP32 PWM channels)
#define LPWM 15        // Left PWM  (Might need configuration for ESP32 PWM channels)
#define R_EN 0         // Right Enable
#define L_EN 4         // Left Enable
#define IS_R 25        // Right Current Sense (Analog Input)
#define IS_L 26        // Left Current Sense (Analog Input)
#define SERVO_PIN 13   // Servo Control Pin
#define HALL_SENSOR_PIN 12 // Hall Effect Sensor Input Pin
#define HB100_PIN 34   // HB100 Doppler Sensor Input Pin (Analog or Digital depending on board)

// --- Servo Constants ---
#define MIN_PULSE_WIDTH 500    // Servo minimum pulse width (microseconds)
#define MAX_PULSE_WIDTH 2500   // Servo maximum pulse width (microseconds)
#define REFRESH_INTERVAL 20000 // Servo refresh interval (microseconds)
#define SERVO_MIN_ANGLE 40     // Software limit for servo angle
#define SERVO_MAX_ANGLE 130    // Software limit for servo angle

// --- Motor & Driver Constants ---
#define CURRENT_LIMIT_MV 7000 // Example current limit in millivolts (NEEDS CALIBRATION)

// --- Encoder / RPM Calculation ---
const int PULSES_PER_REV = 5; // Pulses per revolution of the motor/wheel encoder
volatile unsigned long pulseCount = 0;
volatile float currentRPM = 0.0; // Raw RPM calculated by timer
const unsigned long RPM_CALC_INTERVAL_MS = 500; // How often to calculate RPM
const unsigned long RPM_CALC_INTERVAL_US = RPM_CALC_INTERVAL_MS * 1000;
volatile unsigned long lastRpmCalcTime = 0;
volatile unsigned long lastRpmCalcPulseCount = 0;
esp_timer_handle_t rpm_timer_handle;

// --- Stop Detection ---
const int STOP_CHECK_INTERVAL_US = 1000000; // 1 second interval to check if motor stopped
volatile unsigned long lastCheckedPulseCount = 0;
bool motorRunning = false; // Flag if motor PWM is active
esp_timer_handle_t stop_timer_handle;

// --- TOF10120 Constants (Datasheet Corrected) ---
const uint8_t TOF_I2C_ADDRESS = 0xA4; // Datasheet default (164 decimal)
const uint16_t MIN_VALID_DISTANCE_TOF = 100; // Datasheet minimum range (100mm)
const uint16_t MAX_VALID_DISTANCE_TOF = 1800; // Datasheet maximum range (1800mm)

// --- Filtering ---
const uint8_t FILTER_SIZE = 5; // Moving average filter size for ToF
uint16_t distanceBuffer[FILTER_SIZE];
uint8_t bufferIndex = 0;
const float Q = 0.1; // Kalman filter process noise covariance
const float R = 4.0; // Kalman filter measurement noise covariance (Increased R?)
int invalid_count = 0;
const int MAX_INVALID_ALLOWED = 5; // Max consecutive invalid ToF readings

// --- Kalman Filter State ---
float x_est = 0; // Kalman estimated distance
float P_est = 1; // Kalman error covariance estimate
float K = 0;     // Kalman gain
uint16_t filteredDistance = UINT16_MAX; // Final filtered distance (use UINT16_MAX as invalid)
bool tof_ready = false; // Flag indicating if ToF sensor is configured and providing valid data

// --- Intelligent Driver Model (IDM) Constants ---
// !! TUNE THESE PARAMETERS !!
float desired_velocity = 0.0;   // Target RPM when road is clear (Set by RPi)
float a_max = 1.0;              // Max acceleration [m/s^2]
float b_comfort = 1.5;          // Comfortable deceleration [m/s^2]
float T_gap = 1.2;              // Safe time headway [s] (Adjust based on speed/braking)
float s0 = 150;                 // Minimum desired following distance [mm] (MUST BE >= MIN_VALID_DISTANCE_TOF)
// !! TUNE THESE CONTROL THRESHOLDS !!
const uint16_t IDM_STOP_DISTANCE = 150;     // mm - Stop if object closer than this (must be >= MIN_VALID_DISTANCE_TOF)
const uint16_t IDM_CONTROL_MAX_DIST = 1000; // mm - Max distance to apply IDM control
//-----------------------------
float rpm_from_raspberry = 0.0; // RPM command received from RPi
bool override_by_IDM = false;  // Flag if IDM is controlling the speed setpoint

// --- Smoothing & Filtering ---
const float EMA_ALPHA = 0.05; // Exponential Moving Average alpha for RPM smoothing
float g_filteredRPM = 0.0;    // EMA filtered actual RPM
float previousSetpoint = 0.0; // Previous final RPM target for smoothing
float rpm_from_rasp_filtered = 0.0; // Smoothed RPi command RPM
const float MAX_RPM_DELTA = 10.0;  // Max change in RPM setpoint per cycle for smoothing
const float RPM_SMOOTH_ALPHA = 0.05; // Smoothing factor for RPi command RPM

// --- PID Controller ---
// !! TUNE THESE GAINS !!
float Kp = 1.0;
float Ki = 0.5;
float Kd = 0.0;
//----------------------
const unsigned long PID_INTERVAL_MS = 10; // PID loop interval
const float PID_SAMPLE_TIME_S = (float)PID_INTERVAL_MS / 1000.0;
float integral = 0.0;         // PID integral term
float previousErrorRPM = 0.0; // PID previous error for derivative term
unsigned long lastPIDRunTime = 0;
int currentPWM = 0;           // Last PWM value applied to motor

// --- RPM to PWM Transformation ---
// !! TUNE THESE VALUES based on motor characterization !!
const float RPM_TO_PWM_SLOPE = 0.05; // PWM per RPM (approximate)
const float RPM_TO_PWM_OFFSET = 4.6; // PWM offset (minimum PWM to start moving)
//-------------------------------------
const float MAX_RPM_ESTIMATE = 4000; // Estimated max possible RPM (for PID integral clamping)

// --- Serial Communication ---
const int SERIAL_BUFFER_SIZE = 10; // Size of buffer for <angle,speed> packets
byte serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;
int32_t steering_angle = 90; // Default steering angle
int32_t received_speed = 0;  // Raw speed value from RPi

// --- HB100 Doppler Sensor ---
volatile unsigned long hb100PulseCount = 0; // Raw pulse count from HB100 interrupt (if using interrupt)
float hb100Velocity = 0.0;    // Calculated relative velocity in m/s (Ensure calculation is correct)
unsigned long hb100LastTime = 0;
bool lastHB100State = LOW;
const float HB100_WAVELENGTH = 0.032; // Approx. wavelength for 10.525 GHz sensor (meters)
// Note: HB100 reading method (interrupt vs analog/polling) might need adjustment


// =========================================================================
// INTERRUPT SERVICE ROUTINE (ISR) - Hall Sensor / Encoder
// Increments pulse count on falling edge.
// =========================================================================
void IRAM_ATTR hallSensorISR() {
    pulseCount++;
}

// =========================================================================
// TIMER CALLBACK - RPM Calculation
// Calculates RPM based on pulse count over time interval.
// =========================================================================
void IRAM_ATTR rpm_timer_callback(void *arg) {
    unsigned long currentTime_us = micros();
    unsigned long currentPulseReading;

    // Safely read volatile variable
    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();

    unsigned long deltaTime_us = currentTime_us - lastRpmCalcTime;
    unsigned long deltaPulses = currentPulseReading - lastRpmCalcPulseCount;

    float calculatedRPM = 0.0;
    // Check for valid time interval and pulse change
    if (deltaTime_us > 0 && deltaPulses > 0) {
        float pulses_per_second = (float)deltaPulses * 1000000.0 / (float)deltaTime_us;
        float rps = pulses_per_second / (float)PULSES_PER_REV;
        calculatedRPM = rps * 60.0;
    } else if (deltaTime_us > RPM_CALC_INTERVAL_US * 2) {
        // If no pulses for a while, check if we are supposed to be stopped
        // Accessing RPM_setpoint here is technically accessing non-volatile from ISR,
        // but might be okay if updates are infrequent or atomic. Safer to use a flag.
         float setpointSnapshot = RPM_setpoint; // Read setpoint (might be slightly outdated)
        if (abs(setpointSnapshot) < 0.1) {
            calculatedRPM = 0.0; // Assume stopped if setpoint is zero
        }
        // Otherwise, keep last calculated RPM if setpoint is non-zero but no pulses? Or set to 0?
        // Let's assume 0 if no pulses for 2*interval, regardless of setpoint.
        calculatedRPM = 0.0;
    } else {
      // Not enough time passed or no pulses, keep previous value (do nothing to calculatedRPM)
      // Or should we return the globally stored currentRPM?
      // Let's update global currentRPM only when calculation is valid
      calculatedRPM = currentRPM; // Keep last valid value if conditions not met this cycle
    }

    // Safely update volatile global variable
    noInterrupts();
    currentRPM = calculatedRPM;
    interrupts();

    // Update state for next calculation
    lastRpmCalcTime = currentTime_us;
    lastRpmCalcPulseCount = currentPulseReading;
}

// =========================================================================
// TIMER CALLBACK - Stop Detection
// Checks if pulse count has changed; if not, sets RPM to 0.
// =========================================================================
void IRAM_ATTR stop_timer_callback(void *arg) {
    unsigned long currentPulseReading;
    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();

    // If motor was supposed to be running but pulse count hasn't changed for interval
    if (motorRunning && currentPulseReading == lastCheckedPulseCount) {
        noInterrupts();
        currentRPM = 0.0; // Force RPM to 0 if stalled
        interrupts();
    }
    lastCheckedPulseCount = currentPulseReading; // Update for next check
}

// =========================================================================
// FUNCTION: readHB100Velocity (Example using polling/edge detection)
// Reads relative velocity from HB100 sensor.
// NOTE: Accuracy depends heavily on signal conditioning and frequency measurement.
// Consider using ESP32 Pulse Counter peripheral for better frequency reading.
// =========================================================================
void readHB100Velocity() {
    bool currentState = digitalRead(HB100_PIN);
    // Detect rising edge (or falling, depending on signal)
    if (currentState == HIGH && lastHB100State == LOW) {
        hb100PulseCount++;
    }
    lastHB100State = currentState;

    unsigned long now = millis();
    // Calculate frequency and velocity periodically (e.g., every 200ms)
    if (now - hb100LastTime >= 200) {
        float deltaT_s = (float)(now - hb100LastTime) / 1000.0;
        if (deltaT_s > 0) {
            float freq = (float)hb100PulseCount / deltaT_s;
            // Doppler formula: V = (Freq * Wavelength) / 2
            // Ensure wavelength is correct for your sensor's frequency (e.g., 10.525 GHz)
            hb100Velocity = (freq * HB100_WAVELENGTH) / 2.0; // Velocity in m/s
        } else {
            hb100Velocity = 0.0;
        }
        hb100PulseCount = 0; // Reset pulse count for next interval
        hb100LastTime = now;
    }
}


// =========================================================================
// FUNCTION: configureTofForI2C (NEW)
// Sets the TOF10120 sensor to passive I2C reading mode. Call ONCE in setup().
// =========================================================================
bool configureTofForI2C() {
    Wire.beginTransmission(TOF_I2C_ADDRESS);
    Wire.write(0x09); // Register address for "Distance Sending Mode"
    Wire.write(0x01); // Value '1' for "Host reads passively (UART, I2C)"
    byte error = Wire.endTransmission();
    if (error == 0) {
        Serial.println("OK: TOF10120 configured for I2C passive read mode.");
        delay(10); // Short delay after configuration might be good
        return true;
    } else {
        Serial.print("ERROR: Configuring TOF10120 for I2C mode failed. Code: ");
        Serial.println(error);
        return false;
    }
}

// =========================================================================
// FUNCTION: readDistance (UPDATED)
// Reads distance from TOF10120 via I2C using datasheet parameters.
// Returns distance in mm or UINT16_MAX on error/invalid reading.
// =========================================================================
uint16_t readDistance() {
    Wire.beginTransmission(TOF_I2C_ADDRESS);
    Wire.write(0x00); // Register address for real-time distance LSB (0x00-0x01)
    byte error = Wire.endTransmission();
    if (error != 0) {
         // Optional: Print error occasionally to avoid serial flooding
         // static uint32_t lastErrorPrintWr = 0;
         // if (millis() - lastErrorPrintWr > 2000) { Serial.print("I2C WrErr:"); Serial.println(error); lastErrorPrintWr = millis(); }
         return UINT16_MAX;
    }

    // Datasheet: Wait at least 30us after sending register addr before reading
    delayMicroseconds(50); // 50us should be safe

    uint8_t bytesReceived = Wire.requestFrom(TOF_I2C_ADDRESS, (uint8_t)2); // Request 2 bytes
    if (bytesReceived < 2) { // Check if we actually got 2 bytes
         // Optional: Print error occasionally
         // static uint32_t lastErrorPrintRd = 0;
         // if (millis() - lastErrorPrintRd > 2000) { Serial.println("I2C RdErr: <2 bytes"); lastErrorPrintRd = millis(); }
         return UINT16_MAX;
    }

    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    uint16_t rawValue = (highByte << 8) | lowByte;

    // Filter based on datasheet valid range (100mm - 1800mm)
    if (rawValue < MIN_VALID_DISTANCE_TOF || rawValue > MAX_VALID_DISTANCE_TOF) {
        // Reading is outside the valid operational range. Treat as invalid.
        // Optional: Print invalid readings occasionally
        // static uint32_t lastInvalidPrint = 0;
        // if (millis() - lastInvalidPrint > 2000) { Serial.print("TOF Invalid Raw: "); Serial.println(rawValue); lastInvalidPrint = millis(); }
        return UINT16_MAX;
    }

    // If we reach here, the reading is within the valid 100-1800mm range
    return rawValue;
}


// =========================================================================
// FUNCTION: applyMovingAverage
// Applies a simple moving average filter to the distance readings.
// =========================================================================
uint16_t applyMovingAverage(uint16_t newValue) {
    distanceBuffer[bufferIndex] = newValue; // Store new value (could be UINT16_MAX)
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE; // Cycle buffer index

    uint32_t sum = 0;
    uint16_t validCount = 0;
    for (uint16_t i = 0; i < FILTER_SIZE; i++) {
        // Average only valid readings in the buffer
        if (distanceBuffer[i] != UINT16_MAX) {
            sum += distanceBuffer[i];
            validCount++;
        }
    }

    // Return average if valid readings exist, otherwise return invalid marker
    return (validCount == 0) ? UINT16_MAX : (uint16_t)(sum / validCount);
}


// =========================================================================
// FUNCTION: kalmanFilter
// Applies a simple Kalman filter to the distance readings.
// =========================================================================
float kalmanFilter(float measurement) {
    // Prediction step
    float x_pred = x_est;         // Predicted state is same as previous estimate (assuming constant position)
    float P_pred = P_est + Q;     // Predict error covariance

    // Update step
    K = P_pred / (P_pred + R);                 // Calculate Kalman gain
    x_est = x_pred + K * (measurement - x_pred); // Update state estimate
    P_est = (1 - K) * P_pred;                  // Update error covariance

    // Optional: Constrain output to plausible range
    x_est = constrain(x_est, 0, MAX_VALID_DISTANCE_TOF + 200); // Allow some margin

    return x_est;
}

// =========================================================================
// FUNCTION: setMotorPWM
// Applies the given PWM value (0-255) to the motor driver, includes current check.
// =========================================================================
void setMotorPWM(int pwmValue) {
    pwmValue = constrain(pwmValue, 0, 255);
    currentPWM = pwmValue; // Store the value intended to be applied

    // --- Optional Current Check ---
    // uint16_t raw_IS_R = analogRead(IS_R);
    // float voltage_IS_R_mV = raw_IS_R * (3300.0 / 4095.0); // Assuming 3.3V ADC ref
    // if (voltage_IS_R_mV > CURRENT_LIMIT_MV ) {
    //    Serial.println("!!! CURRENT LIMIT EXCEEDED !!!");
    //    digitalWrite(R_EN, LOW); // Disable driver
    //    digitalWrite(L_EN, LOW);
    //    analogWrite(LPWM, 0);    // Set PWM to 0
    //    digitalWrite(RPWM, LOW); // Ensure forward pin is low
    //    motorRunning = false;
    //    integral = 0; // Reset PID integral term on fault
    //    currentPWM = 0; // Reflect that 0 PWM is actually applied
    //    return; // Exit function early
    // }
    // --- End Optional Current Check ---


    // Apply PWM if current is okay (or if check is disabled)
    digitalWrite(R_EN, HIGH); // Enable driver
    digitalWrite(L_EN, HIGH);
    analogWrite(LPWM, pwmValue); // Set PWM value for forward direction (assuming LPWM controls speed)
    digitalWrite(RPWM, LOW);    // Ensure reverse PWM pin is low
    motorRunning = (pwmValue > 5); // Consider motor running if PWM is non-negligible
}


// =========================================================================
// FUNCTION: updateEMA
// Applies Exponential Moving Average filter.
// =========================================================================
float updateEMA(float currentRawValue) {
    static float previousEMA = 0.0;
    static bool initialized = false;

    if (!initialized) {
        // Initialize with the first non-zero value to avoid starting at 0 if motor is already moving
        if (abs(currentRawValue) > 0.01) {
             previousEMA = currentRawValue;
             initialized = true;
        } else {
             return 0.0; // Return 0 if still initializing with 0
        }
    }

    float currentEMA = (EMA_ALPHA * currentRawValue) + ((1.0 - EMA_ALPHA) * previousEMA);
    previousEMA = currentEMA;
    return currentEMA;
}

// =========================================================================
// FUNCTION: calculatePID_RPM_Output
// Calculates the required control effort (scaled in RPM units).
// =========================================================================
float calculatePID_RPM_Output(float setpointRPM, float measuredRPM) {
    float errorRPM = setpointRPM - measuredRPM;

    // --- Integral Term (with anti-windup clamping) ---
    // Estimate max contribution needed from integral (e.g., enough to overcome friction/reach max PWM)
    // This needs tuning - related to max PWM and RPM_TO_PWM params
    float maxIntegralContribution_RPM = MAX_RPM_ESTIMATE * 0.5; // Example limit - TUNE ME!
    float maxIntegral = (abs(Ki) > 0.001) ? maxIntegralContribution_RPM / Ki : 1e9; // Avoid division by zero

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
    previousErrorRPM = errorRPM;

    // Optional: Clamp total PID output to prevent excessive demands?
    // pidOutput_RPM = constrain(pidOutput_RPM, -MAX_RPM_ESTIMATE, MAX_RPM_ESTIMATE);

    return pidOutput_RPM;
}

// =========================================================================
// FUNCTION: transformRPMtoPWM
// Converts the target RPM effort from PID into a PWM value (0-255).
// =========================================================================
int transformRPMtoPWM(float targetEffort_RPM) {
    // Apply linear transformation: PWM = m * RPM + c
    float calculatedPWM_f = RPM_TO_PWM_SLOPE * targetEffort_RPM + RPM_TO_PWM_OFFSET;

    // Constrain the result to the valid PWM range
    int pwmOut = constrain((int)round(calculatedPWM_f), 0, 255);

    // If target RPM is positive but PWM calculates to 0 due to offset/slope,
    // maybe apply a minimum PWM if we want it to definitely move? Optional.
    // if (targetEffort_RPM > 0.1 && pwmOut < 5) {
    //     pwmOut = 5; // Example minimum PWM
    // }

    return pwmOut;
}


// =========================================================================
// FUNCTION: updateRPM_IDM
// Calculates target RPM based on Intelligent Driver Model.
// Updates the global RPM_setpoint variable.
// =========================================================================
void updateRPM_IDM(float current_speed_mps, float relative_speed_mps, float distance_mm) {
    // Ensure distance is within plausible bounds for calculation (prevent division by zero/sqrt issues)
    // Use the raw valid distance for s? Or filtered? Using filteredDistance here.
    float s = max((float)MIN_VALID_DISTANCE_TOF, (float)distance_mm); // Use distance >= 100mm
    s = min(s, (float)MAX_VALID_DISTANCE_TOF); // Cap at max range

    float v = max(0.0f, current_speed_mps); // Current speed (m/s), ensure non-negative
    float v0_mps = (desired_velocity / 60.0) * (PI * 0.1); // Target speed (m/s) from RPi RPM (assuming 0.1m wheel diam) - NEEDS CORRECT DIAMETER
    float dv = relative_speed_mps; // Relative speed (m/s), (-) means closing gap

    // IDM formula component: desired gap s*
    float s_star_interaction = (v * dv) / (2.0 * sqrt(a_max * b_comfort));
    float s_star = s0 + max(0.0f, (v * T_gap + s_star_interaction)); // Ensure interaction term doesn't make s* negative

    // Clamp v/v0 to prevent issues with pow(negative, non-integer) if v0 is 0
    float v_ratio = (v0_mps > 0.01) ? v / v0_mps : 0.0;
    float free_road_term = pow(v_ratio, 4); // Delta exponent, often 4

    // Clamp s_star/s to prevent issues if s is near zero (though we ensure s >= 100)
    float interaction_term = pow(s_star / s, 2);

    // IDM acceleration calculation
    float acceleration = a_max * (1.0 - free_road_term - interaction_term);

    // Calculate next velocity based on calculated acceleration
    float v_next_mps = v + acceleration * PID_SAMPLE_TIME_S; // Use PID sample time
    v_next_mps = max(0.0f, v_next_mps); // Ensure velocity doesn't go negative

    // Convert next velocity back to target RPM
    // Needs correct wheel diameter! Using 0.1m placeholder.
    const float WHEEL_DIAMETER_M_IDM = 0.1; // EXAMPLE - SET YOUR ACTUAL VALUE
    const float WHEEL_CIRCUMFERENCE_M_IDM = PI * WHEEL_DIAMETER_M_IDM;
    float rpm_next = (v_next_mps / WHEEL_CIRCUMFERENCE_M_IDM) * 60.0;

    // Set the global RPM_setpoint, constrained by 0 and the desired velocity from RPi
    RPM_setpoint = constrain(rpm_next, 0.0, desired_velocity);
}


// =========================================================================
// FUNCTION: control_servo
// Sends pulse to servo based on global steering_angle.
// =========================================================================
void control_servo() {
    static unsigned long lastServoUpdate = 0;
    unsigned long now = micros(); // Use micros for better timing precision

    if (now - lastServoUpdate >= REFRESH_INTERVAL) {
        lastServoUpdate = now;
        // Constrain angle based on software limits
        int constrainedAngle = constrain(steering_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        // Map angle (0-180 typical range) to pulse width
        int pulseWidth = map(constrainedAngle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(pulseWidth); // Send pulse
        digitalWrite(SERVO_PIN, LOW);
    }
}

// =========================================================================
// FUNCTION: parseSerialData
// Parses the fixed-length serial packet '<angle,speed>'
// =========================================================================
void parseSerialData() {
    // Check start and end markers
    if (serialBuffer[0] == '<' && serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') {
        // Extract steering angle (4 bytes, offset 1)
        memcpy(&steering_angle, &serialBuffer[1], sizeof(int32_t));
        // Extract speed (4 bytes, offset 5)
        memcpy(&received_speed, &serialBuffer[5], sizeof(int32_t));

        // Update global variables used by control loops
        // Convert received speed to float RPM for internal use
        rpm_from_raspberry = (float)received_speed;
        // Update the desired_velocity ceiling for IDM
        desired_velocity = rpm_from_raspberry;

        // Optional: Constrain received angle/speed if needed
        steering_angle = constrain(steering_angle, 0, 180); // Example constraint

    } else {
        Serial.println("Invalid serial packet received.");
        // Should potentially clear buffer or ignore if invalid?
    }
    serialBufferIndex = 0; // Reset buffer index for next packet
}


// =========================================================================
// FUNCTION: displayData (Revised)
// Prints key system variables to Serial Plotter or Monitor.
// =========================================================================
void displayData() {
    static unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL_MS = 250; // Print interval (ms)

    // Variable to hold raw distance for this print cycle
    uint16_t rawDistForDisplay = UINT16_MAX; // Default to invalid
     // Find the most recent raw reading from the buffer? Or use a global?
     // Let's search buffer for the last non-UINT16_MAX value for demo
     int searchIndex = (bufferIndex == 0) ? (FILTER_SIZE - 1) : (bufferIndex - 1);
     for (int i = 0; i < FILTER_SIZE; ++i) {
         int currentIndex = (searchIndex - i + FILTER_SIZE) % FILTER_SIZE;
         if (distanceBuffer[currentIndex] != UINT16_MAX) {
             rawDistForDisplay = distanceBuffer[currentIndex];
             break;
         }
     }


    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();

        Serial.print("TGT_RPM:"); // Final target sent to PID
        Serial.print(previousSetpoint, 1); // Use previousSetpoint as it holds the final target
        Serial.print(", RPI_RPM:"); // Commanded by RPi
        Serial.print(rpm_from_raspberry, 1);
        Serial.print(", ACT_RPM:"); // EMA filtered RPM
        Serial.print(g_filteredRPM, 1);
        Serial.print(", PWM:");
        Serial.print(currentPWM); // Actual PWM applied
        Serial.print(", TOF_Filt:"); // Final Filtered Distance
        if (filteredDistance == UINT16_MAX) {
            Serial.print("INV"); // Invalid
        } else {
            Serial.print(filteredDistance);
        }
        Serial.print(", TOF_Raw:"); // Last known Raw reading
        if (rawDistForDisplay == UINT16_MAX) {
            Serial.print("INV");
        } else {
            Serial.print(rawDistForDisplay);
        }
        Serial.print(", IDM:"); // IDM Active?
        Serial.print(override_by_IDM ? "Y" : "N");
        Serial.print(", HB100:"); // Doppler Velocity
        Serial.print(hb100Velocity, 2);
        Serial.print(", Angle:"); // Steering Angle
        Serial.print(steering_angle);
        Serial.println(); // Newline
    }
}


// =========================================================================
// SETUP FUNCTION (Revised)
// Initializes hardware, sensors, variables, timers, and interrupts.
// =========================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 2000); // Wait max 2s for Serial connection (optional)
    Serial.println("===== ESP32 Robot Control Initializing =====");
    Serial.println("System Time: " + String(millis())); // Simple check

    // --- I2C Initialization ---
    Wire.begin(); // Initialize I2C *BEFORE* sensor interaction
    Wire.setClock(400000); // Set I2C speed (400kHz usually works)
    Serial.println("I2C Initialized.");

    // --- Configure TOF Sensor ---
    Serial.println("Configuring TOF10120 sensor for I2C Mode...");
    if (!configureTofForI2C()) {
        Serial.println("CRITICAL WARNING: TOF Sensor config FAILED. Distance readings UNUSABLE.");
        // Consider halting or entering a safe error state
        // while(1) { delay(1000); Serial.println("TOF Config Failed - Halting"); }
    }

    // --- Initialize Buffers & State Variables ---
    Serial.println("Initializing buffers and state variables...");
    for (int i = 0; i < FILTER_SIZE; i++) {
        distanceBuffer[i] = UINT16_MAX; // Initialize buffer as invalid
    }
     bufferIndex = 0;
     invalid_count = 0;
     filteredDistance = UINT16_MAX; // Start as invalid
     tof_ready = false; // Sensor not confirmed ready yet
     // Initialize Kalman filter estimate (e.g., max range or 0?)
     x_est = MAX_VALID_DISTANCE_TOF; // Start assuming path is clear?
     P_est = 100; // Start with higher uncertainty
     K = 0;

    // --- Pin Modes ---
    Serial.println("Setting Pin Modes...");
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW); // Ensure servo pin is low initially
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(IS_R, INPUT); // Current sense pins as input
    pinMode(IS_L, INPUT);
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP); // Use internal pullup for Hall sensor
    pinMode(HB100_PIN, INPUT); // HB100 input

    // --- Initial Motor State ---
    Serial.println("Setting Initial Motor State (Disabled)...");
    digitalWrite(R_EN, LOW); // Ensure drivers are disabled initially
    digitalWrite(L_EN, LOW);
    analogWrite(LPWM, 0); // Set PWM to 0
    digitalWrite(RPWM, LOW); // Ensure reverse pin is low
    motorRunning = false;
    currentPWM = 0;

    // --- Attach Interrupt ---
    Serial.println("Attaching Hall Sensor Interrupt...");
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);

    // --- Initialize Timers & State ---
    Serial.println("Initializing Timers and Control Variables...");
    lastRpmCalcTime = micros();
    lastPIDRunTime = millis();
    hb100LastTime = millis();

    noInterrupts();
    lastRpmCalcPulseCount = pulseCount; // Initialize pulse counts safely
    lastCheckedPulseCount = pulseCount;
    interrupts();

    // Initialize control variables AFTER sensor setup attempt
    RPM_setpoint = 0.0; // Start with 0 target RPM
    rpm_from_raspberry = 0.0;
    desired_velocity = 0.0;
    previousSetpoint = 0.0;
    rpm_from_rasp_filtered = 0.0;
    g_filteredRPM = 0.0;
    integral = 0.0;
    previousErrorRPM = 0.0;

    // --- Setup ESP32 Timers ---
    Serial.println("Starting ESP32 Timers for RPM Calc and Stop Detect...");
    esp_timer_create_args_t rpm_timer_args = { .callback = &rpm_timer_callback, .name = "rpm_calc"};
    esp_timer_create(&rpm_timer_args, &rpm_timer_handle);
    esp_timer_start_periodic(rpm_timer_handle, RPM_CALC_INTERVAL_US);

    esp_timer_create_args_t stop_timer_args = { .callback = &stop_timer_callback, .name = "motor_stop_check"};
    esp_timer_create(&stop_timer_args, &stop_timer_handle);
    esp_timer_start_periodic(stop_timer_handle, STOP_CHECK_INTERVAL_US);

    Serial.println("===== Setup Complete. Entering Main Loop. =====");
}

// =========================================================================
// MAIN LOOP
// Handles serial input, sensor reading, IDM logic, PID control, servo control.
// =========================================================================
void loop() {

    // --- 1. Process Incoming Serial Data ---
     while (Serial.available() >= SERIAL_BUFFER_SIZE) { // Process only if a full packet might be there
         if (Serial.peek() == '<') { // Check for start marker before reading
             // Read potential packet into buffer
             Serial.readBytes(serialBuffer, SERIAL_BUFFER_SIZE);
             if (serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') {
                 parseSerialData(); // Parse if start and end markers match
             } else {
                 // Invalid frame, discard start byte and let loop continue
                 Serial.read(); // Consume the '<' that didn't lead to a valid packet
                 serialBufferIndex = 0; // Reset index
             }
         } else {
             // Discard bytes that are not a start marker
             Serial.read();
             serialBufferIndex = 0; // Reset index
         }
     }

    // --- 2. Read Sensors ---
    readHB100Velocity(); // Update hb100Velocity

    // Get latest raw RPM (updated by timer callback)
    float rawRPM;
    noInterrupts();
    rawRPM = currentRPM;
    interrupts();
    // Apply EMA filter to get smoother RPM for control/display
    g_filteredRPM = updateEMA(rawRPM);

    // Calculate current speed in m/s for IDM
    // !!! SET YOUR ACTUAL WHEEL DIAMETER HERE !!!
    const float WHEEL_DIAMETER_M = 0.10; // EXAMPLE: 10cm wheel diameter
    // !!!-------------------------------------!!!
    const float WHEEL_CIRCUMFERENCE_M = PI * WHEEL_DIAMETER_M;
    float current_speed_mps = (g_filteredRPM / 60.0) * WHEEL_CIRCUMFERENCE_M;

    // --- Read & Filter Distance Sensor ---
    static uint32_t lastToFMeasureTime = 0;
    uint16_t currentRawDistance = UINT16_MAX; // Store this cycle's raw value for display

    // Sample ToF periodically (e.g., every 30-50ms)
    if (millis() - lastToFMeasureTime >= 40) {
        lastToFMeasureTime = millis();
        currentRawDistance = readDistance(); // Read sensor (returns UINT16_MAX on error/invalid)

        if (currentRawDistance != UINT16_MAX) { // Got a VALID reading (100-1800mm)
            invalid_count = 0; // Reset error counter
            uint16_t avgDistance = applyMovingAverage(currentRawDistance); // Apply moving average

            if (avgDistance != UINT16_MAX) { // Check if average is valid
                 // Apply Kalman filter only if average is valid
                 filteredDistance = (uint16_t)kalmanFilter((float)avgDistance);
                 // Clamp filtered distance to ensure it stays within valid sensor range after filtering
                 filteredDistance = constrain(filteredDistance, MIN_VALID_DISTANCE_TOF, MAX_VALID_DISTANCE_TOF);
                 tof_ready = true; // Sensor is providing valid filtered data
            } else {
                 // Moving average buffer might still be full of invalids
                 filteredDistance = UINT16_MAX; // Keep filtered as invalid
                 tof_ready = false;
            }
        } else { // Got UINT16_MAX (I2C error OR reading outside 100-1800mm range)
            // Insert the invalid reading marker into the buffer
            applyMovingAverage(UINT16_MAX);
            if (++invalid_count >= MAX_INVALID_ALLOWED) {
                // Sensor failing consistently
                filteredDistance = UINT16_MAX; // Ensure filtered is invalid
                tof_ready = false; // Mark ToF as not ready
                // Optional: Reset Kalman filter state if sensor fails for too long
                // x_est = MAX_VALID_DISTANCE_TOF; P_est = 100;
            }
            // Note: If invalid_count < MAX_INVALID_ALLOWED, filteredDistance retains its last valid value
        }
    } // End ToF reading block


    // --- 3. Decision Logic (IDM / RPi Control) ---
    // Determine if IDM should override the RPi command based on ToF data

    if (!tof_ready || filteredDistance == UINT16_MAX) {
        // Case 1: ToF Invalid / Not Ready / Consistently Failing
        override_by_IDM = false; // Use RPi command
    } else if (filteredDistance < IDM_STOP_DISTANCE) {
        // Case 2: Object VERY close -> Emergency Stop
        override_by_IDM = true;
        RPM_setpoint = 0.0; // Target 0 RPM immediately
        setMotorPWM(0);     // Force motor PWM to 0 NOW
        integral = 0.0;     // Reset PID integral
        previousErrorRPM = 0.0; // Reset PID derivative memory
    } else if (filteredDistance <= IDM_CONTROL_MAX_DIST) {
        // Case 3: Object in IDM control range -> Calculate IDM speed
        override_by_IDM = true;
        // Prepare inputs for IDM function
        // Ensure hb100Velocity is correct relative speed in m/s. Check sign convention.
        // IDM expects dv = v_leader - v_follower. If HB100 measures closing speed as positive, dv might be -hb100Velocity.
        float relative_speed_mps = -hb100Velocity; // EXAMPLE: Assuming positive HB100 means closing speed
        updateRPM_IDM(current_speed_mps, relative_speed_mps, filteredDistance); // This updates global RPM_setpoint
    } else {
        // Case 4: Path Clear (Object beyond IDM_CONTROL_MAX_DIST)
        override_by_IDM = false; // Use RPi command
    }


    // --- 4. Determine Final RPM Setpoint ---
    float final_rpm_target;
    if (override_by_IDM) {
        // Use the RPM setpoint calculated by IDM (or 0 if stopping)
        final_rpm_target = RPM_setpoint;
    } else {
        // Not overridden: Smoothly transition towards the RPi commanded speed
        // Apply EMA filter to the RPi command itself for smoother changes
        rpm_from_rasp_filtered = (RPM_SMOOTH_ALPHA * rpm_from_raspberry) + ((1.0 - RPM_SMOOTH_ALPHA) * rpm_from_rasp_filtered);
        // Limit the rate of change from the previous setpoint
        float delta_rpm = rpm_from_rasp_filtered - previousSetpoint;
        delta_rpm = constrain(delta_rpm, -MAX_RPM_DELTA, MAX_RPM_DELTA);
        final_rpm_target = previousSetpoint + delta_rpm;
    }
    // Ensure target is non-negative if motor only goes forward
    final_rpm_target = max(0.0f, final_rpm_target);

    // Update the setpoint memory for next cycle's smoothing calculation
    previousSetpoint = final_rpm_target;


    // --- 5. PID Control Loop ---
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDRunTime >= PID_INTERVAL_MS) {
        lastPIDRunTime = currentMillis;

        // Check if target is effectively zero OR if IDM forced an immediate stop
        if (abs(final_rpm_target) < 0.1 || (override_by_IDM && RPM_setpoint == 0.0 && filteredDistance < IDM_STOP_DISTANCE) ) {
            integral = 0.0;         // Reset integral
            previousErrorRPM = 0.0; // Reset derivative term basis
            if (!(override_by_IDM && filteredDistance < IDM_STOP_DISTANCE)) {
                 // Set PWM to 0 only if IDM didn't already force it
                 setMotorPWM(0);
            }
        } else {
            // Target is non-zero, calculate and apply PID output
            // g_filteredRPM (actual speed) was updated earlier
            float targetEffort_RPM = calculatePID_RPM_Output(final_rpm_target, g_filteredRPM);
            int targetPWM = transformRPMtoPWM(targetEffort_RPM);
            setMotorPWM(targetPWM); // Apply calculated PWM
        }
    } // --- End of PID Control Loop ---


    // --- 6. Servo Control ---
    control_servo(); // Update steering angle


    // --- 7. Display Data ---
    // Pass the raw distance read in this loop cycle for accurate display
    displayData(); // Update Serial Monitor/Plotter

} // --- End of Main Loop ---

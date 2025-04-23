#include <Arduino.h>
#include <math.h>
#include "esp_timer.h"
#include <Wire.h>

// --- Pin Definitions ---
// ... (keep others) ...

// --- TOF10120 Constants --- (UPDATED)
// const uint8_t TOF_I2C_ADDRESS = 0x52; // OLD - Incorrect
const uint8_t TOF_I2C_ADDRESS = 0xA4; // NEW - Datasheet default (164 decimal)

const uint8_t FILTER_SIZE = 5;
const uint16_t MIN_VALID_DISTANCE_TOF = 100; // NEW - Datasheet minimum range
const uint16_t MAX_VALID_DISTANCE_TOF = 1800; // NEW - Datasheet maximum range

uint16_t distanceBuffer[FILTER_SIZE];
uint8_t bufferIndex = 0;
const float Q = 0.1; // Kalman Q
const float R = 2.0; // Kalman R
// ... (keep Kalman state variables: x_est, P_est, K) ...
int invalid_count = 0;
const int MAX_INVALID_ALLOWED = 5; // Allow a few more errors?
//================================
float x_est = 0; // Initialize Kalman estimate
float P_est = 1; // Initialize Kalman error covariance
float K = 0;     // Initialize Kalman gain
uint16_t filteredDistance = UINT16_MAX; // Initialize as invalid
bool tof_ready = false; // Start as not ready

// --- IDM global scope --- (Thresholds NEED review based on 100mm min)
float desired_velocity = 0.0; // target RPM when road is clear (Set by RPi)
float a_max = 1.0;            // max acceleration [m/s^2] - TUNE
float b_comfort = 1.5;        // comfortable deceleration [m/s^2] - TUNE
float T_gap = 1.0;            // safe time headway [s] - TUNE
float s0 = 150;               // minimum desired distance [mm] - MUST BE >= 100 (e.g., 150mm)
float safe_max_distance = 1500; // Upper cap for distance sensor (e.g., 1500mm)
float rpm_from_raspberry = 0.0;
bool override_by_IDM = false;

// ... (Rest of your global variables: PID, EMA, Servo, HB100 etc.) ...
float previousSetpoint = 0;
float rpm_from_rasp_filtered = 0;
const float MAX_RPM_DELTA = 10.0;
const float RPM_SMOOTH_ALPHA = 0.05;
float g_filteredRPM = 0.0;
float integral = 0.0;
float previousErrorRPM = 0.0;
int currentPWM = 0;
// ...

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
// Reads distance from TOF10120 via I2C.
// =========================================================================
uint16_t readDistance() {
    Wire.beginTransmission(TOF_I2C_ADDRESS);
    Wire.write(0x00); // Register address for real-time distance LSB (0x00-0x01)
    byte error = Wire.endTransmission();
    if (error != 0) {
         // Optional: Print error only occasionally to avoid spamming
         // static uint32_t lastErrorPrint = 0;
         // if (millis() - lastErrorPrint > 1000) {
         //    Serial.print("I2C Error during AddrWr: "); Serial.println(error);
         //    lastErrorPrint = millis();
         // }
         return UINT16_MAX; // Use error code
    }

    // Datasheet: Wait at least 30us after sending register addr before reading
    delayMicroseconds(50); // 50us should be safe

    Wire.requestFrom(TOF_I2C_ADDRESS, (uint8_t)2); // Request 2 bytes
    if (Wire.available() < 2) {
         // Optional: Print error only occasionally
         // static uint32_t lastErrorPrintRd = 0;
         // if (millis() - lastErrorPrintRd > 1000) {
         //    Serial.println("I2C Error: No/Few bytes received.");
         //    lastErrorPrintRd = millis();
         // }
         return UINT16_MAX; // Use error code
    }

    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    uint16_t rawValue = (highByte << 8) | lowByte;

    // Filter based on datasheet valid range (100mm - 1800mm)
    if (rawValue < MIN_VALID_DISTANCE_TOF || rawValue > MAX_VALID_DISTANCE_TOF) {
        // Reading is outside the valid operational range of the sensor. Treat as invalid.
        // If you constantly get low values (e.g., < 100), print them here sometimes to debug.
        // static uint32_t lastInvalidPrint = 0;
        // if (millis() - lastInvalidPrint > 1000) {
        //    Serial.print("TOF Raw Invalid ("); Serial.print(MIN_VALID_DISTANCE_TOF); Serial.print("-"); Serial.print(MAX_VALID_DISTANCE_TOF); Serial.print("): "); Serial.println(rawValue);
        //    lastInvalidPrint = millis();
        // }
        return UINT16_MAX;
    }

    // If we reach here, the reading is within the valid 100-1800mm range
    return rawValue;
}

// =========================================================================
// FUNCTION: applyMovingAverage (Keep as is, ensure buffer init)
// =========================================================================
uint16_t applyMovingAverage(uint16_t newValue) {
    distanceBuffer[bufferIndex] = newValue;
    bufferIndex = (bufferIndex + 1) % FILTER_SIZE;
    uint32_t sum = 0;
    uint16_t validCount = 0;
    for (uint16_t i = 0; i < FILTER_SIZE; i++) {
        // Check against UINT16_MAX which is now the only invalid marker
        if (distanceBuffer[i] != UINT16_MAX) {
            sum += distanceBuffer[i];
            validCount++;
        }
    }
    // If no valid readings in buffer, return invalid; otherwise return average
    return (validCount == 0) ? UINT16_MAX : (uint16_t)(sum / validCount);
}


// =========================================================================
// FUNCTION: kalmanFilter (Keep as is)
// =========================================================================
float kalmanFilter(float measurement) {
    float x_pred = x_est;
    float P_pred = P_est + Q;
    K = P_pred / (P_pred + R);
    x_est = x_pred + K * (measurement - x_pred);
    P_est = (1 - K) * P_pred;
    // Constrain output to plausible range? Optional. E.g., 0 to MAX_RANGE+margin
    // x_est = constrain(x_est, 0, MAX_VALID_DISTANCE_TOF + 100);
    return x_est;
}


// =========================================================================
// SETUP FUNCTION (UPDATED)
// =========================================================================
void setup() {
    Serial.begin(115200);
    while (!Serial); // Wait for Serial connection (optional)
    Serial.println("ESP32 Motor Control Initializing (Datasheet Corrected)...");

    Wire.begin(); // Initialize I2C *BEFORE* trying to configure sensor
    Wire.setClock(400000); // Set I2C speed (400kHz is often okay)

    Serial.println("Configuring TOF10120 sensor for I2C Mode...");
    if (!configureTofForI2C()) {
        Serial.println("WARNING: TOF Sensor configuration FAILED. Distance readings unusable.");
        // Consider halting or entering a safe mode if ToF is critical
        // while(1); // Example: Halt execution
    }

    // Initialize distance buffer AFTER Wire.begin()
    for (int i = 0; i < FILTER_SIZE; i++) {
        distanceBuffer[i] = UINT16_MAX; // Initialize buffer as invalid
    }
     bufferIndex = 0; // Reset buffer index
     invalid_count = 0; // Reset invalid counter
     filteredDistance = UINT16_MAX; // Start as invalid
     tof_ready = false; // Start as not ready
     x_est = MAX_VALID_DISTANCE_TOF; // Initialize Kalman estimate to max range? Or mid-range?
     P_est = 100; // Start with higher uncertainty?

    // --- Pin Modes --- (Keep setup)
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);
    // ... other pinModes ...
     pinMode(HB100_PIN, INPUT); // Make sure HB100 pin is INPUT

    // --- Initial State --- (Keep setup)
    // ... motor driver init ...
    setMotorPWM(0);

    // --- Attach Interrupt --- (Keep setup)
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);

    // --- Initialize Timers & State --- (Keep setup)
    // ... timer setups, variable initializations ...
    // IMPORTANT: Initialize PID/Speed variables AFTER sensor setup attempt
    RPM_setpoint = 0; // Start at 0 RPM
    rpm_from_raspberry = 0;
    desired_velocity = 0;
    previousSetpoint = 0;
    rpm_from_rasp_filtered = 0;
    g_filteredRPM = 0;
    integral = 0;
    previousErrorRPM = 0;
    currentPWM = 0;

    Serial.println("Setup Complete. Waiting for data...");
}

// =========================================================================
// MAIN LOOP (IDM Logic Needs Revision)
// =========================================================================
void loop() {
    // --- Process Incoming Serial Data --- (Keep as is)
     while (Serial.available() > 0) {
       // ... (Keep existing serial parsing logic) ...
       // Ensure parseSerialData updates rpm_from_raspberry
       // And maybe desired_velocity ONLY when override_by_IDM is false?
       uint8_t byteRead = Serial.read();
        if (serialBufferIndex == 0 && byteRead != '<') {
            continue; // Wait for start marker
        }
        if (serialBufferIndex < SERIAL_BUFFER_SIZE) {
            serialBuffer[serialBufferIndex++] = byteRead;
        }
        if (serialBufferIndex == SERIAL_BUFFER_SIZE) {
            if (serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') {
                parseSerialData(); // Updates steering_angle, received_speed
                // Update rpm_from_raspberry based on received_speed
                rpm_from_raspberry = (float)received_speed;
                 // Update desired_velocity for IDM *only if not overridden*? Or always sync?
                 // Let's always sync desired_velocity for now, IDM uses it as a ceiling.
                 desired_velocity = rpm_from_raspberry;
            } else {
                 Serial.println("Serial Frame Error: No end marker.");
                 // Consider clearing buffer even on error?
            }
            serialBufferIndex = 0; // Always reset index
        }
     }


    // --- Read HB100 ---
    readHB100Velocity(); // Update hb100Velocity

    // --- Read Current RPM ---
    float rawRPM;
    noInterrupts();
    rawRPM = currentRPM;
    interrupts();
    g_filteredRPM = updateEMA(rawRPM); // Use EMA filtered RPM for calculations

    // Calculate current speed in m/s for IDM (ensure WHEEL_DIAMETER is correct!)
    const float WHEEL_DIAMETER_M = 0.1; // EXAMPLE: 10cm wheel diameter - SET YOUR ACTUAL VALUE
    const float WHEEL_CIRCUMFERENCE_M = PI * WHEEL_DIAMETER_M;
    // Use g_filteredRPM as it's smoother
    float current_speed_mps = (g_filteredRPM / 60.0) * WHEEL_CIRCUMFERENCE_M; // m/s


    // --- Read Distance Sensor ---
    static uint32_t lastToFMeasureTime = 0;
    uint16_t currentRawDistance = UINT16_MAX; // Store this measurement cycle's raw value

    if (millis() - lastToFMeasureTime >= 30) { // Sample rate (>=30ms sensor speed)
        lastToFMeasureTime = millis();
        currentRawDistance = readDistance(); // Use updated function

        if (currentRawDistance != UINT16_MAX) { // Got a VALID reading (100-1800mm)
            invalid_count = 0; // Reset error counter
            uint16_t avgDistance = applyMovingAverage(currentRawDistance); // Apply moving average first
            if (avgDistance != UINT16_MAX) { // Check if average is valid
                 // Only apply Kalman if average is valid
                 filteredDistance = (uint16_t)kalmanFilter((float)avgDistance);
                 // Clamp filtered distance to plausible range (optional but recommended)
                 filteredDistance = constrain(filteredDistance, MIN_VALID_DISTANCE_TOF, MAX_VALID_DISTANCE_TOF);
                 tof_ready = true; // Sensor is providing valid data
            } else {
                 // Moving average buffer might be full of invalids still
                 filteredDistance = UINT16_MAX; // Keep filtered as invalid
                 tof_ready = false;
            }
        } else { // Got UINT16_MAX (I2C error OR reading outside 100-1800mm range)
            // Insert the invalid reading into the buffer so moving avg eventually reflects it
            applyMovingAverage(UINT16_MAX);
            if (++invalid_count >= MAX_INVALID_ALLOWED) {
                // After several invalid readings, consider the path unclear or sensor error
                filteredDistance = UINT16_MAX; // Ensure filtered is invalid
                tof_ready = false; // Mark ToF as not ready if consistently failing
                 // Reset Kalman? Optional. Resetting might cause jumps when valid data returns.
                 // x_est = MAX_VALID_DISTANCE_TOF; P_est = 100;
            }
             // If < MAX_INVALID_ALLOWED, retain the *last known* filteredDistance value?
             // This prevents reacting to brief glitches but keeps old data if sensor fails.
             // Current code lets filteredDistance stay until MAX_INVALID is hit.
        }
    }
    // --- End Read Distance Sensor ---


    // =================================================
    // --- IDM Logic (REVISED THRESHOLDS NEEDED) ---
    // =================================================
    // Define new thresholds based on 100mm minimum
    const uint16_t IDM_STOP_DISTANCE = 150;     // mm - Example: Stop if object closer than 15cm
    const uint16_t IDM_CONTROL_MAX_DIST = 1000; // mm - Example: Use IDM up to 1 meter
    // Note: s0 (min desired gap) in IDM vars should also be >= 100, e.g. 150

    if (!tof_ready || filteredDistance == UINT16_MAX) {
        // Case 1: ToF Invalid / Not Ready
        // Fallback: Use speed from Raspberry Pi, assume path is clear
        override_by_IDM = false;
        // Serial.println("[MODE] ToF Invalid -> RPi Control");
    } else if (filteredDistance < IDM_STOP_DISTANCE) {
        // Case 2: Object VERY close (but valid reading) -> Emergency Stop
        override_by_IDM = true;
        RPM_setpoint = 0; // Target 0 RPM
        setMotorPWM(0); // Force stop motor NOW
        integral = 0; // Reset PID integral
        previousErrorRPM = 0;
        // Serial.printf("[MODE] IDM STOP! Dist: %u mm\n", filteredDistance);
        // Skip PID update below? Setting setpoint to 0 might be enough.
    } else if (filteredDistance <= IDM_CONTROL_MAX_DIST) {
        // Case 3: Object in control range (e.g., 150mm - 1000mm) -> Use IDM
        override_by_IDM = true;
        // Use HB100 relative speed? Ensure hb100Velocity is reliable & converted to m/s
        // NOTE: HB100 gives RELATIVE speed. If vehicle moves at V, object moves at V_obj,
        // HB100 might give V - V_obj (or V_obj - V). Be careful with the sign.
        // Assuming positive hb100Velocity means closing speed for simplicity here.
        // Convert hb100Velocity (if in m/s) for IDM's 'dv' term.
        // IDM expects dv = v_leader - v_follower. If HB100 is v_follower - v_leader, use -hb100Velocity.
        // Let's assume hb100Velocity is correct relative speed for now.
        float relative_speed_mps = hb100Velocity; // Ensure this is m/s
        updateRPM_IDM(current_speed_mps, relative_speed_mps, filteredDistance); // Update RPM_setpoint via IDM
        // RPM_setpoint calculated by updateRPM_IDM is the target
        // Serial.printf("[MODE] IDM Adjust. Target RPM: %.1f Dist: %u mm\n", RPM_setpoint, filteredDistance);
    } else {
        // Case 4: Object is far away or path clear (filteredDistance > IDM_CONTROL_MAX_DIST)
        override_by_IDM = false; // Revert control to RPi command
        // Serial.println("[MODE] Path Clear -> RPi Control");
    }
    // =================================================
    // --- End IDM Logic ---
    // =================================================


    // --- Determine Final RPM Setpoint ---
    float final_rpm_target;
    if (override_by_IDM) {
        // Use the RPM setpoint calculated by IDM (or 0 if stopping)
        final_rpm_target = RPM_setpoint;
    } else {
        // Not overridden by IDM, smoothly transition towards RPi commanded speed
        rpm_from_rasp_filtered = (RPM_SMOOTH_ALPHA * rpm_from_raspberry) + (1.0 - RPM_SMOOTH_ALPHA) * rpm_from_rasp_filtered;
        float delta_rpm = rpm_from_rasp_filtered - previousSetpoint;
        delta_rpm = constrain(delta_rpm, -MAX_RPM_DELTA, MAX_RPM_DELTA);
        final_rpm_target = previousSetpoint + delta_rpm;
    }
    // Ensure target is not negative if motor only goes forward
    final_rpm_target = max(0.0f, final_rpm_target);

    // Update the setpoint used for next smoothing iteration AND PID
    previousSetpoint = final_rpm_target;


    // --- PID Control Loop ---
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDRunTime >= PID_INTERVAL_MS) {
        lastPIDRunTime = currentMillis;

        // Check if target is effectively zero
        if (abs(final_rpm_target) < 0.1) {
            integral = 0.0; // Reset integral
            previousErrorRPM = 0.0; // Reset derivative term basis
             setMotorPWM(0); // Explicitly set PWM to 0 if target is 0
        } else if (override_by_IDM && RPM_setpoint == 0 && filteredDistance < IDM_STOP_DISTANCE) {
             // Special case: IDM forced an immediate stop, motor PWM already set to 0.
             // Keep integral reset, maybe don't run PID calculation?
             integral = 0.0;
             previousErrorRPM = 0.0;
             // setMotorPWM(0) was already called in IDM logic
        }
        else {
            // Calculate PID normally using the final_rpm_target
            // g_filteredRPM was updated earlier
            float targetEffort_RPM = calculatePID_RPM_Output(final_rpm_target, g_filteredRPM);
            int targetPWM = transformRPMtoPWM(targetEffort_RPM);
            setMotorPWM(targetPWM); // Apply calculated PWM
        }
    } // --- End of PID Control Loop ---


    // --- Servo Control ---
    control_servo(); // Update steering

    // --- Display Data ---
    displayData(); // Ensure this prints useful info: final_rpm_target, g_filteredRPM, filteredDistance, override_by_IDM
}


// =========================================================================
// FUNCTION: displayData (Modified to show more info)
// =========================================================================
void displayData() {
    static unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL_MS = 250; // Print less frequently

    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();

        float rawRPM_display;
        noInterrupts();
        rawRPM_display = currentRPM;
        interrupts();

        Serial.print("TARGET_RPM:"); // Final target sent to PID
        Serial.print(previousSetpoint, 1); // Use previousSetpoint as it holds the final target
        Serial.print(", RPi_RPM:"); // Commanded by RPi
        Serial.print(rpm_from_raspberry, 1);
        Serial.print(", ActualRPM:"); // EMA filtered RPM
        Serial.print(g_filteredRPM, 1);
        Serial.print(", PWM:");
        Serial.print(currentPWM); // Actual PWM applied
        Serial.print(", TOF_Dist:");
        if (filteredDistance == UINT16_MAX) {
            Serial.print("INV"); // Invalid
        } else {
            Serial.print(filteredDistance);
        }
         Serial.print(", TOF_Raw:"); // Last Raw reading
        if (currentRawDistance == UINT16_MAX) { // Need to capture raw value in loop scope
            Serial.print("INV");
        } else {
            Serial.print(currentRawDistance); // Requires currentRawDistance available here
        }
        Serial.print(", IDM_Active:");
        Serial.print(override_by_IDM ? "Y" : "N");
        Serial.print(", HB100_Vel:"); // Doppler Velocity
        Serial.print(hb100Velocity, 2);
        Serial.println(); // Newline
    }
}

// ... (Keep other functions like updateEMA, calculatePID_RPM_Output, transformRPMtoPWM, updateRPM_IDM, control_servo, parseSerialData, ISRs, Timer callbacks etc.) ...
// REMEMBER TO:
// 1. SET YOUR ACTUAL WHEEL_DIAMETER_M in loop()
// 2. TUNE the new IDM thresholds (IDM_STOP_DISTANCE, IDM_CONTROL_MAX_DIST, s0)
// 3. TUNE PID Gains (Kp, Ki, Kd) again after these changes.
// 4. VERIFY HB100 velocity reading and its sign for IDM usage.

#include <Arduino.h>
#include <math.h>
#include "esp_timer.h" // For ESP32 specific timer

// --- Pin Definitions ---
#define RPWM 2
#define LPWM 15
#define R_EN 0
#define L_EN 4
#define IS_R 25 // Current Sense Right (assuming connected to right motor logic)
#define IS_L 26 // Current Sense Left (unused in current logic, but defined)
#define SERVO_PIN 13
#define HALL_SENSOR_PIN 12

// --- Ultrasonic Pin Definitions ---
#define echoPin 17 // Must be interrupt-capable
#define trigPin 16

// --- Ultrasonic Constants ---
// Speed of sound ~343 m/s = 0.0343 cm/us. Factor = (speed cm/us) / 2
#define SOUND_SPEED_FACTOR (0.01715f)
const unsigned long TRIGGER_PULSE_DURATION_US = 10;      // Standard HC-SR04 trigger pulse width
const unsigned long MIN_MEASUREMENT_INTERVAL_US = 60000; // Min 60ms between sensor triggers recommended
const unsigned long MAX_ECHO_DURATION_US = 30000;        // Max duration to consider valid (relates to max range / timeout)

// --- Servo Constants ---
#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500
#define REFRESH_INTERVAL 20000 // microseconds (for servo pulse generation)
#define SERVO_MIN_ANGLE 40
#define SERVO_MAX_ANGLE 130

// --- Motor & Driver Constants ---
#define CURRENT_LIMIT_MV 7000 // Example: Needs calibration! Max voltage allowed from IS_R pin in mV

// --- Encoder / RPM Calculation ---
const int PULSES_PER_REV = 5;
volatile unsigned long pulseCount = 0;
volatile float currentRPM = 0.0; // Raw RPM calculated by timer
const unsigned long RPM_CALC_INTERVAL_MS = 500;
const unsigned long RPM_CALC_INTERVAL_US = RPM_CALC_INTERVAL_MS * 1000;
volatile unsigned long lastRpmCalcTime = 0;
volatile unsigned long lastRpmCalcPulseCount = 0;
esp_timer_handle_t rpm_timer_handle;

// --- EMA Filter (for RPM smoothing) ---
const float EMA_ALPHA = 0.05; // Smoothing factor (0.0 < alpha <= 1.0). TUNE THIS.
float g_filteredRPM = 0.0;    // Global variable to hold the latest filtered RPM result

// --- PID Controller (RPM based) ---
float RPM_setpoint = 0.0; // Desired speed (RPM) - can be modified by serial or IDM

// PID Gains - MUST BE RETUNED for RPM control
float Kp = 1.0;   // Example starting value - TUNE ME!
float Ki = 0.5;   // Example starting value - TUNE ME!
float Kd = 0.0;   // Example starting value - TUNE ME!
// PID Timing & State Variables
const unsigned long PID_INTERVAL_MS = 10; // How often PID loop runs
const float PID_SAMPLE_TIME_S = (float)PID_INTERVAL_MS / 1000.0;
float integral = 0.0;         // PID integral state
float previousErrorRPM = 0.0; // Store previous error in RPM
float previousMeasuredRPM = 0.0; // For derivative on measurement
unsigned long lastPIDRunTime = 0;
int currentPWM = 0;           // Stores the last calculated PWM value applied to motor

// RPM-to-PWM Transformation Constants - MUST BE TUNED
// PWM = RPM_TO_PWM_SLOPE * targetEffort_RPM + RPM_TO_PWM_OFFSET
const float RPM_TO_PWM_SLOPE = 0.05;  // Example: PWM units per RPM - TUNE ME!
const float RPM_TO_PWM_OFFSET = 4.6; // Example: PWM needed to overcome static friction - TUNE ME!
const float MAX_RPM_ESTIMATE = 4000; // Example: Estimated max possible RPM for anti-windup - TUNE ME!

// --- Stop Detection Timer ---
const int STOP_CHECK_INTERVAL_US = 1000000; // 1 second
volatile unsigned long lastCheckedPulseCount = 0;
bool motorRunning = false; // Tracks if motor PWM > 0
esp_timer_handle_t stop_timer_handle;

// --- Serial Communication Buffer ---
const int SERIAL_BUFFER_SIZE = 10; // Expecting <angle(4bytes)speed(4bytes)> = 10 bytes
byte serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// --- Global State Variables ---
int32_t steering_angle = 90; // Desired servo angle (0-180 mapped)
int32_t received_speed = 0;  // Target speed received via Serial (used as basis for RPM_setpoint)

// --- Global variables for Ultrasonic ISR ---
volatile unsigned long g_echoStartTimeUs = 0;
volatile unsigned long g_echoEndTimeUs = 0;
volatile float g_latestDistanceCm = -1.0;    // Stores the latest valid distance (-1 = no valid reading yet)
volatile bool g_newReadingAvailable = false; // Flag set true by ISR when a reading is complete
unsigned long g_lastTriggerTimeUs = 0;       // Tracks the time the last trigger was sent

// =========================================================================
// INTERRUPT SERVICE ROUTINE (ISR) - Hall Sensor Encoder
// =========================================================================
void IRAM_ATTR hallSensorISR() {
    pulseCount++;
}

// =========================================================================
// INTERRUPT SERVICE ROUTINE (ISR) - Ultrasonic Echo Pin
// =========================================================================
void IRAM_ATTR echoISR() {
    unsigned long currentTimeUs = micros();
    if (digitalRead(echoPin) == HIGH) {
        g_echoStartTimeUs = currentTimeUs;
        g_newReadingAvailable = false; // New measurement cycle started
    } else {
        g_echoEndTimeUs = currentTimeUs;
        unsigned long durationUs = g_echoEndTimeUs - g_echoStartTimeUs;
        // Validate duration
        if (durationUs > 100 && durationUs < MAX_ECHO_DURATION_US) {
            g_latestDistanceCm = (float)durationUs * SOUND_SPEED_FACTOR;
            g_newReadingAvailable = true; // Valid reading complete
        }
        // If duration is out of range, do nothing, flag remains false
    }
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
    // Calculate only if time and pulses have changed significantly
    if (deltaTime_us > RPM_CALC_INTERVAL_US / 2 && deltaPulses > 0) {
        float pulses_per_second = (float)deltaPulses * 1000000.0 / (float)deltaTime_us;
        float rps = pulses_per_second / (float)PULSES_PER_REV; // Revolutions per second
        calculatedRPM = rps * 60.0;                            // Revolutions per minute
    } else if (deltaTime_us > RPM_CALC_INTERVAL_US * 2) { // If no pulses for ~2 intervals, check setpoint
        // Check if motor *should* be running based on setpoint before forcing 0
        if (abs(RPM_setpoint) < 0.1) { // Check if setpoint is effectively zero
             calculatedRPM = 0.0;
        }
         // else: Keep last calculated RPM if setpoint is non-zero but no pulses detected recently
    } else {
         // Not enough time elapsed or no pulses, keep previous RPM
         noInterrupts();
         calculatedRPM = currentRPM; // Keep last value
         interrupts();
    }


    // Atomically update global currentRPM
    noInterrupts();
    currentRPM = calculatedRPM;
    interrupts();

    // Store values for next calculation
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

    // If the motor is supposed to be running (PWM > 0) but pulses haven't changed for STOP_CHECK_INTERVAL_US
    if (motorRunning && currentPulseReading == lastCheckedPulseCount) {
        noInterrupts();
        currentRPM = 0.0; // Force measured RPM to zero as it seems stalled
        interrupts();
        // Consider adding logic here to stop the motor or flag an error if stall is critical
    }
    lastCheckedPulseCount = currentPulseReading; // Update for next check
}

// =========================================================================
// FUNCTION: setMotorPWM - Applies PWM and handles basic current limiting
// =========================================================================
void setMotorPWM(int pwmValue) {
    // Constrain PWM to valid range
    pwmValue = constrain(pwmValue, 0, 255);
    currentPWM = pwmValue; // Store the value being applied (for display/debug)

    // --- Current Limiting Check ---
    // Read voltage from current sense pin (assuming IS_R is relevant)
    uint16_t raw_IS_R = analogRead(IS_R);
    // Convert raw ADC value to millivolts (assuming 3.3V ref, 12-bit ADC)
    float voltage_IS_R_mV = raw_IS_R * (3300.0 / 4095.0);

    // Check against the defined limit
    if (voltage_IS_R_mV < CURRENT_LIMIT_MV) {
        // Current OK: Enable driver and set PWM
        digitalWrite(R_EN, HIGH); // Enable Right channel (assuming affects LPWM/RPWM)
        digitalWrite(L_EN, HIGH); // Enable Left channel (assuming affects LPWM/RPWM) - Adjust if only one enable needed
        analogWrite(LPWM, pwmValue); // Set speed (assuming LPWM controls speed)
        digitalWrite(RPWM, LOW);     // Set direction (assuming RPWM=LOW means forward)
        motorRunning = (pwmValue > 0); // Update motor running state flag
    } else {
        // CURRENT LIMIT EXCEEDED! Stop the motor immediately
        Serial.print("!!! CURRENT LIMIT EXCEEDED (");
        Serial.print(voltage_IS_R_mV);
        Serial.println(" mV) !!!");

        digitalWrite(R_EN, LOW);     // Disable driver
        digitalWrite(L_EN, LOW);     // Disable driver
        analogWrite(LPWM, 0);        // Set PWM to 0
        digitalWrite(RPWM, LOW);     // Ensure direction pin is off
        motorRunning = false;        // Update motor running state flag
        integral = 0;                // Reset PID integral term on fault
        currentPWM = 0;              // Reflect that applied PWM is now 0
        RPM_setpoint = 0;            // Optionally stop trying to run
    }
}

// =========================================================================
// FUNCTION: updateEMA - Exponential Moving Average Filter
// =========================================================================
float updateEMA(float currentRawValue) {
    static float previousEMA = 0.0;
    static bool initialized = false;

    if (!initialized) {
        // Initialize with the first non-zero reading if possible, or just the first reading
        if (abs(currentRawValue) > 0.01) {
           previousEMA = currentRawValue;
           initialized = true;
        } else {
            // If starting at zero, wait for a non-zero value to initialize
            return 0.0; // Return 0 until initialized with a real value
        }
    }

    // Apply EMA formula
    float currentEMA = (EMA_ALPHA * currentRawValue) + ((1.0 - EMA_ALPHA) * previousEMA);
    previousEMA = currentEMA; // Update state for next call
    return currentEMA;
}


// =========================================================================
// FUNCTION: calculatePID_RPM_Output - PID calculation targeting RPM effort
// Uses Derivative on Measurement to reduce setpoint kick
// =========================================================================
float calculatePID_RPM_Output(float setpointRPM, float measuredRPM) {
    // --- Calculate Error (in RPM) ---
    float errorRPM = setpointRPM - measuredRPM;

    // --- Integral Term (with anti-windup) ---
    // Estimate max contribution needed from integral (e.g., enough to reach max RPM from offset)
    // This needs tuning based on how much integral is needed to overcome load/friction.
    float maxIntegralContribution_RPM = MAX_RPM_ESTIMATE * 0.5; // Example: Allow integral to contribute up to half the max RPM - TUNE ME!
    float maxIntegral = (Ki > 0.001) ? maxIntegralContribution_RPM / Ki : 1e9; // Avoid division by zero

    // Accumulate integral error, applying basic anti-windup clamping
    integral += errorRPM * PID_SAMPLE_TIME_S;
    integral = constrain(integral, -maxIntegral, maxIntegral);

    // --- Derivative Term (on Measurement) ---
    float derivative = 0;
    if (PID_SAMPLE_TIME_S > 0) {
        derivative = (measuredRPM - previousMeasuredRPM) / PID_SAMPLE_TIME_S;
    }

    // --- PID Calculation (Output is RPM-based effort) ---
    float p_term = Kp * errorRPM;
    float i_term = Ki * integral;
    // Derivative on measurement is typically subtracted (assuming Kd is positive)
    float d_term = Kd * derivative;
    float pidOutput_RPM = p_term + i_term - d_term; // P + I - D(measurement)

    // --- Update State for Next Iteration ---
    previousErrorRPM = errorRPM;       // Store error for analysis/debug if needed
    previousMeasuredRPM = measuredRPM; // Store current measurement for next derivative calculation

    // --- Return calculated effort (in RPM units) ---
    return pidOutput_RPM;
}

// =========================================================================
// FUNCTION: transformRPMtoPWM - Maps desired RPM effort to PWM value
// =========================================================================
int transformRPMtoPWM(float targetEffort_RPM) {
    // Apply linear transformation: PWM = m * RPM + c
    float calculatedPWM_f = RPM_TO_PWM_SLOPE * targetEffort_RPM + RPM_TO_PWM_OFFSET;

    // Constrain the result to the valid PWM range [0, 255]
    int pwmOut = constrain((int)round(calculatedPWM_f), 0, 255);

    // If the target RPM is very close to zero, force PWM to zero
    if (abs(targetEffort_RPM) < 0.1) { // Threshold might need tuning
       pwmOut = 0;
    }
    // Ensure minimum PWM offset is applied only if target RPM is positive
    // (avoids applying offset when trying to stop) - This is handled by the linear eq + constrain usually.

    return pwmOut;
}

// =========================================================================
// FUNCTION: control_servo - Sends pulse to servo based on steering_angle
// =========================================================================
void control_servo() {
    static unsigned long lastServoUpdate = 0;
    unsigned long now = micros();

    // Refresh servo signal periodically
    if (now - lastServoUpdate >= REFRESH_INTERVAL) {
        lastServoUpdate = now;

        // Constrain angle to limits
        int constrainedAngle = constrain(steering_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        // Map angle (0-180 reference) to servo pulse width (microseconds)
        int pulseWidth = map(constrainedAngle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

        // Send pulse
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(pulseWidth);
        digitalWrite(SERVO_PIN, LOW);
    }
}

// =========================================================================
// FUNCTION: parseSerialData - Extracts angle and speed from serial buffer
// =========================================================================
void parseSerialData() {
    // Check for start '<' and end '>' markers
    if (serialBuffer[0] == '<' && serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') {
        // Extract angle (bytes 1-4) and speed (bytes 5-8) assuming little-endian
        memcpy(&steering_angle, &serialBuffer[1], sizeof(int32_t));
        memcpy(&received_speed, &serialBuffer[5], sizeof(int32_t));

        // Use the received speed as the base target RPM (IDM might override this)
        // RPM_setpoint = (float)received_speed; // Setpoint is now handled in loop based on IDM result

         //Serial.print("Received Angle: "); Serial.print(steering_angle);
         //Serial.print(", Speed: "); Serial.println(received_speed);

    } else {
        Serial.println("Invalid serial packet received.");
    }
    // Reset buffer index regardless of success or failure
    serialBufferIndex = 0;
}

// =========================================================================
// FUNCTION: displayData - Prints key variables to Serial Plotter format
// =========================================================================
void displayData() {
    static unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL_MS = 200; // Print every 200ms

    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();

        // Get raw RPM value atomically for display
        float rawRPM_display;
        noInterrupts();
        rawRPM_display = currentRPM;
        interrupts();

        // Format: SetpointRPM,RawRPM,FilteredRPM,AppliedPWM
        Serial.print("SetpointRPM:");
        Serial.print(RPM_setpoint); // Current target RPM
        Serial.print(",");
        Serial.print("RawRPM:");
        Serial.print(rawRPM_display, 2); // Raw calculated RPM
        Serial.print(",");
        Serial.print("FilteredRPM:");
        Serial.print(g_filteredRPM, 2); // Smoothed RPM used by PID
        Serial.print(",");
        Serial.print("PWM:");
        Serial.println(currentPWM); // Actual PWM applied
    }
}

// =========================================================================
// FUNCTION: triggerUltrasonicMeasurement - Initiates ultrasonic ping
// =========================================================================
void triggerUltrasonicMeasurement() {
    unsigned long currentTimeUs = micros();
    // Check if enough time has passed since the last trigger
    if (currentTimeUs - g_lastTriggerTimeUs >= MIN_MEASUREMENT_INTERVAL_US) {
        g_newReadingAvailable = false; // Reset flag before triggering
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(TRIGGER_PULSE_DURATION_US);
        digitalWrite(trigPin, LOW);
        g_lastTriggerTimeUs = currentTimeUs; // Record trigger time
    }
}

// =========================================================================
// FUNCTION: getUltrasonicDistanceCm - Returns latest distance from ISR
// =========================================================================
float getUltrasonicDistanceCm() {
    return g_latestDistanceCm; // Return value updated by echoISR
}


// =========================================================================
// FUNCTION: calculateIDM - Intelligent Driver Model (Basic ACC)
// Takes distance (cm), current speed (RPM), desired max speed (RPM)
// Returns an adjusted target RPM based on distance.
// NOTE: Assumes IDM constants are tuned for RPM units or uses speed conversion.
//       Using direct RPM might require non-standard tuning.
// =========================================================================
float calculateIDM(float distance, float currentRPM, float desiredRPM) {
    // IDM Constants - NEED TUNING (especially if using RPM directly)
    // These might need adjustment based on whether speeds are RPM or converted linear speed.
    const float a_max = 0.5;     // Max acceleration (RPM/s or linear equivalent) - TUNE
    const float delta = 4.0;     // Acceleration exponent (usually 4)
    const float s0 = 5.0;        // Min gap distance (cm) - TUNE
    const float T_gap = 1.0;     // Desired time gap (s) - TUNE
    const float b_comfort = 0.3; // Comfortable deceleration (RPM/s or linear equivalent) - TUNE

    // Use local variables for calculation clarity
    float v = currentRPM;            // Current speed
    float v0 = max(0.1f, desiredRPM); // Desired speed (avoid division by zero)
    float s = max(0.1f, distance);    // Current gap distance (avoid division by zero)

    // Calculate desired minimum gap (s*) - Check units carefully!
    // If v is RPM, v*T_gap is (rev/min)*s, v*v/(...) is (rev/min)*s. This is inconsistent with s0 (cm).
    // --> Assumes the formula is adapted/tuned empirically for RPM, or conversion is needed.
    // --> For this example, we proceed assuming empirical tuning for RPM was done.
    float v_term = v * T_gap; // Units: RPM * s
    float brake_term = (v * v) / (2.0 * sqrt(a_max * b_comfort)); // Units: RPM^2 / sqrt(RPM/s * RPM/s) -> RPM * s
    // This formula assumes linear velocity units. Using RPM directly might not be physically accurate without scaling.
    // Let's calculate s_star but be aware of the unit issue.
    float s_star = s0 + max(0.0f, v_term + brake_term); // Ensure terms aren't negative

    // Calculate acceleration command based on IDM formula
    float accel_term = 1.0 - pow(v / v0, delta);
    float gap_term = pow(s_star / s, 2.0);
    float acceleration = a_max * (accel_term - gap_term);

    // Calculate next target RPM using simple Euler integration
    float nextRPM = v + acceleration * PID_SAMPLE_TIME_S; // PID sample time as dt

    // Bound the result: cannot be negative, cannot exceed original desired speed
    nextRPM = constrain(nextRPM, 0.0, desiredRPM);

    return nextRPM;
}


// =========================================================================
// SETUP FUNCTION
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("===== ESP32 Motor Control + IDM Initializing =====");
    Serial.println("Build timestamp: " __DATE__ " " __TIME__); // Add build time

    // --- Pin Modes ---
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW); // Ensure servo pin is low initially
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(IS_R, INPUT);
    pinMode(IS_L, INPUT);
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP); // Use internal pullup for hall sensor

    // Ultrasonic Pin Modes
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    digitalWrite(trigPin, LOW); // Ensure trigger starts low

    // --- Initial Motor State ---
    digitalWrite(R_EN, LOW); // Disable driver initially
    digitalWrite(L_EN, LOW); // Disable driver initially
    analogWrite(LPWM, 0);    // Set PWM to 0
    digitalWrite(RPWM, LOW); // Set direction (e.g., LOW = forward)
    setMotorPWM(0);          // Ensure motor is stopped using the function

    // --- Attach Interrupts ---
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);
    Serial.println("Hall Sensor ISR Attached.");
    // Attach Ultrasonic Interrupt
    attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);
    Serial.println("Ultrasonic Echo ISR Attached.");

    // --- Initialize Timers & State ---
    lastRpmCalcTime = micros();
    lastPIDRunTime = millis();
    noInterrupts(); // Temporarily disable interrupts for atomic init
    lastRpmCalcPulseCount = pulseCount;
    lastCheckedPulseCount = pulseCount;
    interrupts(); // Re-enable interrupts
    Serial.println("State variables initialized.");

    // --- Setup ESP32 Timers ---
    // RPM Calculation Timer
    esp_timer_create_args_t rpm_timer_args = { .callback = &rpm_timer_callback, .name = "rpm_calc"};
    esp_timer_create(&rpm_timer_args, &rpm_timer_handle);
    esp_timer_start_periodic(rpm_timer_handle, RPM_CALC_INTERVAL_US);
    Serial.println("RPM Calculation Timer Started.");

    // Stop Detection Timer
    esp_timer_create_args_t stop_timer_args = { .callback = &stop_timer_callback, .name = "motor_stop_check"};
    esp_timer_create(&stop_timer_args, &stop_timer_handle);
    esp_timer_start_periodic(stop_timer_handle, STOP_CHECK_INTERVAL_US);
    Serial.println("Motor Stop Check Timer Started.");

    Serial.println("===== Setup Complete. Waiting for data... =====");
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
    // --- 1. Process Incoming Serial Data ---
    while (Serial.available() > 0) {
        uint8_t byteRead = Serial.read();
        // Look for start marker '<'
        if (serialBufferIndex == 0 && byteRead != '<') {
            continue; // Ignore bytes until start marker
        }
        // Fill buffer
        if (serialBufferIndex < SERIAL_BUFFER_SIZE) {
            serialBuffer[serialBufferIndex++] = byteRead;
        }
        // Check if buffer is full
        if (serialBufferIndex == SERIAL_BUFFER_SIZE) {
            // Check for end marker '>' and parse
            if (serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') {
                parseSerialData(); // Updates steering_angle and received_speed
            } else {
                Serial.println("Serial Frame Error: No end marker or incorrect size.");
                serialBufferIndex = 0; // Reset index on error
            }
            // Always reset index after attempting parse or on error if buffer full
           // serialBufferIndex = 0; // This was inside parseSerialData, keep it there.
        }
         // Handle buffer overflow? If index goes beyond size (shouldn't happen with check above)
         if (serialBufferIndex >= SERIAL_BUFFER_SIZE) {
             serialBufferIndex = 0; // Reset defensively
         }
    }

    // --- 2. Ultrasonic Sensing & IDM ---
    triggerUltrasonicMeasurement(); // Initiate ping if interval met
    float currentDistance = getUltrasonicDistanceCm(); // Get latest distance from ISR

    float base_target_rpm = (float)received_speed; // Start with speed from serial

    // Apply IDM logic only if a valid distance reading is available
    if (currentDistance >= 0) {
        // Use IDM to adjust the RPM setpoint based on distance
        if (currentDistance < 10) { // Emergency stop distance
            RPM_setpoint = 0;
             // Serial.printf("[IDM] STOP Condition! Dist: %.1f cm\n", currentDistance);
        } else if (currentDistance <= 25) { // IDM active zone (Tune this upper limit)
            RPM_setpoint = calculateIDM(currentDistance, g_filteredRPM, base_target_rpm);
            // Serial.printf("[IDM] Active. Dist: %.1f cm, TargetRPM: %.2f\n", currentDistance, RPM_setpoint);
        } else {
            // Distance is large, use the base target speed
            RPM_setpoint = base_target_rpm;
            // Serial.printf("[IDM] Clear path. Dist: %.1f cm, TargetRPM: %.2f\n", currentDistance, RPM_setpoint);
        }
    } else {
        // No valid distance reading yet (or sensor failed), use base target speed
        RPM_setpoint = base_target_rpm;
        // Serial.printf("[IDM] No valid distance reading. TargetRPM: %.2f\n", RPM_setpoint);
    }

    // --- 3. Get Filtered RPM ---
    // Read raw RPM calculated by timer ISR
    float rawRPM;
    noInterrupts();
    rawRPM = currentRPM;
    interrupts();
    // Apply EMA filter
    g_filteredRPM = updateEMA(rawRPM);

    // --- 4. PID Control Loop (Runs periodically) ---
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDRunTime >= PID_INTERVAL_MS) {
        lastPIDRunTime = currentMillis; // Update time of this PID run

        // Reset integral if setpoint is zero (helps prevent windup at stop)
        if (abs(RPM_setpoint) < 0.1) {
            integral = 0.0;
            // previousErrorRPM = 0.0; // Already handled by pid calc if error is 0
             previousMeasuredRPM = 0.0; // Assume speed is zero if setpoint is zero
        }

        // Calculate desired control effort in RPM units using PID
        float targetEffort_RPM = calculatePID_RPM_Output(RPM_setpoint, g_filteredRPM);

        // Transform the RPM-based effort into a PWM value
        int targetPWM = transformRPMtoPWM(targetEffort_RPM);

        // Apply the calculated PWM to the motor (includes current check)
        setMotorPWM(targetPWM);
    }

    // --- 5. Servo Control ---
    control_servo(); // Updates servo position based on steering_angle

    // --- 6. Display Data ---
    displayData(); // Prints data to Serial Plotter periodically
}
// Removed extra closing brace that might have been here previously. End of code.

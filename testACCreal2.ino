#include <Arduino.h>
#include <math.h>
#include "esp_timer.h" // For ESP32 specific timer
// #include <driver/pcnt.h> // *** REMOVED: PCNT no longer needed ***

// --- Pin Definitions ---
#define RPWM 2
#define LPWM 15
#define R_EN 0
#define L_EN 4
#define IS_R 25      // Analog Input for Current Sense R
#define IS_L 26      // Analog Input for Current Sense L (If used)
#define SERVO_PIN 13
#define HALL_SENSOR_PIN 12
// #define HB100_PIN   GPIO_NUM_34 // *** REMOVED: PCNT pin no longer needed ***

// --- Servo Constants ---
#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500
#define REFRESH_INTERVAL 20000 // Micros for ~50Hz
#define SERVO_MIN_ANGLE 40
#define SERVO_MAX_ANGLE 130

// --- Motor & Driver Constants ---
#define CURRENT_LIMIT_MV 7000 // Example: Needs calibration!

// --- Encoder / RPM Calculation ---
const int PULSES_PER_REV = 5;
volatile unsigned long pulseCount = 0;
volatile float currentRPM = 0.0; // Raw calculated RPM
const unsigned long RPM_CALC_INTERVAL_MS = 600; // How often RPM is calculated (ms)
const unsigned long RPM_CALC_INTERVAL_US = RPM_CALC_INTERVAL_MS * 1000;
volatile unsigned long lastRpmCalcTime = 0;
volatile unsigned long lastRpmCalcPulseCount = 0;
esp_timer_handle_t rpm_timer_handle;

// --- EMA Filter for RPM ---
const float EMA_ALPHA = 0.1; // Smoothing factor
float g_filteredRPM = 0.0;   // Global variable to hold the latest filtered RPM result

// --- HLK-LD1115H Radar Sensor (UART) ---
#define RADAR_SERIAL Serial2       // *** NEW: Use Serial2 for the radar ***
#define RADAR_RX_PIN 16            // *** NEW: Default RX pin for Serial2 ***
#define RADAR_TX_PIN 17            // *** NEW: Default TX pin for Serial2 ***
#define RADAR_BAUD_RATE 115200     // *** NEW: Baud rate - CHECK DATASHEET! ***
volatile float radar_distance_m = -1.0; // *** NEW: Stores measured distance (meters), -1 = invalid ***
volatile float radar_velocity_mps = 0.0; // *** NEW: Stores relative velocity (m/s), positive = away? CHECK DATASHEET! ***
char radarSerialBuffer[64];       // *** NEW: Buffer for incoming radar UART data ***
int radarBufferIndex = 0;         // *** NEW: Index for radar buffer ***

// --- ACC Control Logic Parameters ---
#define ACC_TARGET_DISTANCE_M 0.5 // *** NEW: Desired following distance (m) for <=1m simulation - TUNE ME ***
#define ACC_MAX_RANGE_M       1.5 // *** NEW: Ignore radar targets beyond this range for ACC - TUNE ME ***
// --- NEW PID GAINS specifically for ACC DISTANCE control - MUST BE TUNED ---
// These control how strongly the system reacts to distance errors.
float Kp_dist = 150.0; // Example: Proportional gain (RPM change per meter of error) - TUNE ME!
float Ki_dist = 5.0;   // Example: Integral gain - TUNE ME!
float Kd_dist = 50.0;  // Example: Derivative gain (based on relative velocity) - TUNE ME!
float integral_dist = 0.0;     // *** NEW: Integral state for distance PID ***
// float previousError_dist = 0.0; // *** NEW: Previous distance error (can use velocity instead for Kd) ***

// --- Motor Speed PID Controller (Existing - Coefficients unchanged) ---
float RPM_setpoint = 0.0; // Base desired speed (RPM) - Updated by serial data, adjusted by ACC
float Kp = 0.8;
float Ki = 0.5;
float Kd = 0.0; // Was 0 before, keeping it 0 unless tuning is needed later
// --- Motor PID Timing & State Variables ---
const unsigned long PID_INTERVAL_MS = 10; // How often motor PID runs (ms)
const float PID_SAMPLE_TIME_S = (float)PID_INTERVAL_MS / 1000.0;
float integral = 0.0;           // Motor speed integral state
float previousErrorRPM = 0.0;   // Motor speed previous error
unsigned long lastPIDRunTime = 0;
int currentPWM = 0;             // Stores the last calculated motor PWM

// --- RPM-to-PWM Transformation Constants (Existing - Unchanged) ---
const float RPM_TO_PWM_SLOPE = 0.05;
const float RPM_TO_PWM_OFFSET = 4.6;
const float MAX_RPM_ESTIMATE = 4500; // Used for motor PID anti-windup

// --- Stop Detection Timer ---
const int STOP_CHECK_INTERVAL_US = 1000000; // 1 second
volatile unsigned long lastCheckedPulseCount = 0;
bool motorRunning = false;
esp_timer_handle_t stop_timer_handle;

// --- Serial Communication Buffer (for Pi5 commands) ---
const int SERIAL_BUFFER_SIZE = 10;
byte serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// --- Global State Variables ---
volatile int32_t steering_angle = 90; // Updated by serial data
volatile int32_t received_speed = 0; // Base speed command from Pi5

// =========================================================================
// INTERRUPT SERVICE ROUTINE (ISR) - Hall Sensor / Encoder
// =========================================================================
void IRAM_ATTR hallSensorISR() {
    pulseCount++;
}

// =========================================================================
// FUNCTION: hb100_pcnt_init -> REMOVED
// =========================================================================
// void hb100_pcnt_init() { ... } // Entire function removed

// =========================================================================
// TIMER CALLBACK - RPM Calculation
// =========================================================================
void rpm_timer_callback(void *arg) {
    // ... (Keep existing code - no changes needed here) ...
     unsigned long currentTime_us = micros();
    unsigned long currentPulseReading;
    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();

    unsigned long deltaTime_us = currentTime_us - lastRpmCalcTime;
    unsigned long deltaPulses = currentPulseReading - lastRpmCalcPulseCount;

    float calculatedRPM = 0.0;
    if (deltaTime_us > 1000 && deltaPulses > 0) {
        float pulses_per_second = (float)deltaPulses * 1000000.0 / (float)deltaTime_us;
        float rps = pulses_per_second / (float)PULSES_PER_REV;
        calculatedRPM = rps * 60.0;
    }
    noInterrupts();
    currentRPM = calculatedRPM;
    interrupts();
    lastRpmCalcTime = currentTime_us;
    lastRpmCalcPulseCount = currentPulseReading;
}

// =========================================================================
// TIMER CALLBACK - Stop Detection
// =========================================================================
void stop_timer_callback(void *arg){
    // ... (Keep existing code - no changes needed here) ...
    unsigned long currentPulseReading;
    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();

    if (motorRunning && currentPulseReading == lastCheckedPulseCount) {
        noInterrupts();
        currentRPM = 0.0;
        interrupts();
    }
    lastCheckedPulseCount = currentPulseReading;
}

// =========================================================================
// TIMER CALLBACK - HB100 Velocity Calculation -> REMOVED
// =========================================================================
// void hb100_timer_callback(void *arg) { ... } // Entire function removed


// =========================================================================
// FUNCTION: setMotorPWM
// =========================================================================
void setMotorPWM(int pwmValue) {
    // ... (Keep existing code - no changes needed here) ...
     pwmValue = constrain(pwmValue, 0, 255);
    currentPWM = pwmValue; // Store the value being applied for display/debug
    uint16_t raw_IS_R = analogRead(IS_R);
    float voltage_IS_R_mV = raw_IS_R * (3300.0 / 4095.0);

    if (voltage_IS_R_mV < CURRENT_LIMIT_MV ) {
        digitalWrite(R_EN, HIGH);
        digitalWrite(L_EN, HIGH);
        analogWrite(LPWM, pwmValue);
        digitalWrite(RPWM, LOW);
        motorRunning = (pwmValue > 10);
    } else {
        Serial.printf("!!! CURRENT LIMIT EXCEEDED (%.1f mV) !!!\n", voltage_IS_R_mV);
        digitalWrite(R_EN, LOW);
        digitalWrite(L_EN, LOW);
        analogWrite(LPWM, 0);
        digitalWrite(RPWM, LOW);
        motorRunning = false;
        integral = 0; // Reset motor speed PID integral term on fault
        integral_dist = 0; // Also reset distance PID integral
        currentPWM = 0;
    }
}

// =========================================================================
// FUNCTION: updateEMA filter
// =========================================================================
float updateEMA(float currentRawValue) {
    // ... (Keep existing code - no changes needed here) ...
    static float previousEMA = 0.0;
    static bool initialized = false;
    if (!initialized) {
        if (abs(currentRawValue) > 0.01) {
             previousEMA = currentRawValue;
             initialized = true;
        }
        return currentRawValue;
    }
    float currentEMA = (EMA_ALPHA * currentRawValue) + ((1.0 - EMA_ALPHA) * previousEMA);
    previousEMA = currentEMA;
    return currentEMA;
}

// =========================================================================
// FUNCTION: calculatePID_RPM_Output (Motor Speed PID)
// =========================================================================
float calculatePID_RPM_Output(float setpointRPM, float measuredRPM) {
    // ... (Keep existing code - no changes needed here) ...
    // Note: Uses global Kp, Ki, Kd, integral, previousErrorRPM
     float errorRPM = setpointRPM - measuredRPM;
    float maxIntegralContribution_RPM = MAX_RPM_ESTIMATE * 1.1;
    float maxIntegral = (abs(Ki) > 1e-6) ? maxIntegralContribution_RPM / Ki : 1e9;
    integral += errorRPM * PID_SAMPLE_TIME_S;
    integral = constrain(integral, -abs(maxIntegral), abs(maxIntegral));
    float derivative = 0;
    if (PID_SAMPLE_TIME_S > 0) {
        derivative = (errorRPM - previousErrorRPM) / PID_SAMPLE_TIME_S;
    }
    float p_term = Kp * errorRPM;
    float i_term = Ki * integral;
    float d_term = Kd * derivative;
    float pidOutput_RPM = p_term + i_term + d_term;
    previousErrorRPM = errorRPM;
    return pidOutput_RPM;
}

// =========================================================================
// FUNCTION: transformRPMtoPWM (Motor Speed PID)
// =========================================================================
int transformRPMtoPWM(float targetEffort_RPM) {
    // ... (Keep existing code - no changes needed here) ...
     float calculatedPWM_f = RPM_TO_PWM_SLOPE * targetEffort_RPM + RPM_TO_PWM_OFFSET;
    int pwmOut = constrain((int)round(calculatedPWM_f), 0, 255);
    if (abs(RPM_setpoint) < 0.1 && pwmOut < (int)round(RPM_TO_PWM_OFFSET)+5) {
        pwmOut = 0;
    }
    return pwmOut;
}

// =========================================================================
// FUNCTION: control_servo
// =========================================================================
void control_servo() {
    // ... (Keep existing code - consider replacing blocking delay) ...
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
// FUNCTION: parseSerialData (From Pi5)
// =========================================================================
void parseSerialData() {
    // ... (Keep existing code - Updates steering_angle and received_speed) ...
    if (serialBuffer[0] == '<' && serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') {
        int32_t received_angle_local = *((int32_t*)&serialBuffer[1]);
        int32_t received_speed_local = *((int32_t*)&serialBuffer[5]);
        steering_angle = received_angle_local;
        received_speed = received_speed_local; // Store base speed command
        // RPM_setpoint is now potentially adjusted by ACC logic in the main loop
    } else {
        Serial.println("Invalid serial packet format in parseSerialData.");
    }
}

// =========================================================================
// FUNCTION: readRadarSerial - *** NEW PLACEHOLDER ***
// Reads data from RADAR_SERIAL, parses it, and updates globals.
// NEEDS TO BE IMPLEMENTED BASED ON HLK-LD1115H DATASHEET
// =========================================================================
void readRadarSerial() {
    while (RADAR_SERIAL.available()) {
        char receivedChar = RADAR_SERIAL.read();

        // --- !!! IMPLEMENT PARSING LOGIC HERE !!! ---
        // This depends HEAVILY on the HLK-LD1115H datasheet's UART protocol.
        // Example: Look for start/end characters, parse CSV, fixed-length binary, etc.
        // Below is a HYPOTHETICAL example assuming "D:dist_m,V:vel_mps\n" format

        if (receivedChar == '\n') { // Example: Process on newline
            radarSerialBuffer[radarBufferIndex] = '\0'; // Null-terminate string

            // Hypothetical parsing
            float dist = -1.0;
            float vel = 0.0;
            // Use sscanf, strtok, or manual parsing to extract values
            // Example using sscanf (adjust format string based on actual data!)
            if (sscanf(radarSerialBuffer, "D:%f,V:%f", &dist, &vel) == 2) {
                 // Successfully parsed distance and velocity
                 radar_distance_m = dist;   // Update global distance
                 radar_velocity_mps = vel;  // Update global velocity
            } else {
                 // Parsing failed
                 Serial.print("Radar parse error: "); Serial.println(radarSerialBuffer);
                 // Optionally invalidate readings on parse error
                 // radar_distance_m = -1.0;
                 // radar_velocity_mps = 0.0;
            }
            radarBufferIndex = 0; // Reset buffer index
        } else if (radarBufferIndex < sizeof(radarSerialBuffer) - 1) {
            // Add character to buffer if space available
            radarSerialBuffer[radarBufferIndex++] = receivedChar;
        } else {
            // Buffer overflow, reset index
            Serial.println("Radar buffer overflow!");
            radarBufferIndex = 0;
        }
        // --- !!! END OF PLACEHOLDER PARSING LOGIC !!! ---
    }
}


// =========================================================================
// FUNCTION: displayData (Updated to show Radar data)
// =========================================================================
void displayData() {
    static unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL_MS = 250; // Print every 250ms

    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();

        // Read volatile globals safely
        float rawRPM_display = currentRPM;
        float filteredRPM_display = g_filteredRPM;
        float setpoint_display = RPM_setpoint; // Final setpoint after ACC adjustment
        int pwm_display = currentPWM;
        float radar_dist_display = radar_distance_m;     // *** NEW ***
        float radar_vel_display = radar_velocity_mps;   // *** NEW ***
        int steer_display = steering_angle;

        // Print comma-separated values
        Serial.printf("SP:%.1f,RAW:%.1f,FILT:%.1f,PWM:%d,DIST:%.2f,RAD_V:%.2f,STEER:%d\n",
                      setpoint_display,
                      rawRPM_display,
                      filteredRPM_display,
                      pwm_display,
                      radar_dist_display,    // *** NEW ***
                      radar_vel_display,     // *** NEW ***
                      steer_display);
    }
}

// =========================================================================
// SETUP FUNCTION
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 ACC Sim Initializing (HLK-LD1115H)..."); // Updated message

    // --- Initialize Radar Serial ---
    // *** NEW: Begin UART communication with the radar module ***
    RADAR_SERIAL.begin(RADAR_BAUD_RATE, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    Serial.printf("Radar Serial%d initialized at %d baud.\n", RADAR_SERIAL == Serial1 ? 1 : 2, RADAR_BAUD_RATE);


    // --- Pin Modes ---
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW);
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(IS_R, INPUT);
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
    // pinMode(HB100_PIN, INPUT); // REMOVED

    // --- Initial State ---
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    analogWrite(LPWM, 0);
    digitalWrite(RPWM, LOW);
    setMotorPWM(0);

    // --- Attach Encoder Interrupt ---
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);
    Serial.println("Encoder ISR Attached.");

    // --- Initialize HB100 Pulse Counter -> REMOVED ---
    // hb100_pcnt_init();

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
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "rpm_calc"
    };
    esp_timer_create(&rpm_timer_args, &rpm_timer_handle);
    esp_timer_start_periodic(rpm_timer_handle, RPM_CALC_INTERVAL_US);
    Serial.println("RPM Timer Started.");

    // Stop Detection Timer
    esp_timer_create_args_t stop_timer_args = {
        .callback = &stop_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "motor_stop_check"
    };
    esp_timer_create(&stop_timer_args, &stop_timer_handle);
    esp_timer_start_periodic(stop_timer_handle, STOP_CHECK_INTERVAL_US);
    Serial.println("Stop Check Timer Started.");

    // HB100 Read Timer -> REMOVED ---
    // esp_timer_create_args_t hb100_timer_args = { ... };
    // esp_timer_create(&hb100_timer_args, &hb100_timer_handle);
    // esp_timer_start_periodic(hb100_timer_handle, HB100_READ_INTERVAL_US);

    Serial.println("Setup Complete. Waiting for serial data...");
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
    // --- Process Incoming Serial Data (From Pi5) ---
    while (Serial.available() > 0 && serialBufferIndex < SERIAL_BUFFER_SIZE) {
        uint8_t byteRead = Serial.read();
        if (serialBufferIndex == 0 && byteRead != '<') { continue; }
        serialBuffer[serialBufferIndex++] = byteRead;
        if (serialBufferIndex == SERIAL_BUFFER_SIZE) {
            parseSerialData(); // Updates steering_angle and received_speed
            serialBufferIndex = 0; // Reset for next packet
        }
    }

    // --- Process Incoming Serial Data (From Radar) ---
    // *** NEW: Call function to read and parse radar data ***
    readRadarSerial(); // Updates radar_distance_m and radar_velocity_mps


    // --- Get Filtered RPM ---
    float rawRPM;
    noInterrupts();
    rawRPM = currentRPM;
    interrupts();
    g_filteredRPM = updateEMA(rawRPM); // Use the filtered value for PID

    // --- ACC Logic & PID Control Loop (Runs periodically) ---
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDRunTime >= PID_INTERVAL_MS) {
        lastPIDRunTime = currentMillis;

        // --- ACC State Decision ---
        float final_rpm_setpoint = (float)received_speed; // Start with speed from Pi5
        bool target_detected = (radar_distance_m > 0.1 && radar_distance_m < ACC_MAX_RANGE_M); // Basic detection check

        if (target_detected) {
            // --- Gap Control Mode ---
            float distance_error = radar_distance_m - ACC_TARGET_DISTANCE_M;

            // Simple Distance PID - Calculate adjustment based on distance error & relative velocity
            // Integral term
            integral_dist += distance_error * PID_SAMPLE_TIME_S;
            // Basic Anti-windup for distance integral (needs tuning)
            float max_dist_integral = 200.0; // Example limit - TUNE ME
            integral_dist = constrain(integral_dist, -max_dist_integral, max_dist_integral);

            // Derivative term (using radar's relative velocity directly)
            // Note: Check sign convention of radar_velocity_mps! Assuming positive = moving away
            // We want to slow down if radar_velocity_mps is negative (closing gap)
            float derivative_dist = -radar_velocity_mps; // Use negative velocity as derivative input

            // Calculate PID output for distance control (units are roughly RPM adjustment)
            float distance_pid_output = (Kp_dist * distance_error) +
                                        (Ki_dist * integral_dist) +
                                        (Kd_dist * derivative_dist);

            // Adjust the base RPM setpoint from Pi5 based on distance PID
            final_rpm_setpoint = (float)received_speed - distance_pid_output;

            // Prevent ACC from demanding negative RPM if original setpoint was positive
            if (received_speed > 0) {
                 final_rpm_setpoint = max(0.0f, final_rpm_setpoint);
            }
            // Add constraints if needed (e.g., max deceleration)

        } else {
            // --- Speed Control Mode ---
            // No target detected or target out of range, use speed from Pi5 directly
            final_rpm_setpoint = (float)received_speed;
            // Reset distance integral when not following
            integral_dist = 0.0;
        }

        // Reset motor speed integral if final setpoint is zero
        if (abs(final_rpm_setpoint) < 0.1) {
            integral = 0.0;
            previousErrorRPM = 0.0;
        }

        // Update the global RPM_setpoint that the low-level PID will use
        RPM_setpoint = final_rpm_setpoint;


        // --- Low-Level Motor Speed PID ---
        // Calculate desired motor effort using the (potentially ACC-adjusted) RPM_setpoint
        float targetEffort_RPM = calculatePID_RPM_Output(RPM_setpoint, g_filteredRPM);

        // Transform effort to PWM
        int targetPWM = transformRPMtoPWM(targetEffort_RPM);

        // Apply PWM to motor
        setMotorPWM(targetPWM);

    } // --- End of PID Control Loop ---

    // --- Servo Control ---
    control_servo();

    // --- Display Data ---
    displayData();

    // delay(1); // Optional small delay
}
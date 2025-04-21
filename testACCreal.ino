#include <Arduino.h>
#include <math.h>
#include "esp_timer.h" // For ESP32 specific timer
#include <driver/pcnt.h> // *** NEW: Include for Pulse Counter ***

// --- Pin Definitions ---
#define RPWM 2
#define LPWM 15
#define R_EN 0
#define L_EN 4
#define IS_R 25      // Analog Input for Current Sense R
#define IS_L 26      // Analog Input for Current Sense L (If used)
#define SERVO_PIN 13
#define HALL_SENSOR_PIN 12
#define HB100_PIN   GPIO_NUM_14 // *** NEW: Choose an available GPIO for HB100 input ***

// --- Servo Constants ---
#define MIN_PULSE_WIDTH 500
#define MAX_PULSE_WIDTH 2500
#define REFRESH_INTERVAL 20000 // Micros for ~50Hz
#define SERVO_MIN_ANGLE 40
#define SERVO_MAX_ANGLE 130

// --- Motor & Driver Constants ---
// !!! NEEDS CALIBRATION for your specific BTS7960 module !!!
// This represents the voltage (in mV) corresponding to your desired max current
#define CURRENT_LIMIT_MV 7000 // Example: Needs calibration!

// --- Encoder / RPM Calculation ---
const int PULSES_PER_REV = 5; // Pulses per revolution of the motor/wheel
volatile unsigned long pulseCount = 0;
volatile float currentRPM = 0.0; // Raw calculated RPM
const unsigned long RPM_CALC_INTERVAL_MS = 100; // How often RPM is calculated (ms) - adjusted for stability
const unsigned long RPM_CALC_INTERVAL_US = RPM_CALC_INTERVAL_MS * 1000;
volatile unsigned long lastRpmCalcTime = 0;
volatile unsigned long lastRpmCalcPulseCount = 0;
esp_timer_handle_t rpm_timer_handle;

// --- EMA Filter for RPM ---
const float EMA_ALPHA = 0.1; // Smoothing factor (0.0 < alpha <= 1.0). TUNE THIS. Lower = smoother but more lag.
float g_filteredRPM = 0.0;   // Global variable to hold the latest filtered RPM result

// --- HB100 Doppler Sensor ---
#define PCNT_UNIT           PCNT_UNIT_0 // *** NEW: Choose a PCNT unit (0-7) ***
#define HB100_WAVELENGTH    0.0285 // *** NEW: Approx. wavelength in meters for 10.525 GHz ***
#define HB100_READ_INTERVAL_MS 250 // *** NEW: How often to calculate HB100 velocity (ms) ***
#define HB100_READ_INTERVAL_US (HB100_READ_INTERVAL_MS * 1000)
volatile float hb100_velocity_kmh = 0.0; // *** NEW: Stores calculated velocity ***
esp_timer_handle_t hb100_timer_handle;  // *** NEW: Timer for reading PCNT ***
// PCNT filter value - helps debounce noisy signals (adjust 0-1023)
#define PCNT_FILTER_VAL 50 // *** NEW: Example filter value (tune if needed) ***


// --- PID Controller ---
float RPM_setpoint = 0.0; // Desired speed (RPM) - Updated by serial data

// *** NEW PID GAINS - MUST BE RETUNED ***
float Kp = 0.8;  // Example starting value - TUNE ME!
float Ki = 0.4;  // Example starting value - TUNE ME!
float Kd = 0.05; // Example starting value - TUNE ME!

// --- PID Timing & State Variables ---
const unsigned long PID_INTERVAL_MS = 20; // How often PID runs (ms) - adjusted
const float PID_SAMPLE_TIME_S = (float)PID_INTERVAL_MS / 1000.0;
float integral = 0.0;
float previousErrorRPM = 0.0;
unsigned long lastPIDRunTime = 0;
int currentPWM = 0; // Stores the last calculated PWM value

// *** NEW: RPM-to-PWM Transformation Constants - MUST BE TUNED ***
const float RPM_TO_PWM_SLOPE = 0.06;  // Example: PWM units per RPM - TUNE ME!
const float RPM_TO_PWM_OFFSET = 5.0; // Example: PWM needed just to start motion - TUNE ME!
const float MAX_RPM_ESTIMATE = 4500; // Example: Estimated max RPM for anti-windup - TUNE ME!

// --- Stop Detection Timer ---
const int STOP_CHECK_INTERVAL_US = 1000000; // 1 second
volatile unsigned long lastCheckedPulseCount = 0;
bool motorRunning = false; // Updated in setMotorPWM
esp_timer_handle_t stop_timer_handle;

// --- Serial Communication Buffer ---
const int SERIAL_BUFFER_SIZE = 10;
byte serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// --- Global State Variables ---
volatile int32_t steering_angle = 90; // Updated by serial data
volatile int32_t received_speed = 0; // Updated by serial data (used to set RPM_setpoint)

// =========================================================================
// INTERRUPT SERVICE ROUTINE (ISR) - Hall Sensor / Encoder
// =========================================================================
void IRAM_ATTR hallSensorISR() {
    pulseCount++;
}

// =========================================================================
// FUNCTION: hb100_pcnt_init
// *** NEW: Initializes the Pulse Counter for the HB100 sensor ***
// =========================================================================
void hb100_pcnt_init() {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = HB100_PIN,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED, // We don't use a control pin
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT,
        .pos_mode = PCNT_COUNT_INC,    // Count rising edges
        .neg_mode = PCNT_COUNT_DIS,    // Don't count falling edges
        .lctrl_mode = PCNT_MODE_KEEP,  // Keep default control modes
        .hctrl_mode = PCNT_MODE_KEEP,
        // Set limits? Optional, can use for overflow detection if needed
         .counter_h_lim = INT16_MAX, // Example limit
         .counter_l_lim = INT16_MIN, // Example limit
    };
    pcnt_unit_config(&pcnt_config);

    // Configure PCNT filter (optional, but recommended)
    if (PCNT_FILTER_VAL > 0 && PCNT_FILTER_VAL <= 1023) {
        pcnt_set_filter_value(PCNT_UNIT, PCNT_FILTER_VAL);
        pcnt_filter_enable(PCNT_UNIT);
        Serial.printf("PCNT Filter enabled with value %d\n", PCNT_FILTER_VAL);
    } else {
        pcnt_filter_disable(PCNT_UNIT);
    }

    // Initialize PCNT's counter
    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
    Serial.println("HB100 PCNT Initialized.");
}


// =========================================================================
// TIMER CALLBACK - RPM Calculation
// =========================================================================
void IRAM_ATTR rpm_timer_callback(void *arg) {
    unsigned long currentTime_us = micros();
    unsigned long currentPulseReading;

    // Atomically read pulseCount
    // Using noInterrupts() is safest, but potentially adds slight latency.
    // Direct volatile read might be okay if acceptable for your accuracy needs.
    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();

    unsigned long deltaTime_us = currentTime_us - lastRpmCalcTime;
    unsigned long deltaPulses = currentPulseReading - lastRpmCalcPulseCount;

    float calculatedRPM = 0.0;
    // Calculate RPM only if time has passed and pulses were detected
    if (deltaTime_us > 1000 && deltaPulses > 0) { // Min 1ms interval, ensure pulses counted
        float pulses_per_second = (float)deltaPulses * 1000000.0 / (float)deltaTime_us;
        float rps = pulses_per_second / (float)PULSES_PER_REV;
        calculatedRPM = rps * 60.0;
    }
    // Consider adding logic here: If deltaPulses is 0 but setpoint > 0 for a while,
    // perhaps calculatedRPM should decay towards 0 or use last known good value?
    // Currently, if deltaPulses is 0, calculatedRPM remains 0 from initialization.

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

    // If the motor is flagged as running but pulses haven't changed in 1 sec
    if (motorRunning && currentPulseReading == lastCheckedPulseCount) {
        noInterrupts();
        currentRPM = 0.0; // Force measured RPM to zero
        // Also potentially reset EMA filter state if desired
        // g_filteredRPM = 0.0; // Reset filtered value too
        // Or re-seed EMA next time: updateEMA(0.0); // depends on EMA function design
        interrupts();
         // Serial.println("Stop detected!"); // Debug
    }
    lastCheckedPulseCount = currentPulseReading; // Update for next check
}

// =========================================================================
// TIMER CALLBACK - HB100 Velocity Calculation
// *** NEW: Reads PCNT and calculates velocity ***
// =========================================================================
void IRAM_ATTR hb100_timer_callback(void *arg) {
    int16_t count = 0;
    // Read PCNT value safely
    esp_err_t result = pcnt_get_counter_value(PCNT_UNIT, &count);

    if (result == ESP_OK) {
        // Clear counter immediately after reading for the next interval
        pcnt_counter_clear(PCNT_UNIT);

        // Calculate frequency based on counts over the known interval
        float frequency = (float)abs(count) * (1000000.0 / HB100_READ_INTERVAL_US); // Use abs(count) if neg_mode is used

        // Calculate velocity using Doppler formula (v = f * lambda / 2)
        float velocity_mps = (frequency * HB100_WAVELENGTH) / 2.0;

        // Convert m/s to km/h
        float velocity_kmh = velocity_mps * 3.6;

        // Store result in volatile global variable (atomic write for float on ESP32)
        hb100_velocity_kmh = velocity_kmh;

    } else {
        Serial.println("Error reading HB100 PCNT");
        hb100_velocity_kmh = -1.0; // Indicate error condition
    }
}


// =========================================================================
// FUNCTION: setMotorPWM
// Applies the given PWM value (0-255) to the motor driver.
// Includes basic current check.
// =========================================================================
void setMotorPWM(int pwmValue) {
    pwmValue = constrain(pwmValue, 0, 255);
    currentPWM = pwmValue; // Store the value being applied for display/debug

    // --- Basic Current Check (using voltage threshold) ---
    // !!! THIS NEEDS CALIBRATION based on your driver's mV output at max desired Amps !!!
    uint16_t raw_IS_R = analogRead(IS_R); // Only checking one side for simplicity here
    float voltage_IS_R_mV = raw_IS_R * (3300.0 / 4095.0);

    if (voltage_IS_R_mV < CURRENT_LIMIT_MV ) {
        digitalWrite(R_EN, HIGH); // Assuming EN pins enable the driver
        digitalWrite(L_EN, HIGH);
        // Assuming forward direction: LPWM controls speed, RPWM is LOW
        analogWrite(LPWM, pwmValue);
        digitalWrite(RPWM, LOW); // Ensure other PWM is off for simple forward
        motorRunning = (pwmValue > 10); // Update status (consider a small deadzone)
    } else {
        // Current limit exceeded! Stop the motor immediately
        Serial.printf("!!! CURRENT LIMIT EXCEEDED (%.1f mV) !!!\n", voltage_IS_R_mV);
        digitalWrite(R_EN, LOW);
        digitalWrite(L_EN, LOW);
        analogWrite(LPWM, 0);
        digitalWrite(RPWM, LOW);
        motorRunning = false;
        integral = 0; // Reset PID integral term on fault
        currentPWM = 0;
    }
}
// =========================================================================
// FUNCTION: updateEMA filter
// =========================================================================
float updateEMA(float currentRawValue) {
    // Static variables retain value between calls, initialized once.
    static float previousEMA = 0.0;
    static bool initialized = false;

    // Initialize the filter with the first non-zero reading
    if (!initialized) {
        if (abs(currentRawValue) > 0.01) { // Avoid initializing with transient zero
             previousEMA = currentRawValue;
             initialized = true;
        }
        // Return raw value until initialized to avoid returning 0 if first value is 0
        return currentRawValue;
    }

    // Apply the EMA formula
    float currentEMA = (EMA_ALPHA * currentRawValue) + ((1.0 - EMA_ALPHA) * previousEMA);

    // Update the state for the next call
    previousEMA = currentEMA;

    return currentEMA;
}
// =========================================================================
// FUNCTION: calculatePID_RPM_Output (NEW PID Logic)
// =========================================================================
float calculatePID_RPM_Output(float setpointRPM, float measuredRPM) {
    // Calculate Error (in RPM)
    float errorRPM = setpointRPM - measuredRPM;

    // Integral Term (with anti-windup clamping)
    float maxIntegralContribution_RPM = MAX_RPM_ESTIMATE * 1.1; // Allow integral to slightly exceed max RPM estimate
    float maxIntegral = (abs(Ki) > 1e-6) ? maxIntegralContribution_RPM / Ki : 1e9; // Avoid division by zero

    integral += errorRPM * PID_SAMPLE_TIME_S;
    integral = constrain(integral, -abs(maxIntegral), abs(maxIntegral)); // Clamp integral

    // Derivative Term
    float derivative = 0;
    if (PID_SAMPLE_TIME_S > 0) {
        derivative = (errorRPM - previousErrorRPM) / PID_SAMPLE_TIME_S;
    }

    // PID Calculation (Output is RPM-based effort)
    float p_term = Kp * errorRPM;
    float i_term = Ki * integral;
    float d_term = Kd * derivative;
    float pidOutput_RPM = p_term + i_term + d_term;

    // Update State for Next Iteration
    previousErrorRPM = errorRPM;

    // Basic anti-windup check based on output effort (optional addition)
    // If pidOutput_RPM is already demanding > max RPM, maybe clamp integral more?
    // (Clamping integral above is usually sufficient)

    return pidOutput_RPM;
}

// =========================================================================
// FUNCTION: transformRPMtoPWM (NEW Transformation Logic)
// =========================================================================
int transformRPMtoPWM(float targetEffort_RPM) {
    // Apply linear transformation: PWM = m * RPM + c
    float calculatedPWM_f = RPM_TO_PWM_SLOPE * targetEffort_RPM + RPM_TO_PWM_OFFSET;

    // Add feedforward based on setpoint (optional, helps response)
    // float feedforwardPWM = RPM_TO_PWM_SLOPE * RPM_setpoint + RPM_TO_PWM_OFFSET;
    // calculatedPWM_f += feedforwardPWM; // If using feedforward

    // Constrain the result to the valid PWM range
    int pwmOut = constrain((int)round(calculatedPWM_f), 0, 255);

    // Prevent applying tiny PWM values if setpoint is zero (helps stop cleanly)
    if (abs(RPM_setpoint) < 0.1 && pwmOut < (int)round(RPM_TO_PWM_OFFSET)+5) { // Threshold check
        pwmOut = 0;
    }
     // If setpoint is non-zero but calculated PWM is very low (below offset), maybe force minimum?
    // else if (abs(RPM_setpoint) > 0.1 && pwmOut < (int)round(RPM_TO_PWM_OFFSET)) {
    //    pwmOut = (int)round(RPM_TO_PWM_OFFSET); // Force minimum PWM to overcome stiction? Needs testing.
    // }


    return pwmOut;
}

// =========================================================================
// FUNCTION: control_servo
// Uses blocking delayMicroseconds - CONSIDER REPLACING with ESP32Servo library
// =========================================================================
void control_servo() {
    static unsigned long lastServoUpdate = 0;
    unsigned long now = micros(); // Use micros for servo timing

    // Refresh servo periodically based on REFRESH_INTERVAL
    if (now - lastServoUpdate >= REFRESH_INTERVAL) {
        lastServoUpdate = now;

        // Use volatile steering_angle directly, constrain it
        int constrainedAngle = constrain(steering_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);

        // Map angle to pulse width
        int pulseWidth = map(constrainedAngle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

        // Generate pulse (BLOCKING)
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(pulseWidth);
        digitalWrite(SERVO_PIN, LOW);
    }
}

// =========================================================================
// FUNCTION: parseSerialData
// Parses the fixed 10-byte packet: '<' angle(4) speed(4) '>'
// Updates RPM_setpoint and steering_angle globals.
// =========================================================================
void parseSerialData() {
    // Check start and end markers (already done partially in loop, defensive check)
    if (serialBuffer[0] == '<' && serialBuffer[SERIAL_BUFFER_SIZE - 1] == '>') {
        // Extract data assuming little-endian
        int32_t received_angle_local = *((int32_t*)&serialBuffer[1]);
        int32_t received_speed_local = *((int32_t*)&serialBuffer[5]);

        // Update global volatile variables
        // Direct assignment is generally atomic for int32_t on ESP32
        steering_angle = received_angle_local;
        received_speed = received_speed_local; // Store the raw received speed if needed elsewhere

        // *** Set the PID setpoint based on received speed ***
        RPM_setpoint = (float)received_speed_local;

    } else {
        Serial.println("Invalid serial packet format in parseSerialData.");
    }
    // Reset buffer index externally after calling this function
    // serialBufferIndex = 0; // Moved reset to the loop
}

// =========================================================================
// FUNCTION: displayData (Modified to show more info)
// =========================================================================
void displayData() {
    static unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL_MS = 250; // Print every 250ms

    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();

        // Read volatile globals safely (direct reads usually ok for atomic types)
        float rawRPM_display = currentRPM;
        float filteredRPM_display = g_filteredRPM;
        float setpoint_display = RPM_setpoint;
        int pwm_display = currentPWM; // Read last applied PWM
        float hb100_vel_display = hb100_velocity_kmh;
        int steer_display = steering_angle;

        // Print comma-separated values for easy plotting or parsing
        Serial.printf("SP:%.1f,RAW:%.1f,FILT:%.1f,PWM:%d,HB100:%.2f,STEER:%d\n",
                      setpoint_display,
                      rawRPM_display,
                      filteredRPM_display,
                      pwm_display,
                      hb100_vel_display,
                      steer_display);
    }
}

// =========================================================================
// SETUP FUNCTION
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Motor Control + HB100 Initializing...");

    // --- Pin Modes ---
    pinMode(SERVO_PIN, OUTPUT);
    digitalWrite(SERVO_PIN, LOW); // Ensure servo pin is low initially
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(IS_R, INPUT); // Analog input
    // pinMode(IS_L, INPUT); // Only if using IS_L
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
    pinMode(HB100_PIN, INPUT); // *** NEW: Set HB100 pin as input ***

    // --- Initial State ---
    digitalWrite(R_EN, LOW); // Disable driver
    digitalWrite(L_EN, LOW);
    analogWrite(LPWM, 0);    // Set PWM pins to 0
    digitalWrite(RPWM, LOW);
    setMotorPWM(0);          // Ensure motor state is off

    // --- Attach Encoder Interrupt ---
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), hallSensorISR, FALLING);
    Serial.println("Encoder ISR Attached.");

    // --- Initialize HB100 Pulse Counter ---
    hb100_pcnt_init(); // *** NEW ***

    // --- Initialize Timers & State ---
    lastRpmCalcTime = micros();
    lastPIDRunTime = millis();
    noInterrupts(); // Minimize time interrupts are disabled
    lastRpmCalcPulseCount = pulseCount;
    lastCheckedPulseCount = pulseCount;
    interrupts();
    Serial.println("State variables initialized.");

    // --- Setup ESP32 Timers ---
    // RPM Calculation Timer
    esp_timer_create_args_t rpm_timer_args = { .callback = &rpm_timer_callback, .name = "rpm_calc", .dispatch_method = ESP_TIMER_ISR}; // Run callback from ISR context
    esp_timer_create(&rpm_timer_args, &rpm_timer_handle);
    esp_timer_start_periodic(rpm_timer_handle, RPM_CALC_INTERVAL_US);
    Serial.println("RPM Timer Started.");

    // Stop Detection Timer
    esp_timer_create_args_t stop_timer_args = { .callback = &stop_timer_callback, .name = "motor_stop_check", .dispatch_method = ESP_TIMER_ISR}; // Run callback from ISR context
    esp_timer_create(&stop_timer_args, &stop_timer_handle);
    esp_timer_start_periodic(stop_timer_handle, STOP_CHECK_INTERVAL_US);
    Serial.println("Stop Check Timer Started.");

    // HB100 Read Timer *** NEW ***
    esp_timer_create_args_t hb100_timer_args = { .callback = &hb100_timer_callback, .name = "hb100_read", .dispatch_method = ESP_TIMER_ISR}; // Run callback from ISR context
    esp_timer_create(&hb100_timer_args, &hb100_timer_handle);
    esp_timer_start_periodic(hb100_timer_handle, HB100_READ_INTERVAL_US);
    Serial.println("HB100 Read Timer Started.");

    Serial.println("Setup Complete. Waiting for serial data...");
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
    // --- Process Incoming Serial Data ---
    // Non-blocking read into buffer
    while (Serial.available() > 0 && serialBufferIndex < SERIAL_BUFFER_SIZE) {
        uint8_t byteRead = Serial.read();
        // Handle start marker condition
        if (serialBufferIndex == 0 && byteRead != '<') {
            continue; // Discard bytes until start marker
        }
        serialBuffer[serialBufferIndex++] = byteRead;

        // Check if full packet received
        if (serialBufferIndex == SERIAL_BUFFER_SIZE) {
            parseSerialData(); // Process the buffer, updates RPM_setpoint/steering_angle
            serialBufferIndex = 0; // Reset for next packet
        }
    }
    // Handle incomplete packet timeout? (Optional)


    // --- Get Filtered RPM ---
    // Get the latest raw RPM calculated by the timer callback
    float rawRPM;
    noInterrupts(); // Safely read volatile variable updated by timer ISR
    rawRPM = currentRPM;
    interrupts();
    // Update the global filtered RPM value using the EMA function
    g_filteredRPM = updateEMA(rawRPM); // Use the filtered value for PID


    // --- PID Control Loop (Runs periodically) ---
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDRunTime >= PID_INTERVAL_MS) {
        lastPIDRunTime = currentMillis; // Update time of this PID run

        // Reset integral if setpoint is zero
        if (abs(RPM_setpoint) < 0.1) {
            integral = 0.0;
            previousErrorRPM = 0.0; // Reset previous error as well
        }

        // Calculate desired control effort using PID (using filtered RPM)
        float targetEffort_RPM = calculatePID_RPM_Output(RPM_setpoint, g_filteredRPM); // Use filtered RPM

        // Transform the RPM-based effort into a PWM value
        int targetPWM = transformRPMtoPWM(targetEffort_RPM);

        // Apply the calculated PWM to the motor (includes current check)
        setMotorPWM(targetPWM);
    } // --- End of PID Control Loop ---


    // --- Servo Control ---
    // Call non-blocking servo control frequently
    control_servo(); // Still uses blocking delay! Replace if possible.


    // --- Display Data ---
    displayData(); // Print data periodically


    // Yield for other tasks (important if using FreeRTOS features implicitly)
    // delay(1); // Use delay(0) or vTaskDelay(1) if needed, avoid long delays
}

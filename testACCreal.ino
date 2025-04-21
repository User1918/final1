#include <Arduino.h>
#include <math.h>
#include "esp_timer.h" // For ESP32 specific timer
#include <driver/pcnt.h> // *** NEW: Include for Pulse Counter ***

// --- Pin Definitions ---
// ... (keep existing motor/servo pins) ...
#define HB100_PIN   GPIO_NUM_14 // *** NEW: Choose an available GPIO for HB100 input ***

// --- Hall Sensor / Encoder ---
// ... (keep existing encoder definitions) ...
const int hallSensorPin = 12; // Keep this if still used by interrupt method

// --- RPM Calculation via Timer ---
// ... (keep existing RPM timer definitions) ...

// --- HB100 Doppler Sensor ---
#define PCNT_UNIT           PCNT_UNIT_0 // *** NEW: Choose a PCNT unit (0-7) ***
#define HB100_WAVELENGTH    0.0285 // *** NEW: Approx. wavelength in meters for 10.525 GHz ***
#define HB100_READ_INTERVAL_MS 200 // *** NEW: How often to calculate HB100 velocity (ms) ***
#define HB100_READ_INTERVAL_US (HB100_READ_INTERVAL_MS * 1000)
volatile float hb100_velocity_kmh = 0.0; // *** NEW: Stores calculated velocity ***
esp_timer_handle_t hb100_timer_handle;  // *** NEW: Timer for reading PCNT ***
// PCNT filter value - helps debounce noisy signals (adjust 0-1023)
#define PCNT_FILTER_VAL 100 // *** NEW: Example filter value ***

// --- Motor Control & Setpoint ---
// ... (keep existing motor control definitions) ...
// !!! SUGGESTION: Use the speed received from Pi5 as the setpoint? !!!
// float RPM_setpoint = 0; // Keep if set manually, or assign from speed_motor later

// --- PID Controller Coefficients ---
// ... (keep existing PID definitions) ...

// --- PID Timing & State Variables ---
// ... (keep existing PID variables) ...

// --- Timer for Stop Detection ---
// ... (keep existing stop timer definitions) ...

// --- Global variables from serial ---
volatile int32_t steering_angle = 90; // Default center
volatile int32_t speed_motor = 0;     // Received motor command (RPM?)


// =========================================================================
// INTERRUPT SERVICE ROUTINE (ISR) - Hall Sensor / Encoder
// (Keep this if you still need the interrupt method for the encoder)
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
        // Set limits? Not strictly necessary if we clear periodically
        // .counter_h_lim = INT16_MAX,
        // .counter_l_lim = INT16_MIN,
    };
    pcnt_unit_config(&pcnt_config);

    // Configure PCNT filter (optional, but recommended)
    if (PCNT_FILTER_VAL > 0) {
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
// FUNCTION: setMotorPWM
// !!! ATTENTION: Corrected current calculation and PWM variable usage !!!
// =========================================================================
void setMotorPWM(int targetPWM) { // Changed argument name for clarity
    // --- Read Current Sensors ---
    // Read raw ADC values
    int raw_IS_R = analogRead(IS_R);
    int raw_IS_L = analogRead(IS_L);

    // --- Convert ADC to Millivolts ---
    // Use floating point for accuracy!
    float voltage_IS_R = raw_IS_R * (3300.0 / 4095.0);
    float voltage_IS_L = raw_IS_L * (3300.0 / 4095.0);

    // --- Convert Voltage to Milliamps (NEEDS CALIBRATION / DATASHEET) ---
    // This is an EXAMPLE for BTS7960, check YOUR board's datasheet!
    // BTS7960 formula might be: I_mA = (voltage_IS_mV / 8.5) for R_IS=1k
    // Or I_load = V_IS * (I_is_norm / V_is_norm) = V_IS * (8500 A / 1 V) -> mA = mV * 8.5? Check!
    // Placeholder - replace with your actual conversion factor K_IS (mA/mV)
    float K_IS = 8.5; // Example mA/mV - ** FIND THE CORRECT VALUE **
    float currentR_mA = voltage_IS_R * K_IS;
    float currentL_mA = voltage_IS_L * K_IS;

    // --- Current Limit Check and Motor Control ---
    if (currentR_mA < CURRENT_LIMIT && currentL_mA < CURRENT_LIMIT) {
        digitalWrite(R_EN, HIGH);
        digitalWrite(L_EN, HIGH);
        // Assuming forward direction only for simplicity
        analogWrite(LPWM, targetPWM); // Use the passed PWM value
        analogWrite(RPWM, 0);
        motorRunning = (targetPWM > 0); // Update motor running status
    } else {
        // Overcurrent detected! Stop the motor.
        digitalWrite(R_EN, LOW);
        digitalWrite(L_EN, LOW);
        analogWrite(LPWM, 0);
        analogWrite(RPWM, 0);
        motorRunning = false;
        // Optional: Print a warning
        // Serial.println("!!! OVERCURRENT DETECTED !!!");
    }
}

// =========================================================================
// Receive Function from pi5
// (Corrected to match buffer size and casting)
// =========================================================================
void receiveData() {
    static uint8_t buffer[10];
    static int index = 0;

    while (Serial.available()) {
        uint8_t byteRead = Serial.read();

        // Look for start character '<' only at the beginning
        if (index == 0) {
            if (byteRead == '<') {
                buffer[index++] = byteRead;
            } else {
                // Ignore bytes until start character is found
            }
        } else {
            // Store subsequent bytes
            buffer[index++] = byteRead;

            // Check if buffer is full (received 10 bytes)
            if (index == 10) {
                // Check for end character '>'
                if (buffer[9] == '>') {
                    // Extract data (assuming little-endian)
                    int32_t received_angle = *((int32_t*)&buffer[1]);
                    int32_t received_speed = *((int32_t*)&buffer[5]);

                    // Update global volatile variables safely
                    // (Direct assignment is usually atomic for int32_t on ESP32)
                    steering_angle = received_angle;
                    speed_motor = received_speed; // This is likely intended as RPM setpoint

                    // Optional: Print received values for debugging
                    // Serial.printf("Received Angle: %d, Speed: %d\n", steering_angle, speed_motor);

                } else {
                    // End character mismatch, invalid packet
                     Serial.println("Invalid end char");
                }
                // Reset index regardless of end character match to start looking for next packet
                index = 0;
            }
        }
         // Prevent buffer overflow if start char found but end char never comes
        if (index >= 10) {
            index = 0;
             Serial.println("Buffer overflow/reset");
        }
    }
}


// =========================================================================
// TIMER CALLBACK - RPM Calculation
// (No changes needed here)
// =========================================================================
void rpm_timer_callback(void *arg) {
    // ... (keep existing code) ...
     unsigned long currentTime_us = micros();
    unsigned long currentPulseReading;

    // Temporarily disable interrupts to safely read pulseCount
    // Consider if this is truly needed or if atomic read is sufficient
    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();

    unsigned long deltaTime_us = currentTime_us - lastRpmCalcTime;
    unsigned long deltaPulses = currentPulseReading - lastRpmCalcPulseCount;

    float calculatedRPM = 0.0;
    // Prevent division by zero and ensure reasonable time has passed
    if (deltaTime_us > 1000) { // e.g., require at least 1ms to avoid noise
        float pulses_per_second = (float)deltaPulses * 1000000.0 / (float)deltaTime_us;
        float rps = pulses_per_second / (float)PULSES_PER_REV;
        calculatedRPM = rps * 60.0;
    }

    // Safely update global volatile variable
    noInterrupts();
    currentRPM = calculatedRPM;
    interrupts();

    // Update state for next calculation
    lastRpmCalcTime = currentTime_us;
    lastRpmCalcPulseCount = currentPulseReading;
}

// =========================================================================
// FUNCTION: calculatePID
// (No changes needed here, assuming RPM_setpoint is updated elsewhere)
// =========================================================================
int calculatePID(float currentSetpoint_rpm, float currentMeasurement_rpm) { // Changed arg names
    // --- Calculate Error (in RPM) ---
    // Consider if error transformation is needed if input/output are RPM now
    float rpm_error = currentSetpoint_rpm - currentMeasurement_rpm;

    // --- Integral Term ---
    integral += rpm_error * PID_SAMPLE_TIME_S;
    // Optional: Add Anti-windup

    // --- Derivative Term ---
    float derivative = 0;
    if (PID_SAMPLE_TIME_S > 0) {
        derivative = (rpm_error - previousError) / PID_SAMPLE_TIME_S;
    }

    // --- PID Calculation ---
    float pwmOutput = Kp * rpm_error + Ki * integral + Kd * derivative;

    // --- Update State ---
    previousError = rpm_error;

    // --- Constrain and Return PWM ---
    return constrain((int)round(pwmOutput), 0, 255); // Use round() for better conversion
}


// =========================================================================
// TIMER CALLBACK - Stop Detection
// (No changes needed here)
// =========================================================================
void stop_timer_callback(void *arg) {
   // ... (keep existing code) ...
     unsigned long currentPulseReading;

    noInterrupts();
    currentPulseReading = pulseCount;
    interrupts();

    if (motorRunning && currentPulseReading == lastCheckedPulseCount) {
        // Motor commanded to run, but no pulses detected for STOP_CHECK_INTERVAL_US
        noInterrupts();
        currentRPM = 0.0; // Force measured RPM to zero
        interrupts();
        // Serial.println("Motor stop detected!"); // Optional debug
    }
    lastCheckedPulseCount = currentPulseReading;
}

// =========================================================================
// TIMER CALLBACK - HB100 Velocity Calculation
// *** NEW: Reads PCNT and calculates velocity ***
// =========================================================================
void hb100_timer_callback(void *arg) {
    int16_t count = 0;
    esp_err_t result = pcnt_get_counter_value(PCNT_UNIT, &count); // Read PCNT value

    if (result == ESP_OK) {
        pcnt_counter_clear(PCNT_UNIT); // Clear counter for the next interval

        // Calculate frequency
        float frequency = (float)count * (1000000.0 / HB100_READ_INTERVAL_US);

        // Calculate velocity using Doppler formula (v = f * lambda / 2)
        float velocity_mps = (frequency * HB100_WAVELENGTH) / 2.0;

        // Convert m/s to km/h
        float velocity_kmh = velocity_mps * 3.6;

        // Store result in volatile global variable
        // (Direct assignment usually atomic for float on ESP32)
        hb100_velocity_kmh = velocity_kmh;

    } else {
        Serial.println("Error reading PCNT");
        hb100_velocity_kmh = -1.0; // Indicate error
    }
}


//==========================================================================
// STEERING FUNC
// !!! ATTENTION: Using delayMicroseconds() blocks! Consider ESP32Servo library !!!
//==========================================================================
void control_servo() {
    int targetAngle = steering_angle; // Use the volatile global directly

    // Clamp angle
    if (targetAngle < 40) targetAngle = 40;
    if (targetAngle > 130) targetAngle = 130;

    // --- Blocking Software PWM - Consider replacing ---
    int pulseWidth = map(targetAngle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    digitalWrite(SERVO_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(SERVO_PIN, LOW);
    // The second delay isn't strictly needed here if called frequently enough from loop
    // delayMicroseconds(REFRESH_INTERVAL - pulseWidth);
    // --- End Blocking Section ---

    /* --- SUGGESTED REPLACEMENT using ESP32Servo library ---
    #include <ESP32Servo.h>
    Servo myServo; // Declare globally or in setup

    void setup() {
        // ... other setup ...
        myServo.attach(SERVO_PIN);
        // Optional: myServo.setPeriodHertz(50); // Standard 50Hz servo rate
    }

    void control_servo_non_blocking() {
        int targetAngle = steering_angle;
        if (targetAngle < 40) targetAngle = 40;
        if (targetAngle > 130) targetAngle = 130;
        myServo.write(targetAngle); // Let the library handle PWM
    }
    // Call control_servo_non_blocking() in loop instead
    */
}

// =========================================================================
// SETUP FUNCTION
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Setup Starting...");

    pinMode(SERVO_PIN, OUTPUT); // Keep for basic digitalWrite, needed even for ESP32Servo attach

    // Motor pins
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    pinMode(IS_R, INPUT); // Analog input, no pinMode needed but doesn't hurt
    pinMode(IS_L, INPUT); // Analog input

    digitalWrite(R_EN, LOW); // Ensure motors are disabled initially
    digitalWrite(L_EN, LOW);
    analogWrite(LPWM, 0);
    analogWrite(RPWM, 0);

    // Encoder Interrupt Pin (if still using interrupt method)
    pinMode(hallSensorPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(hallSensorPin), hallSensorISR, FALLING);

    // --- Initialize HB100 Pulse Counter ---
    hb100_pcnt_init(); // *** NEW ***

    // --- Initialize Timers & State ---
    lastRpmCalcTime = micros();
    noInterrupts(); // Minimize time interrupts are disabled
    lastRpmCalcPulseCount = pulseCount;
    lastCheckedPulseCount = pulseCount;
    interrupts();

    // RPM Timer Setup
    esp_timer_create_args_t rpm_timer_args = { .callback = &rpm_timer_callback, .name = "rpm_calc"};
    esp_timer_create(&rpm_timer_args, &rpm_timer_handle);
    esp_timer_start_periodic(rpm_timer_handle, RPM_CALC_INTERVAL_US);
    Serial.println("RPM Timer Started.");

    // Stop Detection Timer Setup
    esp_timer_create_args_t stop_timer_args = { .callback = &stop_timer_callback, .name = "motor_stop_check"};
    esp_timer_create(&stop_timer_args, &stop_timer_handle);
    esp_timer_start_periodic(stop_timer_handle, STOP_CHECK_INTERVAL_US);
    Serial.println("Stop Check Timer Started.");

     // HB100 Read Timer Setup *** NEW ***
    esp_timer_create_args_t hb100_timer_args = { .callback = &hb100_timer_callback, .name = "hb100_read"};
    esp_timer_create(&hb100_timer_args, &hb100_timer_handle);
    esp_timer_start_periodic(hb100_timer_handle, HB100_READ_INTERVAL_US);
    Serial.println("HB100 Read Timer Started.");

    Serial.println("Setup Complete.");
}

// =========================================================================
// MAIN LOOP
// =========================================================================
void loop() {
    unsigned long currentMillis = millis();

    // 1. Receive data from Pi5 (non-blocking)
    receiveData();

    // 2. Control Servo (Consider non-blocking version)
    control_servo(); // Call the servo control function

    // 3. Run PID Controller periodically
    if (currentMillis - lastPIDRunTime >= PID_INTERVAL_MS) {
        lastPIDRunTime = currentMillis; // Update time of this PID run

        // --- Get current speed measurement ---
        float measuredRPM;
        // Reading volatile float is usually atomic, maybe skip noInterrupts here?
        // noInterrupts();
        measuredRPM = currentRPM;
        // interrupts();

        // --- Use received speed as setpoint (or keep manual RPM_setpoint) ---
        // !!! ASSUMPTION: speed_motor from Pi5 is the desired RPM !!!
        float current_rpm_setpoint = (float)speed_motor;

        // --- Calculate PID output ---
        int targetPWM = calculatePID(current_rpm_setpoint, measuredRPM);

        // --- Apply PWM to motor ---
        setMotorPWM(targetPWM); // Pass the calculated PWM
    }

    // 4. Display Data periodically (non-blocking)
    displayData();

    // Optional: Small delay to prevent loop from running too fast if nothing else happens
    // delay(1); // Use with caution, can affect timing if too long
}

// =========================================================================
// FUNCTION: displayData
// (Corrected to show target PWM, added HB100 velocity)
// =========================================================================
void displayData() {
    static unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL_MS = 200; // Print every 200ms

    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();

        // Read volatile globals safely (usually atomic reads for float/long on ESP32)
        float measuredRPM = currentRPM;
        float targetRPM = (float)speed_motor; // Assuming speed_motor is the target
        float current_hb100_vel = hb100_velocity_kmh;

        Serial.printf("Target RPM: %.2f | Measured RPM: %.2f | HB100 Vel (km/h): %.2f | Steer: %d\n",
                      targetRPM,
                      measuredRPM,
                      current_hb100_vel,
                      steering_angle); // steering_angle is volatile, read directly
    }
}
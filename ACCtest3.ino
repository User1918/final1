#include <Arduino.h>
#include <math.h>
#include "esp_timer.h" // For ESP32 specific timer

// --- Pin Definitions ---
#define RPWM 2  // Right PWM (currently unused for PWM, set LOW)
#define LPWM 15 // Left PWM (used for motor speed control)
#define R_EN 0  // Right Enable
#define L_EN 4  // Left Enable

// --- Hall Sensor ---
const int hallSensorPin = 12;
const int PULSES_PER_REV = 5; // Number of pulses per motor revolution
volatile unsigned long pulseCount = 0;
volatile unsigned long lastPulseTime = 0; // Timestamp of the last pulse in microseconds
volatile float omega_out_motor = 0;      // Measured motor angular velocity (radians/second)

// --- Motor Control & Setpoint ---
float omega_setpoint = 100.0; // Desired motor angular velocity (radians/second) - Can be changed
int currentPWM = 0;         // The PWM value currently being applied
bool motorRunning = false;    // Flag indicating if the motor PWM is active

// --- Linear Feedforward Constants (PWM = a * omega + b) ---
// These constants estimate the PWM needed for a given omega without PID
const float a = 80.0 / 339.0;
const float b = 180.0 / 113.0;

// --- PID Controller Coefficients ---
float Kp = 1.0; // Proportional gain (Reacts to current error)
float Ki = 0.5; // Integral gain (Eliminates steady-state error) - TUNED SLIGHTLY
float Kd = 0.01; // Derivative gain (Dampens oscillations, sensitive to noise) - TUNED SLIGHTLY

// --- PID State Variables ---
float previousError = 0; // Error from the previous PID calculation
float integral = 0;      // Integral accumulator
unsigned long lastPIDCalculationTime = 0; // Timestamp of the last PID calculation

// --- Timer for Stop Detection ---
const int STOP_CHECK_INTERVAL_US = 100000; // Check every 100ms (0.1 seconds) if motor stopped
volatile unsigned long lastCheckedPulseCount = 0; // Pulse count at the last timer check

// --- PID Calculation Interval ---
const unsigned long PID_INTERVAL_MS = 10; // Calculate PID every 10 milliseconds

// =========================================================================
// INTERRUPT SERVICE ROUTINE (ISR) - Hall Sensor
// Calculates motor speed based on time between full rotations
// =========================================================================
void IRAM_ATTR hallSensorISR() {
    unsigned long currentTime = micros(); // Get current time in microseconds
    pulseCount++;

    // Calculate omega every full revolution for better accuracy
    static unsigned long lastFullRotationTime = 0;
    if (pulseCount % PULSES_PER_REV == 0) {
        unsigned long deltaT_us = currentTime - lastFullRotationTime;
        if (deltaT_us > 0) {
            // Calculate omega: (2 * PI radians) / (time_for_one_rev_in_seconds)
            omega_out_motor = (2.0 * PI) / (deltaT_us / 1000000.0);
        } else {
             // Avoid division by zero if time difference is somehow zero
            omega_out_motor = 0;
        }
        lastFullRotationTime = currentTime; // Update time for the start of the next revolution
    }
    lastPulseTime = currentTime; // Store time of the *last* pulse detected
}

// =========================================================================
// FUNCTION: setMotorPWM
// Applies the calculated PWM value to the motor driver pins
// =========================================================================
void setMotorPWM(int pwm) {
    // Ensure PWM is within valid range (0-255 for ESP32 analogWrite)
    int constrainedPWM = constrain(pwm, 0, 255);

    // Assuming motor runs in one direction using LPWM
    analogWrite(LPWM, constrainedPWM); // Set PWM duty cycle
    digitalWrite(RPWM, LOW);          // Keep other direction pin LOW

    currentPWM = constrainedPWM; // Store the applied PWM value
    motorRunning = (constrainedPWM > 0); // Update motor running status
}

// =========================================================================
// FUNCTION: calculatePID
// Calculates the required PWM adjustment based on the error
// Returns the calculated PWM value (0-255)
// =========================================================================
int calculatePID(float currentSetpoint, float currentMeasurement) {
    unsigned long currentTime = micros();
    // Calculate time elapsed since last PID calculation (in seconds)
    float deltaTime = (currentTime - lastPIDCalculationTime) / 1000000.0;

    // --- Prevent issues on first run or large gaps ---
    // If deltaTime is 0 or too large (e.g., > 0.5 sec), reset/skip
    if (deltaTime <= 0 || deltaTime > 0.5) {
        deltaTime = PID_INTERVAL_MS / 1000.0; // Use default interval time
        integral = 0; // Reset integral to prevent windup after a pause
        // Consider resetting previousError = 0; as well if needed
    }
    lastPIDCalculationTime = currentTime; // Update timestamp for the next calculation

    // --- Calculate Error ---
    float W_error = currentSetpoint - currentMeasurement; // Raw error in rad/s

    // --- Optional: Linear transformation of error (from original code) ---
    // This step scales the error based on 'a' and adds an offset 'b'.
    // It's unusual in standard PID, but kept as per original logic.
    // Normally, PID works directly on W_error. If control is poor,
    // consider removing this transformation and retuning Kp, Ki, Kd.
    float error = a * W_error + b;

    // --- Integral Term ---
    integral += error * deltaTime;
    // Optional: Add Anti-windup for Integral term
    // float maxIntegral = 200.0; // Example limit, tune as needed
    // integral = constrain(integral, -maxIntegral, maxIntegral);

    // --- Derivative Term ---
    float derivative = 0;
    if (deltaTime > 0) {
        derivative = (error - previousError) / deltaTime;
    }

    // --- PID Calculation ---
    // Calculates the total required PWM output
    float pwmOutput = Kp * error + Ki * integral + Kd * derivative;

    // --- Update State for Next Iteration ---
    previousError = error;

    // --- Constrain and Return PWM ---
    // The PID output is treated as the target PWM value
    int targetPWM = constrain((int)pwmOutput, 0, 255);

    return targetPWM;
}


// =========================================================================
// TIMER CALLBACK - Stop Detection
// Periodically checks if the pulse count has stopped changing
// =========================================================================
void timer_callback(void *arg) {
    noInterrupts(); // Read volatile variable safely
    unsigned long currentPulseCount = pulseCount;
    interrupts();

    // If the motor was supposed to be running but pulse count hasn't changed,
    // assume it has stopped or stalled.
    if (motorRunning && currentPulseCount == lastCheckedPulseCount) {
        noInterrupts();
        omega_out_motor = 0; // Set measured speed to zero
        // Optional: Reset PID state when motor stops unexpectedly
        // integral = 0;
        // previousError = 0;
        interrupts();

         // Optional: If setpoint is also 0, explicitly stop PWM
         // if (omega_setpoint <= 0) {
         //    setMotorPWM(0);
         // }
    }
    lastCheckedPulseCount = currentPulseCount; // Update for the next check
}

// =========================================================================
// SETUP FUNCTION
// Initializes hardware, serial communication, interrupts, and timers
// =========================================================================
void setup() {
    Serial.begin(115200);
    Serial.println("Starting Motor PID Control");

    // --- Initialize Motor Driver Pins ---
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(R_EN, OUTPUT);
    pinMode(L_EN, OUTPUT);
    digitalWrite(R_EN, HIGH); // Enable driver
    digitalWrite(L_EN, HIGH); // Enable driver
    setMotorPWM(0); // Start with motor off

    // --- Initialize Hall Sensor Pin & Interrupt ---
    pinMode(hallSensorPin, INPUT_PULLUP); // Use internal pull-up resistor
    attachInterrupt(digitalPinToInterrupt(hallSensorPin), hallSensorISR, FALLING); // Trigger on falling edge

    // --- Initialize Timers ---
    lastPIDCalculationTime = micros(); // Initialize PID timer baseline

    // Configure and start the periodic timer for stop detection
    esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "motor_stop_check" // Descriptive name
    };
    esp_timer_handle_t timer_handle;
    esp_timer_create(&timer_args, &timer_handle);
    esp_timer_start_periodic(timer_handle, STOP_CHECK_INTERVAL_US);

    Serial.println("Setup complete. Running PID loop.");
    Serial.print("Initial Omega Setpoint (rad/s): ");
    Serial.println(omega_setpoint);
     Serial.println("Kp Ki Kd");
    Serial.print(Kp);Serial.print(" ");Serial.print(Ki);Serial.print(" ");Serial.println(Kd);
}

// =========================================================================
// MAIN LOOP
// Periodically calculates PID output and displays data
// =========================================================================
void loop() {
    // --- Process Serial Input (Optional) ---
    // processInput(); // Uncomment this line to enable changing setpoint via Serial

    // --- Run PID Calculation at Regular Intervals ---
    unsigned long currentMillis = millis();
    if (currentMillis - lastPIDRunTime >= PID_INTERVAL_MS) {
        lastPIDRunTime = currentMillis; // Update time of this PID run

        noInterrupts(); // Safely read shared variable
        float currentOmega = omega_out_motor;
        interrupts();

        // Calculate the target PWM using the PID function
        int targetPWM = calculatePID(omega_setpoint, currentOmega);

        // Apply the calculated PWM to the motor
        setMotorPWM(targetPWM);
    }

    // --- Display Data Periodically ---
    displayData();
}

// =========================================================================
// FUNCTION: processInput (Optional)
// Reads new omega setpoint from Serial Monitor
// =========================================================================
void processInput() {
    static String inputString = ""; // Holds incoming characters
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') { // Check if newline character (end of input)
            if (inputString.length() > 0) {
                float newSetpoint = inputString.toFloat();
                if (newSetpoint >= 0) { // Basic validation
                    omega_setpoint = newSetpoint;
                    Serial.print("New Omega Setpoint (rad/s): ");
                    Serial.println(omega_setpoint);
                    // Reset PID state when setpoint changes significantly might be good
                    integral = 0;
                    previousError = 0; // Start fresh with the new target
                } else {
                    Serial.println("Invalid setpoint (must be >= 0).");
                }
                inputString = ""; // Clear the string for next input
            }
        } else if (isDigit(inChar) || inChar == '.' || (inChar == '-' && inputString.length() == 0)) {
             // Allow digits, decimal point, and leading minus sign
            inputString += inChar;
        }
         // Ignore other characters
    }
}

// =========================================================================
// FUNCTION: displayData
// Prints current setpoint, measured omega, and PWM to Serial Monitor
// =========================================================================
void displayData() {
    static unsigned long lastPrintTime = 0;
    const unsigned long PRINT_INTERVAL_MS = 200; // Print data every 200ms

    if (millis() - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = millis();

        noInterrupts(); // Safely read volatile variable
        float currentOmegaOutMotor = omega_out_motor;
        interrupts();

        // Read current PWM (not volatile, but good practice if changed elsewhere)
        int currentPWMOut = currentPWM;

        //Serial.print("Setpoint (rad/s): ");
        Serial.print(omega_setpoint);
        Serial.print(" "); // Tab separation for easier plotting/parsing
        //Serial.print("Measured (rad/s): ");
        Serial.print(currentOmegaOutMotor);
        Serial.print(" "); // Tab separation
        //Serial.print("PWM: ");
        Serial.println(currentPWMOut);
    }
}


// --- Removed calculateInitialPWM function ---
// The initialPWM calculation based on the linear model is not directly used
// when the PID controller calculates the full output PWM.
// If you wanted feedforward, you would typically calculate initialPWM
// and *add* the PID adjustment to it, rather than having the PID calculate
// the entire PWM value.
// ESP32 Code (Arduino IDE)
// Receives variable steering angle and motor speed from Raspberry Pi via USB serial
#include <Arduino.h>
#include <HardwareSerial.h>

// --- Configuration ---
HardwareSerial& SerialPort = Serial; // Use Serial for USB serial

int steeringAngle = 0;
int motorSpeed = 0;

void setup() {
  SerialPort.begin(115200);
  SerialPort.println("ESP32 Serial Receiver Started");
}

void loop() {
  if (SerialPort.available() > 0) {
    String dataString = SerialPort.readStringUntil('\n');
    dataString.trim(); // Remove any leading/trailing whitespace

    int commaIndex = dataString.indexOf(',');
    if (commaIndex != -1) {
      String steeringStr = dataString.substring(0, commaIndex);
      String speedStr = dataString.substring(commaIndex + 1);

      steeringAngle = steeringStr.toInt();
      motorSpeed = speedStr.toInt();

      SerialPort.print("Received: Steering=");
      SerialPort.print(steeringAngle);
      SerialPort.print(", Speed=");
      SerialPort.println(motorSpeed);

      // Use the variable 'steeringAngle' and 'motorSpeed' to control your system
      // For example, mapping these values to motor PWM or servo positions:
      // int steeringPWM = map(steeringAngle, -45, 45, 0, 255);
      // int motorPWM = map(motorSpeed, 0, 100, 0, 255);
      // analogWrite(steeringControlPin, steeringPWM);
      // analogWrite(motorSpeedPin, motorPWM);
    } else {
      SerialPort.println("Invalid data format received.");
    }
  }
  delay(10); // Small delay
}

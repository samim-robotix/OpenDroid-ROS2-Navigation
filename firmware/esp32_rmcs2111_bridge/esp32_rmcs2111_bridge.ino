/**
 * @file esp32_rmcs2111_bridge.ino
 * @brief OpenDroid Embedded Motor Controller Bridge for ESP32 & Rhino RMCS-2111
 * @author OpenDroid Robotics Team
 * 
 * Hardware Architecture:
 * - MCU: ESP32 DevKit V1 (Dual Core 240MHz, 3.3V Logic)
 * - Motor Drivers: 2x Rhino RMCS-2111 Heavy Duty Planetary DC Geared Motor Drivers
 * - Communication Protocols:
 *     - Host SBC (Jetson/Raspberry Pi) <-> ESP32: High-Speed UART / USB-CDC (115200 baud) or micro-ROS
 *     - ESP32 <-> Left RMCS-2111 Driver: Serial2 (GPIO 16 RX2, GPIO 17 TX2)
 *     - ESP32 <-> Right RMCS-2111 Driver: Serial1 / SoftwareSerial / I2C bus
 * - Power: 12V LiFePO4 / Lead-Acid Battery with common GND reference
 */

#include <Arduino.h>

// ---------------------- PIN DEFINITIONS & CONSTANTS -------------------------
// Left Motor Driver (RMCS-2111) UART Interface
#define LEFT_MOTOR_RX 16   // ESP32 RX2 <- RMCS-2111 TXD (Orange)
#define LEFT_MOTOR_TX 17   // ESP32 TX2 -> RMCS-2111 RXD (Yellow)

// Right Motor Driver (RMCS-2111) UART Interface
#define RIGHT_MOTOR_RX 18  // ESP32 RX1 <- RMCS-2111 TXD (Orange)
#define RIGHT_MOTOR_TX 19  // ESP32 TX1 -> RMCS-2111 RXD (Yellow)

// Robot Kinematic Parameters (from bcr_bot.xacro)
const float WHEEL_RADIUS = 0.10;       // meters (200mm diameter)
const float TRACK_WIDTH  = 0.60;       // meters (600mm center-to-center)
const float MAX_RPM      = 66.0;       // Maximum rated output RPM
const float GEAR_RATIO   = 270.0;      // 1:270 Planetary reduction

// HardwareSerial Ports
HardwareSerial LeftMotorSerial(2);
HardwareSerial RightMotorSerial(1);

// State variables
float target_linear_x = 0.0;   // m/s
float target_angular_z = 0.0;  // rad/s
unsigned long last_cmd_time = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 500; // Emergency stop if no command received

// ---------------------- FUNCTION PROTOTYPES ---------------------------------
void setMotorSpeed(HardwareSerial &serialPort, int16_t speedRpm, bool forward);
void processHostCommand(String cmd);
void emergencyStop();

void setup() {
  // Initialize Host SBC Telemetry (USB-Serial)
  Serial.begin(115200);
  delay(500);
  Serial.println("[OpenDroid] Firmware Initializing...");

  // Initialize Motor Driver Serial Channels
  LeftMotorSerial.begin(9600, SERIAL_8N1, LEFT_MOTOR_RX, LEFT_MOTOR_TX);
  RightMotorSerial.begin(9600, SERIAL_8N1, RIGHT_MOTOR_RX, RIGHT_MOTOR_TX);

  // Arm motors in neutral
  emergencyStop();
  Serial.println("[OpenDroid] Dual RMCS-2111 Drivers Armed & Online.");
}

void loop() {
  // Read Serial commands from Host ROS 2 Serial Bridge (e.g. "V,linear_x,angular_z\n")
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      processHostCommand(input);
      last_cmd_time = millis();
    }
  }

  // Safety Watchdog: Auto-brake if communication is lost
  if (millis() - last_cmd_time > WATCHDOG_TIMEOUT_MS) {
    emergencyStop();
  }

  delay(10);
}

void processHostCommand(String cmd) {
  // Command protocol format: V,<linear_m_s>,<angular_rad_s>
  if (cmd.startsWith("V,")) {
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    if (secondComma > 0) {
      target_linear_x = cmd.substring(firstComma + 1, secondComma).toFloat();
      target_angular_z = cmd.substring(secondComma + 1).toFloat();

      // Differential Drive Kinematics
      float v_left  = target_linear_x - (target_angular_z * TRACK_WIDTH / 2.0);
      float v_right = target_linear_x + (target_angular_z * TRACK_WIDTH / 2.0);

      // Convert linear m/s to wheel RPM: RPM = (v / (2 * pi * R)) * 60
      float rpm_left  = (v_left / (2.0 * 3.14159 * WHEEL_RADIUS)) * 60.0;
      float rpm_right = (v_right / (2.0 * 3.14159 * WHEEL_RADIUS)) * 60.0;

      // Clamp to max rated RPM
      rpm_left  = constrain(rpm_left, -MAX_RPM, MAX_RPM);
      rpm_right = constrain(rpm_right, -MAX_RPM, MAX_RPM);

      // Dispatch to RMCS-2111 drivers
      setMotorSpeed(LeftMotorSerial, abs((int16_t)rpm_left), rpm_left >= 0);
      setMotorSpeed(RightMotorSerial, abs((int16_t)rpm_right), rpm_right >= 0);

      // Telemetry echo back to host
      Serial.printf("[STATUS] L_RPM: %.2f | R_RPM: %.2f\n", rpm_left, rpm_right);
    }
  } else if (cmd.equalsIgnoreCase("STOP")) {
    emergencyStop();
  }
}

void setMotorSpeed(HardwareSerial &serialPort, int16_t speedRpm, bool forward) {
  // Protocol for Rhino RMCS-2111 (UART ASCII format or speed packet)
  // Format example: S,<speed_rpm>,<dir>\r
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "S,%d,%d\r", speedRpm, forward ? 1 : 0);
  serialPort.print(buffer);
}

void emergencyStop() {
  target_linear_x = 0.0;
  target_angular_z = 0.0;
  setMotorSpeed(LeftMotorSerial, 0, true);
  setMotorSpeed(RightMotorSerial, 0, true);
}

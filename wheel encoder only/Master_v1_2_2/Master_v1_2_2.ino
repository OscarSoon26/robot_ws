// =============================================================================
// 1. MASTER BOARD CODE (ESP32-S3 | ID: 0x100 | TX: GPIO 48, RX: GPIO 47)
// =============================================================================
#include <Arduino.h>
#include "driver/twai.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define LED 21
#define CAN_TX_PIN 48
#define CAN_RX_PIN 47

//      MPU6050
#define SDA 38
#define SCL 39

Adafruit_MPU6050 mpu;

// --- Physical Robot Parameters ---
const float WHEEL_RADIUS = 0.065; // 65mm radius in meters
const float WHEEL_BASE   = 0.305; // 30.5cm track width in meters

// --- Global Telemetry & Sensor States ---
float base_left_rpm  = 0.0;
float base_right_rpm = 0.0;
float v_left_mps     = 0.0;
float v_right_mps    = 0.0;
float v_robot_linear = 0.0;

// Raw Gyro readings
float gyro_raw_x = 0.0;
float gyro_raw_y = 0.0;
float gyro_raw_z = 0.0;

// --- IMU Calibration Offsets (Accel & Gyro) ---
float offsetX = 0.0, offsetY = 0.0, offsetZ = 0.0;
float offsetGyroX = 0.0, offsetGyroY = 0.0, offsetGyroZ = 0.0;

// --- IMU print interval ---
unsigned long lastMpuPrintTime = 0;
const int mpuPrintInterval = 200; // 5 Hz (Non-blocking replacement for delay(200))

unsigned long lastDebugPrintTime = 0;
const int debugPrintInterval = 200; // 5 Hz

// ====================  Frame ID Central ====================

#define MASTER_BOARD_ID 0x100
#define ARM_BOARD_ID    0x200
#define BASE_BOARD_ID   0x300


// Motor IDs to ignore
#define MOTOR_NODE_TX   0x601
#define MOTOR_NODE_RX   0x581

unsigned long lastTransmitTime = 0;
const int transmitInterval = 50;

unsigned long lastBlinkTime = 0;
const int blinkInterval = 500;
bool ledState = LOW;


// =============================================================================
// FUNCTION 1: Calibrate Both Accelerometer and Gyroscope
// =============================================================================
void calibrateMPU() {
  Serial.println(">> Calibrating IMU... Keep robot flat and still!");
  const int samples = 200;
  float sumAccX = 0, sumAccY = 0, sumAccZ = 0;
  float sumGyrX = 0, sumGyrY = 0, sumGyrZ = 0;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumAccX += a.acceleration.x;
    sumAccY += a.acceleration.y;
    sumAccZ += a.acceleration.z;

    sumGyrX += g.gyro.x;
    sumGyrY += g.gyro.y;
    sumGyrZ += g.gyro.z;
    delay(5);
  }

  // Accelerometer biases (preserving 1g on Z)
  offsetX = sumAccX / samples;
  offsetY = sumAccY / samples;
  offsetZ = (sumAccZ / samples) - 9.80665;

  // Gyroscope zero-rate offsets
  offsetGyroX = sumGyrX / samples;
  offsetGyroY = sumGyrY / samples;
  offsetGyroZ = sumGyrZ / samples;

  Serial.printf(">> IMU Calibrated! Accel Bias: [%.2f, %.2f, %.2f] | Gyro Bias: [%.4f, %.4f, %.4f]\n", 
                offsetX, offsetY, offsetZ, offsetGyroX, offsetGyroY, offsetGyroZ);
}

// =============================================================================
// FUNCTION 2: Convert Received RPM to Linear Velocity (m/s)
// =============================================================================
void rpmToLinearVel(float leftRPM, float rightRPM, float &vLeft, float &vRight, float &vLinear) {
  // v = (RPM * 2 * PI / 60) * r
  vLeft   = (leftRPM  * 2.0f * M_PI / 60.0f) * WHEEL_RADIUS;
  vRight  = (rightRPM * 2.0f * M_PI / 60.0f) * WHEEL_RADIUS;
  vLinear = (vRight + vLeft) / 2.0f;
}

// =============================================================================
// FUNCTION 3: Print Linear Velocity & Raw Gyro XYZ
// =============================================================================
void printTelemetryAndRawGyro() {
  // Read current raw IMU state
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gyro_raw_x = g.gyro.x;
  gyro_raw_y = g.gyro.y;
  gyro_raw_z = g.gyro.z;

  // Print Formatted Debug Output
  Serial.printf("Vel [m/s] -> L: %+.3f | R: %+.3f | Center: %+.3f  ||  Raw Gyro [rad/s] -> X: %+.3f | Y: %+.3f | Z: %+.3f\n",
                v_left_mps,
                v_right_mps,
                v_robot_linear,
                gyro_raw_x,
                gyro_raw_y,
                gyro_raw_z);
}

/*                         debug imu
// =============================================================================
// FUNCTION: Read & Print MPU6050 IMU Data
// =============================================================================
void readAndPrintMPU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Apply calibration subtraction
  float lin_accel_x = a.acceleration.x - offsetX;
  float lin_accel_y = a.acceleration.y - offsetY;
  float lin_accel_z = a.acceleration.z - 9.80665 - offsetZ;

  // Print Calibrated Linear Accelerometer Data (m/s^2)
  Serial.print("Linear Accel [m/s^2] -> X: ");
  Serial.print(lin_accel_x, 2);
  Serial.print(" | Y: ");
  Serial.print(lin_accel_y, 2);
  Serial.print(" | Z: ");
  Serial.print(lin_accel_z, 2);

  // Print Gyroscope Data (rad/s)
  Serial.print("  ||  Gyro [rad/s] -> X: ");
  Serial.print(g.gyro.x, 2);
  Serial.print(" | Y: ");
  Serial.print(g.gyro.y, 2);
  Serial.print(" | Z: ");
  Serial.print(g.gyro.z, 2);

  // Print Temperature (deg C)
  Serial.print("  ||  Temp: ");
  Serial.print(temp.temperature, 1);
  Serial.println(" °C");
}
*/
void initCAN() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("System: Failed to install CAN driver");
    while (1);
  }

  if (twai_start() != ESP_OK) {
    Serial.println("System: Failed to start CAN driver");
    while (1);
  }

  Serial.println("System: Master CAN Running at 500kbps.");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // Wait for native USB serial (especially on ESP32-S3)
  }

  Serial.println("Initializing MPU6050...");
  Wire.begin(SDA, SCL);

  // Initialize the sensor (default I2C address is 0x68)
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip! Check wiring.");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Configure sensor ranges & filter bandwidth
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);     // Options: 2_G, 4_G, 8_G, 16_G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);          // Options: 250, 500, 1000, 2000 DEG
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);       // Low-pass filter

  // Run calibration right after sensor init while robot is still
  calibrateMPU();

  delay(1980);

  pinMode(LED, OUTPUT);

  Serial.println("\n=====================================");
  Serial.println("   Master Board 3-Node CAN Network   ");
  Serial.println("=====================================");

  initCAN();
}

void loop() {
  // 1. Heartbeat LED
  if (millis() - lastBlinkTime >= blinkInterval) {
    lastBlinkTime = millis();
    ledState = !ledState;
    digitalWrite(LED, ledState);
  }

  // 2. Bus-Off Check & Auto-Recovery
  twai_status_info_t status;
  twai_get_status_info(&status);
  if (status.state == TWAI_STATE_BUS_OFF) {
    Serial.println(">> Warning: Bus-Off! Initiating recovery...");
    twai_initiate_recovery();
    delay(50);
    twai_start();
    Serial.println(">> Bus recovered.");
  }

// 3. Receive Packets from Arm or Base (Read ALL available frames in queue)
  twai_message_t rx_msg;
  while (twai_receive(&rx_msg, 0) == ESP_OK) {
    if (rx_msg.identifier == MOTOR_NODE_TX || rx_msg.identifier == MOTOR_NODE_RX) {
      // Silently ignore motor communications
    } else if (rx_msg.identifier == ARM_BOARD_ID) {
      /*       Arm
      Serial.print(">> [RX FROM ARM  (0x200)] Data: ");
      for (int i = 0; i < rx_msg.data_length_code; i++) {
        Serial.printf("%02X ", rx_msg.data[i]);
      }
      Serial.println();
      */
    } else if (rx_msg.identifier == BASE_BOARD_ID) {
      // Explicit signed int16_t cast restores negative signs correctly
      int16_t raw_left  = (int16_t)((rx_msg.data[1] << 8) | rx_msg.data[0]);
      int16_t raw_right = (int16_t)((rx_msg.data[3] << 8) | rx_msg.data[2]);

      float base_left_rpm  = raw_left / 10.0f;
      float base_right_rpm = raw_right / 10.0f;
      /*             debug
      Serial.printf("[RX FROM BASE  (0x300)] Left: %.1f RPM | Right: %.1f RPM\n", base_left_rpm, base_right_rpm);
      */   
      // Convert RPM to linear m/s
      rpmToLinearVel(base_left_rpm, base_right_rpm, v_left_mps, v_right_mps, v_robot_linear); 
    } else {
      Serial.printf(">> [RX UNKNOWN  (0x%03X)] Data: ", rx_msg.identifier);
      for (int i = 0; i < rx_msg.data_length_code; i++) {
        Serial.printf("%02X ", rx_msg.data[i]);
      }
      Serial.println();
    }
  }
  
  // 4. Transmit Periodic Frame to Bus
  if (millis() - lastTransmitTime > transmitInterval) {
    if (status.state == TWAI_STATE_RUNNING) {   
      static uint8_t counter = 0;

      twai_message_t tx_msg = {0};
      tx_msg.identifier = MASTER_BOARD_ID;
      tx_msg.extd = 0;
      tx_msg.rtr = 0;
      tx_msg.data_length_code = 4;

      tx_msg.data[0] = 0xAA;
      tx_msg.data[1] = 0x55;
      tx_msg.data[2] = 0x01;
      tx_msg.data[3] = counter++;

      esp_err_t res = twai_transmit(&tx_msg, pdMS_TO_TICKS(2));
      /*                debug
      Serial.printf("[TX MASTER: %s] | State: %d | Bus_Errs: %u\n",
                    esp_err_to_name(res), status.state, status.bus_error_count);
      */
    }
    lastTransmitTime = millis();
  }
  /*
  // 5. Read MPU6050 Non-blocking (200ms / 5Hz)
  if (millis() - lastMpuPrintTime >= mpuPrintInterval) {
    readAndPrintMPU();
    lastMpuPrintTime = millis();
  }
  */
  // 5. Print Telemetry & Raw Gyro (200ms / 5Hz)
  if (millis() - lastDebugPrintTime >= debugPrintInterval) {
    printTelemetryAndRawGyro();
    lastDebugPrintTime = millis();
  }
}
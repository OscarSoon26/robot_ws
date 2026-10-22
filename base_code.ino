#include <Arduino.h>
#include "ZLDriverCANOpen.h"
#include <ESP32-TWAI-CAN.hpp>

// ==================== REQUIRED LIBRARIES ====================
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// =======================================================
//   ESP32-S3 HARDWARE PIN CONFIGURATION
// =======================================================
#define LED_G        21  // Status Indicator LED

#define CAN_TX_PIN   48  // TWAI / CAN Bus Transmit Pin
#define CAN_RX_PIN   47  // TWAI / CAN Bus Receive Pin
#define CAN_SPEED    500000 

#define CH1_PIN      14  // Steering (Left/Right)
#define CH2_PIN      13  // Throttle (Forward/Reverse)

// =======================================================
//   CANOPEN & MOTOR CONFIGURATION
// =======================================================
#define MOTOR_NODE_TX 0x601 
#define MOTOR_NODE_RX 0x581

// =======================================================
//   OPTIMIZED MAPPING SPEED LIMITS (10Hz Lidar)
// =======================================================
#define MAX_LINEAR_RPM    45   // Straight speed limit (approx 0.3 m/s)
#define MAX_ANGULAR_RPM   8    // Turning speed limit (approx 0.4 rad/s)
#define MOTOR_SAFETY_LIMIT 150 // Absolute maximum physical limit of your motors
#define RC_DEADBAND        40  // Deadband applied directly to raw receiver pulses (microseconds)

// Global Command Variables 
int leftCmdRPM = 0;
int rightCmdRPM = 0;
float currentLeftRPM = 0.0;
float currentRightRPM = 0.0;

// ==================== ROS 2 Autonomous Command States ====================
int nav2_target_left_rpm = 0;
int nav2_target_right_rpm = 0;
unsigned long last_nav2_cmd_time = 0;
const unsigned long NAV2_TIMEOUT_MS = 500; // Stop autonomous mode if CAN drops > 500ms

// ========================
//         OBJECTS
// ========================
CANIO CANBUS(CAN_SPEED, CAN_TX_PIN, CAN_RX_PIN, 10, 10);
ZLDriverCANOpen motor(MOTOR_NODE_TX, MOTOR_NODE_RX);

// ========================
//   INTERRUPT VARIABLES
// ========================
volatile unsigned long ch1_start = 0;
volatile uint16_t ch1_val = 1500; 
volatile unsigned long ch1_last_tick = 0; 

volatile unsigned long ch2_start = 0;
volatile uint16_t ch2_val = 1500; 
volatile unsigned long ch2_last_tick = 0; 

// ==================== PARAMETER STRUCTURE ====================
typedef struct send_data {
  int leftCmdRPM_R = 0;
  int rightCmdRPM_R = 0;
  float currentLeftRPM_R = 0.0;
  float currentRightRPM_R = 0.0;
} send_data;

send_data base;

uint8_t communication_channel = 9;
uint8_t receiver_mac_address[] = {0xC8, 0x2E, 0x18, 0xF7, 0x37, 0x4C};
esp_now_peer_info_t target_peer_info;

// ==================== Frame ID Central ====================
#define MASTER_BOARD_ID 0x100
#define ARM_BOARD_ID    0x200
#define BASE_BOARD_ID   0x300

// Central timing
unsigned long lastTransmitTime = 0;
const int transmitInterval = 50; 

// LED configuration
unsigned long lastBlinkTime = 0;
const int blinkInterval = 500;
bool ledState = LOW;

// =======================================================
//   INTERRUPT SERVICE ROUTINES (PWM RECEIVER)
// =======================================================
void IRAM_ATTR ch1_isr() {
  if (digitalRead(CH1_PIN) == HIGH) {
    ch1_start = micros();
  } else {
    unsigned long pulse = micros() - ch1_start;
    if (pulse > 800 && pulse < 2200) { 
      ch1_val = pulse;
      ch1_last_tick = millis(); 
    }
  }
}

void IRAM_ATTR ch2_isr() {
  if (digitalRead(CH2_PIN) == HIGH) {
    ch2_start = micros();
  } else {
    unsigned long pulse = micros() - ch2_start;
    if (pulse > 800 && pulse < 2200) { 
      ch2_val = pulse;
      ch2_last_tick = millis(); 
    }
  }
}

// ESP-NOW Feedback Callback
void OnDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  digitalWrite(LED_G, (status == ESP_NOW_SEND_SUCCESS ? HIGH : LOW));
}

// =======================================================
//   SETUP
// =======================================================
void setup() {
  Serial.begin(115200);
  delay(2000); 

  // Native hardware pull-down setup on ESP32-S3 Pins 14 and 13
  pinMode(CH1_PIN, INPUT_PULLDOWN);
  pinMode(CH2_PIN, INPUT_PULLDOWN);
  pinMode(LED_G, OUTPUT);
  
  attachInterrupt(digitalPinToInterrupt(CH1_PIN), ch1_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2_PIN), ch2_isr, CHANGE);

  // Initialize TWAI CAN Controller on GPIO 48 (TX) / 47 (RX)
  if (CANBUS.begin()) {
    Serial.println("System: ESP32-S3 TWAI CAN Hardware Started.");
  } else {
    Serial.println("System: ESP32-S3 TWAI CAN Hardware Failed!");
    while (1);
  }

  Serial.println("System: Resetting Driver State...");
  motor.canSendSDO(MOTOR_NODE_TX, 2, 0x6040, 0x00, 0x80);
  delay(200);
  motor.canSendSDO(MOTOR_NODE_TX, 2, 0x6040, 0x00, 0x06);
  delay(200);

  Serial.println("System: Enabling Motor Driver...");
  if (motor.begin()) {
    Serial.println("System: Motor Driver Ready.");
  } else {
    Serial.println("System: Driver didn't acknowledge init, attempting to continue...");
  }

  // Initialize Wi-Fi in Station Mode
  WiFi.mode(WIFI_STA);

  // Enforce explicit hardware channel selection
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(communication_channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Link feedback callback
  esp_now_register_send_cb(OnDataSent);

  // Configure target peer
  target_peer_info.channel = communication_channel;
  target_peer_info.ifidx = WIFI_IF_STA;
  target_peer_info.encrypt = false;

  memcpy(target_peer_info.peer_addr, receiver_mac_address, 6);
  
  if (esp_now_add_peer(&target_peer_info) != ESP_OK) {
    Serial.println("Failed to add peer device registration");
    return;
  }
}

// =======================================================
//   MAIN LOOP
// =======================================================
void loop() {
  // 1. Heartbeat LED
  if (millis() - lastBlinkTime >= blinkInterval) {
    lastBlinkTime = millis();
    ledState = !ledState;
    digitalWrite(LED_G, ledState);
  }

  // 2. Capture current RC values safely
  noInterrupts();
  uint16_t rawSteer = ch1_val;
  uint16_t rawThrottle = ch2_val;
  unsigned long last_steer_time = ch1_last_tick;
  unsigned long last_throttle_time = ch2_last_tick;
  interrupts();

  // 3. FAILSAFE: 300ms timeout
  if (millis() - last_steer_time > 300) {
    rawSteer = 1500;
  }
  if (millis() - last_throttle_time > 300) {
    rawThrottle = 1500;
  }

  // 4. CAN BUS LOOP: Update motors every 50ms
  static unsigned long lastCanUpdate = 0;
  if (millis() - lastCanUpdate >= 50) { 
    
    int throttle_in = rawThrottle;
    int steer_in = rawSteer;

    // Apply the deadband directly to raw pulse signals (Neutral is 1500us)
    if (abs(throttle_in - 1500) < RC_DEADBAND) throttle_in = 1500;
    if (abs(steer_in - 1500) < RC_DEADBAND) steer_in = 1500;

    int targetLeftRPM = 0;
    int targetRightRPM = 0;

    // CHECK CONTROL ARBITRATION:
    // RC manual override takes highest priority if sticks move away from neutral
    if (throttle_in != 1500 || steer_in != 1500) {
      int throttle = map(throttle_in, 1000, 2000, -MAX_LINEAR_RPM, MAX_LINEAR_RPM);
      int steer = map(steer_in, 1000, 2000, -MAX_ANGULAR_RPM, MAX_ANGULAR_RPM);

      // STANDARD KINEMATICS (Mix steering and throttle)
      targetLeftRPM = throttle + steer;
      targetRightRPM = throttle - steer;

      // Mirror Fix
      targetRightRPM = -targetRightRPM; 
    } 
    // Otherwise, execute autonomous Nav2 command if recent (< 500ms)
    else if (millis() - last_nav2_cmd_time < NAV2_TIMEOUT_MS) {
      targetLeftRPM = nav2_target_left_rpm;
      targetRightRPM = nav2_target_right_rpm;
    } 
    // If neither RC nor ROS 2 is commanding, hold stationary
    else {
      targetLeftRPM = 0;
      targetRightRPM = 0;
    }

    // Constrain targets to absolute physical hardware limits
    targetLeftRPM = constrain(targetLeftRPM, -MOTOR_SAFETY_LIMIT, MOTOR_SAFETY_LIMIT);
    targetRightRPM = constrain(targetRightRPM, -MOTOR_SAFETY_LIMIT, MOTOR_SAFETY_LIMIT);

    // --- ACCELERATION CURVE (SLEW RATE) ---
    const int MAX_SPEED_STEP = 4; 

    // Smooth Left Motor
    if (targetLeftRPM > leftCmdRPM) {
      leftCmdRPM += min(MAX_SPEED_STEP, targetLeftRPM - leftCmdRPM);
    } else if (targetLeftRPM < leftCmdRPM) {
      leftCmdRPM -= min(MAX_SPEED_STEP, leftCmdRPM - targetLeftRPM);
    }

    // Smooth Right Motor
    if (targetRightRPM > rightCmdRPM) {
      rightCmdRPM += min(MAX_SPEED_STEP, targetRightRPM - rightCmdRPM);
    } else if (targetRightRPM < rightCmdRPM) {
      rightCmdRPM -= min(MAX_SPEED_STEP, rightCmdRPM - targetRightRPM);
    }
    
    // Read the real wheel speeds from driver
    motor.readSpeed(&currentLeftRPM, &currentRightRPM);
    
    motor.writeSpeed(leftCmdRPM, rightCmdRPM);
    motor.parseCANOpen();

    // Assign active readings for wireless telemetry
    base.leftCmdRPM_R       = -leftCmdRPM;   
    base.rightCmdRPM_R      = rightCmdRPM;   
    base.currentLeftRPM_R   = -currentLeftRPM;   
    base.currentRightRPM_R  = currentRightRPM; 

    // Dispatch data package over ESP-NOW
    esp_now_send(receiver_mac_address, (uint8_t *)&base, sizeof(base));

    lastCanUpdate = millis(); 
  }

  // ==========================================
  // 5. Bus-Off Check & Complete Motor Recovery
  // ==========================================
  twai_status_info_t status;
  twai_get_status_info(&status);
  
  if (status.state == TWAI_STATE_BUS_OFF) {
    Serial.println(">> Warning: Bus-Off! Initiating ESP32-S3 recovery...");
    twai_initiate_recovery(); 
  } 
  else if (status.state == TWAI_STATE_STOPPED) {
    twai_start(); 
    Serial.println(">> Bus hardware recovered! Re-enabling Motor Driver...");

    // 1. Clear the driver's safety trip
    motor.canSendSDO(MOTOR_NODE_TX, 2, 0x6040, 0x00, 0x80);
    delay(50);
    
    // 2. Re-run CiA 402 ignition sequence (0x06 -> 0x07 -> 0x0F)
    motor.begin();
  }
  
  // ==========================================
  // 6. Central CAN RX Router (Master, Arm & Motor)
  // ==========================================
  CanFrame rx_lib_msg;
  while (ESP32Can.readFrame(rx_lib_msg, 0)) {
    if (rx_lib_msg.identifier == MOTOR_NODE_RX) {
      motor.processReceivedFrame(rx_lib_msg); // Updates motor RPM / Temp instantly
    }
    else if (rx_lib_msg.identifier == ARM_BOARD_ID) {
      // Arm Board Frame Handler
    }
    else if (rx_lib_msg.identifier == MASTER_BOARD_ID) {
      // Unpack 8-byte command packet from ESP32-S3 Master
      if (rx_lib_msg.data_length_code >= 6 && rx_lib_msg.data[0] == 0xAA && rx_lib_msg.data[1] == 0x55) {
        int16_t raw_l = (int16_t)((rx_lib_msg.data[3] << 8) | rx_lib_msg.data[2]);
        int16_t raw_r = (int16_t)((rx_lib_msg.data[5] << 8) | rx_lib_msg.data[4]);

        // Decode fixed-point scaled RPMs
        float decoded_l = raw_l / 10.0f;
        float decoded_r = raw_r / 10.0f;

        // Apply mirroring identical to RC logic
        nav2_target_left_rpm  = (int)(-decoded_l);
        nav2_target_right_rpm = (int)(decoded_r);

        last_nav2_cmd_time = millis(); // Refresh watchdog
      }
    }
  }

  // ==========================================
  // 7. Transmit Periodic Frame to Bus
  // ==========================================
  if (millis() - lastTransmitTime > transmitInterval) {
    if (status.state == TWAI_STATE_RUNNING) {
      static uint8_t counter = 0;

      CanFrame tx_lib_msg = {0};
      // Pack signed 16-bit integers
      int16_t leftRPM_scaled  = (int16_t)(-currentLeftRPM * 10.0f);
      int16_t rightRPM_scaled = (int16_t)(currentRightRPM * 10.0f);

      tx_lib_msg.identifier = BASE_BOARD_ID;
      tx_lib_msg.data_length_code = 5;

      // Left Wheel
      tx_lib_msg.data[0] = (uint8_t)(leftRPM_scaled & 0xFF);
      tx_lib_msg.data[1] = (uint8_t)((leftRPM_scaled >> 8) & 0xFF);

      // Right Wheel        
      tx_lib_msg.data[2] = (uint8_t)(rightRPM_scaled & 0xFF);
      tx_lib_msg.data[3] = (uint8_t)((rightRPM_scaled >> 8) & 0xFF);

      tx_lib_msg.data[4] = counter++;

      // Write frame to TWAI controller
      bool success = ESP32Can.writeFrame(tx_lib_msg, 2);
      
      Serial.printf("[TX BASE: %s] | State: %d | Bus_Errs: %u\n",
                    success ? "OK" : "TIMEOUT/FAIL", 
                    status.state, 
                    status.bus_error_count);
    }
    lastTransmitTime = millis();
  }
  
  // Debug Serial Printout
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    Serial.print(" | Cmd | L: "); Serial.print(-leftCmdRPM); 
    Serial.print(" R: "); Serial.print(rightCmdRPM);
    Serial.printf(" || RPM L: %.1f, R: %.1f\n", -currentLeftRPM, currentRightRPM);
    
    lastPrint = millis();
  }
}
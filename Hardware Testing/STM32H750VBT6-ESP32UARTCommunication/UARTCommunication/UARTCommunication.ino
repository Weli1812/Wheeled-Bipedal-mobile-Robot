#include <Wire.h>
#include <MPU9250.h> // Using Bolder Flight Systems v1.0.2 header

// Include both EKFs
// MAKE SURE TO CHANGE 'double' TO 'float' INSIDE THESE FILES TOO!
#include "src/EKF.h"
#include "src/src2/XY_EKF.h"

// ==========================================
// PIN DEFINITIONS & HARDWARE
// ==========================================
const int LEFT_ENC_A  = 32;
const int LEFT_ENC_B  = 35; 
const int RIGHT_ENC_A = 4;
const int RIGHT_ENC_B = 5; 

#define RXD2 16
#define TXD2 17

const float MOTOR_PPR          = 17.0f;        
const float GEARBOX_RATIO      = 25.0f;      // <-- CHANGE THIS TO YOUR MOTOR'S REDUCTION RATIO!
const float TICKS_PER_WHEEL_REV = MOTOR_PPR * GEARBOX_RATIO; 

const float WHEEL_RADIUS = 0.12f; 
const float WHEEL_BASE   = 0.40f;    

// ==========================================
// SYNC HEADER DEFINITION (STM32 Framing)
// ==========================================
#define SYNC_BYTE_0  0xAA
#define SYNC_BYTE_1  0x55

// ==========================================
// GLOBAL STATE VARIABLES
// ==========================================
MPU9250 mpu(Wire, 0x68);
float currentRoll = 0.0f, currentPitch = 0.0f, currentYaw = 0.0f;

// Calibration Offsets
float yawOffset = 0.0f; 
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
float vel_left = 0.0f, vel_right = 0.0f;

// STM32 State Vector Variables
float s_displacement = 0.0f; // [s] Forward displacement in meters

float X_state[3]  = {0.0f, 0.0f, 0.0f}; 
float P_matrix[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};

unsigned long lastImuTime = 0;
unsigned long previousLocMicros = 0; 

// Debug printing timer
unsigned long lastDebugPrint = 0;

// ==========================================
// INTERRUPTS
// ==========================================
void IRAM_ATTR leftEncoderISR() { 
  if (digitalRead(LEFT_ENC_B) == HIGH) leftTicks++; 
  else leftTicks--; 
}

void IRAM_ATTR rightEncoderISR() { 
  if (digitalRead(RIGHT_ENC_B) == HIGH) rightTicks++; 
  else rightTicks--; 
}

void setup() {
  Serial.begin(921600);
  Serial2.begin(921600, SERIAL_8N1, RXD2, TXD2);

  // Extremely short timeout so readStringUntil doesn't block the 200Hz EKF loop!
  Serial2.setTimeout(2);

  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  Wire.begin();
  if (mpu.begin() < 0) { 
    Serial.println("MPU Init Failed!"); 
    while(1){} 
  }
  init_biped_EKF(); 

  // =================================================================
  // EKF & GYRO CALIBRATION WARMUP
  // =================================================================
  Serial.println("Warming up EKF... DO NOT MOVE ROBOT.");
  float temp_r, temp_p, temp_y;
  float sum_yaw = 0.0f;
  
  for (int i = 0; i < 450; i++) {
    mpu.readSensor();
    float ax = mpu.getAccelY_mss(); float ay = mpu.getAccelX_mss(); float az = -mpu.getAccelZ_mss();
    float gx = mpu.getGyroY_rads(); float gy = mpu.getGyroX_rads(); float gz = -mpu.getGyroZ_rads();
    float mx = mpu.getMagY_uT();    float my = mpu.getMagX_uT();    float mz = -mpu.getMagZ_uT();

    // ADDED 'true' FLAG HERE FOR STARTUP
    rpy_EKF(ax, ay, az, gx, gy, gz, mx, my, mz, 0.05f, 0.0f, &temp_r, &temp_p, &temp_y);

    if (i >= 400) {
      sum_yaw += temp_y;
      gyroBiasX += mpu.getGyroX_rads();
      gyroBiasY += mpu.getGyroY_rads();
      gyroBiasZ += mpu.getGyroZ_rads();
    }
    delay(10); 
  }
  
  // Calculate average biases
  yawOffset = sum_yaw / 50.0f;
  gyroBiasX = gyroBiasX / 50.0f;
  gyroBiasY = gyroBiasY / 50.0f;
  gyroBiasZ = gyroBiasZ / 50.0f;

  Serial.println("Calibration complete!");
  lastImuTime = micros();
  previousLocMicros = micros();
}

void loop() {
  // =================================================================
  // TASK 0: READ INCOMING STM32 STATUS (Non-blocking)
  // =================================================================
  if (Serial2.available()) {
    String stm32Data = Serial2.readStringUntil('\n');
    stm32Data.trim();
    if (stm32Data.length() > 0) {
      Serial.print("[STM32 Says]: ");
      Serial.println(stm32Data);
    }
  }

  unsigned long currentMicros = micros();

  // =================================================================
  // TASK 1: ATTITUDE EKF 
  // =================================================================
  mpu.readSensor();
  float dt_imu = (float)(currentMicros - lastImuTime) / 1000000.0f;
  lastImuTime = currentMicros;
  if (dt_imu > 1.0f) dt_imu = 0.01f; 

  // ADDED 'false' FLAG HERE FOR NORMAL OPERATION
  rpy_EKF(mpu.getAccelY_mss(), mpu.getAccelX_mss(), -mpu.getAccelZ_mss(),
          mpu.getGyroY_rads(), mpu.getGyroX_rads(), -mpu.getGyroZ_rads(),
          mpu.getMagY_uT(), mpu.getMagX_uT(), -mpu.getMagZ_uT(),
          dt_imu, 0, &currentRoll, &currentPitch, &currentYaw);

  currentYaw = currentYaw - yawOffset;
  while (currentYaw > 180.0f)  currentYaw -= 360.0f;
  while (currentYaw <= -180.0f) currentYaw += 360.0f;

  // =================================================================
  // TASK 2: KINEMATICS & STM32 ARRAY PREPARATION (200Hz / 5ms)
  // =================================================================
  if (currentMicros - previousLocMicros >= 5000) {
    float dt_loc = (float)(currentMicros - previousLocMicros) / 1000000.0f;
    previousLocMicros = currentMicros;

    noInterrupts();
    long curLeftTicks = leftTicks;
    long curRightTicks = rightTicks;
    leftTicks = 0; rightTicks = 0;
    interrupts();

    // 1. Wheel Distances
    float dist_left = ((float)curLeftTicks / TICKS_PER_WHEEL_REV) * (2.0f * PI * WHEEL_RADIUS);
    float dist_right = ((float)curRightTicks / TICKS_PER_WHEEL_REV) * (2.0f * PI * WHEEL_RADIUS);
    
    // 2. Wheel Velocities
    vel_left = dist_left / dt_loc;
    vel_right = dist_right / dt_loc;

    // Run XY Position Filter
    float X_new[3], P_new[9];
    float currentYawRad = currentYaw * PI / 180.0f; 
    XY_EKF(X_state, P_matrix, vel_right, vel_left, currentYawRad, dt_loc, WHEEL_BASE, X_new, P_new);
    for (int i = 0; i < 3; i++) X_state[i] = X_new[i];
    for (int i = 0; i < 9; i++) P_matrix[i] = P_new[i];

    // =================================================================
    // STM32 LQR STATE VECTOR CALCULATION
    // =================================================================
    float phi_rad = currentPitch * (PI / 180.0f);
    s_displacement += (dist_left + dist_right) / 2.0f;
    float theta_rad = currentYawRad;
    float phi_dot = mpu.getGyroX_rads() - gyroBiasX; 
    float v_forward = (vel_right + vel_left) / 2.0f;
    float omega = -(mpu.getGyroZ_rads() - gyroBiasZ); 

    // =================================================================
    // TASK 3: SEND BINARY ARRAY WITH SYNC HEADERS TO STM32
    // =================================================================
    // 2 bytes for header + 24 bytes for 6 floats = 26 bytes total
    uint8_t tx_packet[26];

    // 1. Write the explicit Sync Header bytes required by STM32
    tx_packet[0] = SYNC_BYTE_0; 
    tx_packet[1] = SYNC_BYTE_1; 

    // 2. Package the floats into a temporary array
    float payload[6];
    payload[0] = phi_rad;
    payload[1] = s_displacement;
    payload[2] = theta_rad;
    payload[3] = phi_dot;
    payload[4] = v_forward;
    payload[5] = omega;

    // 3. Copy the 24 bytes of floating-point data right behind the header
    memcpy(&tx_packet[2], payload, 24);

    // 4. Blast the full 26-byte synchronized packet over Serial2
    Serial2.write(tx_packet, sizeof(tx_packet));

    // =================================================================
    // TASK 4: DEBUG PRINTING (Throttled to 10Hz)
    // =================================================================
    if (millis() - lastDebugPrint >= 100) {
      lastDebugPrint = millis();
      Serial.print(phi_rad, 4); Serial.print(",");
      Serial.print(s_displacement, 4); Serial.print(",");
      Serial.print(theta_rad, 4); Serial.print(",");
      Serial.print(phi_dot, 4); Serial.print(",");
      Serial.print(v_forward, 4); Serial.print(",");
      Serial.println(omega, 4);
    }
  }
}
#include <Wire.h>
#include <MPU9250.h>
#include "BluetoothSerial.h"

// Include both EKFs
#include "EKF.h"
#include"XY_EKF.h"

// Include the Trajectory Data
#include "trajectory_data.h"

// CRITICAL: Tell the C++ compiler how to read the MATLAB C code
extern "C" {
    #include "untitled2.h"
}

BluetoothSerial SerialBT;

// ==========================================
// PIN DEFINITIONS & HARDWARE
// ==========================================
const int ENB = 33, IN3 = 25, IN4 = 26;         // Left Motor
const int ENA = 13, IN1 = 14, IN2 = 27;         // Right Motor
const int LEFT_ENC_PIN = 32;
const int RIGHT_ENC_PIN = 4;

const float SLOTS_PER_REV = 20.0; 
const double WHEEL_RADIUS = 0.033; 
const double WHEEL_BASE = 0.15;                 // CRITICAL: Ensure MATLAB 'L' equals 0.15!
const int freq = 5000;       
const int resolution = 8;

// ==========================================
// GLOBAL STATE VARIABLES
// ==========================================
MPU9250 mpu(Wire, 0x68);
double currentRoll = 0.0, currentPitch = 0.0, currentYaw = 0.0;
double yawOffset = 0.0; // The permanent Tare offset

volatile unsigned long leftTicks = 0;
volatile unsigned long rightTicks = 0;
double vel_left = 0.0, vel_right = 0.0;

double X_state[3] = {0.0, 0.0, 0.0}; // [X, Y, Theta]
double P_matrix[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

// Controller Outputs (Made global so Kinematics knows which way wheels are spinning)
double pwmr = 0, pwml = 0;
boolean_T dirr = false, dirl = false; 

// Timers
unsigned long lastImuTime = 0;
unsigned long previousLocMillis = 0;
unsigned long previousCtrlMillis = 0; // Timer for the 10Hz MATLAB Controller

int traj_index = 0; // Trajectory position tracker

// ==========================================
// INTERRUPTS 
// ==========================================
void IRAM_ATTR leftEncoderISR() { leftTicks++; }
void IRAM_ATTR rightEncoderISR() { rightTicks++; }

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Biped_Localizer");
  SerialBT.setTimeout(10); 
  Serial.println("Bluetooth Started! Ready to pair...");

  // =================================================================
  // 1. EMERGENCY HARD BRAKE & PIN SETUP
  // =================================================================
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);
  
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  digitalWrite(ENA, LOW); digitalWrite(ENB, LOW);

  ledcAttach(ENA, freq, resolution);
  ledcAttach(ENB, freq, resolution);

  // Encoder Init
  pinMode(LEFT_ENC_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN), rightEncoderISR, RISING);

  // IMU & EKF Init
  Wire.begin();
  if (mpu.begin() < 0) { Serial.println("MPU Init Failed!"); while(1){} }
  init_biped_EKF();
  
  // =================================================================
  // 2. EKF WARMUP & YAW TARE SEQUENCE
  // =================================================================
  Serial.println("Warming up EKF... DO NOT MOVE ROBOT.");
  double temp_r, temp_p, temp_y;
  double sum_yaw = 0.0;
  
  for (int i = 0; i < 200; i++) {
    mpu.readSensor();
    float ax = mpu.getAccelY_mss(); float ay = mpu.getAccelX_mss(); float az = -mpu.getAccelZ_mss();
    float gx = mpu.getGyroY_rads(); float gy = mpu.getGyroX_rads(); float gz = -mpu.getGyroZ_rads();
    float mx = mpu.getMagY_uT();    float my = mpu.getMagX_uT();    float mz = -mpu.getMagZ_uT();

    rpy_EKF(ax, ay, az, gx, gy, gz, mx, my, mz, 0.05, 0.0, &temp_r, &temp_p, &temp_y);
    if (i >= 150) { sum_yaw += temp_y; }
    delay(10); 
  }
  
  yawOffset = sum_yaw / 50.0;
  Serial.print("Tare complete! Yaw Offset: "); Serial.println(yawOffset);

  // 3. INITIALIZE MATLAB CODER PERSISTENT VARIABLES
  untitled2_init();

  lastImuTime = micros();
}

void loop() {
  unsigned long currentMillis = millis();

  // =================================================================
  // TASK 1: HIGH SPEED ATTITUDE EKF (~100Hz)
  // =================================================================
  mpu.readSensor();
  unsigned long currentMicros = micros();
  float dt_imu = (currentMicros - lastImuTime) / 1000000.0;
  lastImuTime = currentMicros;
  if (dt_imu > 1.0) dt_imu = 0.01;

  rpy_EKF(mpu.getAccelY_mss(), mpu.getAccelX_mss(), -mpu.getAccelZ_mss(),
          mpu.getGyroY_rads(), mpu.getGyroX_rads(), -mpu.getGyroZ_rads(),
          mpu.getMagY_uT(), mpu.getMagX_uT(), -mpu.getMagZ_uT(),
          dt_imu, 0, &currentRoll, &currentPitch, &currentYaw);

  // APPLY YAW OFFSET & BULLETPROOF WRAP
  currentYaw = currentYaw - yawOffset;
  while (currentYaw > 180.0)  currentYaw -= 360.0;
  while (currentYaw <= -180.0) currentYaw += 360.0;

  // =================================================================
  // TASK 2: KINEMATICS & LOCALIZATION EKF (20Hz / 50ms)
  // =================================================================
  if (currentMillis - previousLocMillis >= 50) {
    double dt_loc = (currentMillis - previousLocMillis) / 1000.0;
    previousLocMillis = currentMillis;

    noInterrupts();
    unsigned long curLeftTicks = leftTicks;
    unsigned long curRightTicks = rightTicks;
    leftTicks = 0; rightTicks = 0;
    interrupts();

    double dist_left = (curLeftTicks / SLOTS_PER_REV) * (2.0 * PI * WHEEL_RADIUS);
    double dist_right = (curRightTicks / SLOTS_PER_REV) * (2.0 * PI * WHEEL_RADIUS);
    
    // Determine sign of velocity based on MATLAB controller output direction (dirl/dirr == true means backward)
    vel_left = dirl ? -(dist_left / dt_loc) : (dist_left / dt_loc);
    vel_right = dirr ? -(dist_right / dt_loc) : (dist_right / dt_loc);

    double X_new[3], P_new[9];
    XY_EKF(X_state, P_matrix, vel_right, vel_left, (currentYaw * PI / 180.0), dt_loc, WHEEL_BASE, X_new, P_new);
    
    for (int i = 0; i < 3; i++) X_state[i] = X_new[i];
    for (int i = 0; i < 9; i++) P_matrix[i] = P_new[i];

    // SEND TELEMETRY TO MATLAB
    SerialBT.print(X_state[0], 4); SerialBT.print(",");
    SerialBT.print(X_state[1], 4); SerialBT.print(",");
    SerialBT.print(X_state[2], 4); SerialBT.print(",");
    SerialBT.print(currentRoll, 2); SerialBT.print(",");
    SerialBT.print(currentPitch, 2); SerialBT.print(",");
    SerialBT.println(currentYaw, 2);
  }

  // =================================================================
  // TASK 3: MATLAB LQR/PID AUTONOMOUS CONTROLLER (10Hz / 100ms)
  // =================================================================
  if (currentMillis - previousCtrlMillis >= 100) {
    previousCtrlMillis = currentMillis;

    // 1. Fetch the current target from your trajectory arrays
    double xd_k = xd_traj[traj_index];
    double yd_k = yd_traj[traj_index];
    double thetad_k = thetad_traj[traj_index];
    double vd_k = vd_traj[traj_index];
    double wd_k = wd_traj[traj_index];

    // 2. Call the Generated MATLAB Controller
    // X_state[2] is passed instead of currentYaw because the localization EKF natively outputs radians 
    untitled2(X_state[0], X_state[1], X_state[2], 
              xd_k, yd_k, thetad_k, vd_k, wd_k, 
              vel_right, vel_left, 
              &pwmr, &pwml, &dirr, &dirl);

    // 3. Write to Motor Drivers
    // RIGHT MOTOR (dirr == false is Forward)
    if (!dirr) {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    }

    // LEFT MOTOR (dirl == false is Forward)
    if (!dirl) {
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    } else {
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    }

    // Write PWM (Scale the 0.0-1.0 MATLAB output to 0-255)
    ledcWrite(ENA, (uint32_t)(pwmr * 255.0));
    ledcWrite(ENB, (uint32_t)(pwml * 255.0));

    // 4. Advance Trajectory
    if (traj_index < TRAJ_LENGTH - 1) {
        traj_index++;
    }
  }
}
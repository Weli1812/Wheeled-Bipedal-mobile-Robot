#include <Wire.h>
#include <MPU9250.h>

// Include both EKFs
#include "src/EKF.h"
#include "src/src2/XY_EKF.h"

// Include the Trajectory Data
#include "src/src3/trajectory_data.h"
#include "src/src3/LQRwheels.h"

// ==========================================
// PIN DEFINITIONS & HARDWARE
// ==========================================
const int ENB = 33, IN3 = 25, IN4 = 26;         // Left Motor
const int ENA = 13, IN1 = 14, IN2 = 27;         // Right Motor
const int LEFT_ENC_PIN = 32;
const int RIGHT_ENC_PIN = 4;

const float SLOTS_PER_REV = 20.0; 
const double WHEEL_RADIUS = 0.033; 
const double WHEEL_BASE = 0.15;                 
const int freq = 5000;       
const int resolution = 8;

// TT Motor Deadband Settings
const uint32_t MIN_PWM = 220; // Minimum PWM to make the wheels actually spin
const uint32_t MAX_PWM = 255; 

// ==========================================
// GLOBAL STATE VARIABLES
// ==========================================
MPU9250 mpu(Wire, 0x68);
double currentRoll = 0.0, currentPitch = 0.0, currentYaw = 0.0;
double yawOffset = 0.0; // The permanent Tare offset

volatile unsigned long leftTicks = 0;
volatile unsigned long rightTicks = 0;

// Filtered velocities
double vel_left = 0.0, vel_right = 0.0;

// --- INITIAL POSITION SETTINGS ---
const double START_X = 9.0;
const double START_Y = 1.0;
const double START_THETA_RAD = 1.871; // ~107.2 degrees

// Starting state initialized with the constants above
double X_state[3] = {START_X, START_Y, START_THETA_RAD}; // [X, Y, Theta in radians]
double P_matrix[9] = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

// Controller Outputs 
double pwmr = 0, pwml = 0;
boolean_T dirr = false, dirl = false; 

// Timers
unsigned long lastImuTime = 0;
unsigned long previousLocMillis = 0;
unsigned long previousCtrlMillis = 0; 

int traj_index = 0; // Trajectory position tracker

// ==========================================
// INTERRUPTS 
// ==========================================
void IRAM_ATTR leftEncoderISR() { leftTicks++; }
void IRAM_ATTR rightEncoderISR() { rightTicks++; }

void setup() {
  Serial.begin(115200);
  Serial.println("System Starting...");

  // 1. EMERGENCY HARD BRAKE & PIN SETUP
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
  
  // 2. EKF WARMUP & YAW TARE SEQUENCE
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

  // 3. INITIALIZE TIMERS
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

  // APPLY YAW OFFSET & SHIFT TO DESIRED STARTING ANGLE
  double start_heading_deg = START_THETA_RAD * (180.0 / PI); 
  currentYaw = (currentYaw - yawOffset) + start_heading_deg;
  
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
    
    double raw_vel_left = dirl ? -(dist_left / dt_loc) : (dist_left / dt_loc);
    double raw_vel_right = dirr ? -(dist_right / dt_loc) : (dist_right / dt_loc);

    // [FIX 1] LOW PASS FILTER FOR VELOCITY
    // This stops the LQR from stuttering violently due to low-resolution encoder spikes.
    // 0.3 means we trust the new reading 30%, and maintain 70% of our smoothed momentum.
    vel_left = (0.7 * vel_left) + (0.3 * raw_vel_left);
    vel_right = (0.7 * vel_right) + (0.3 * raw_vel_right);

    double X_new[3], P_new[9];
    XY_EKF(X_state, P_matrix, vel_right, vel_left, (currentYaw * PI / 180.0), dt_loc, WHEEL_BASE, X_new, P_new);
    
    for (int i = 0; i < 3; i++) X_state[i] = X_new[i];
    for (int i = 0; i < 9; i++) P_matrix[i] = P_new[i];

    // SEND TELEMETRY TO MATLAB VIA USB SERIAL
    Serial.print(X_state[0], 4); Serial.print(",");
    Serial.print(X_state[1], 4); Serial.print(",");
    Serial.print(X_state[2], 4); Serial.print(",");
    Serial.print(currentRoll, 2); Serial.print(",");
    Serial.print(currentPitch, 2); Serial.print(",");
    Serial.println(currentYaw, 2);
  }

  // =================================================================
  // TASK 3: MATLAB LQR/PID AUTONOMOUS CONTROLLER (10Hz / 100ms)
  // =================================================================
  if (currentMillis - previousCtrlMillis >= 100) {
    previousCtrlMillis = currentMillis;

    double xd_k = xd_traj[traj_index];
    double yd_k = yd_traj[traj_index];
    double thetad_k = thetad_traj[traj_index];
    double vd_k = vd_traj[traj_index];
    double wd_k = wd_traj[traj_index];

    // Call the Generated MATLAB Controller
    LQRwheels(X_state[0], X_state[1], X_state[2], 0.1, 
              xd_k, yd_k, thetad_k, vd_k, wd_k, 
              vel_right, vel_left, 
              &pwmr, &pwml, &dirr, &dirl);

    // Write to Motor Drivers (FLIPPED HIGH/LOW LOGIC)
    if (dirr) {
        digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);  
    } else {
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  
    }

    if (dirl) {
        digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  
    } else {
        digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);  
    }

    // -------------------------------------------------------------
    // TT MOTOR DEADBAND COMPENSATION & WAYPOINT TRACKING
    // -------------------------------------------------------------
    uint32_t final_pwmR = 0;
    uint32_t final_pwmL = 0;

    // Check if we have reached the end of the trajectory array
    if (traj_index >= TRAJ_LENGTH - 1) {
        final_pwmR = 0;
        final_pwmL = 0;
        Serial.println("Trajectory Complete. Motors Stopped.");
    } 
    else {
        // [FIX 2] TWEAKED DEADBAND THRESHOLD
        // Raised to 0.05 to ignore micro-noise that causes the motors to click/buzz
        if (pwmr > 0.05) {
            final_pwmR = MIN_PWM + (uint32_t)((pwmr) * (MAX_PWM - MIN_PWM));
        }
        if (pwml > 0.05) {
            final_pwmL = MIN_PWM + (uint32_t)((pwml) * (MAX_PWM - MIN_PWM));
        }
        
        // [FIX 3] INCREASED WAYPOINT ACCEPTANCE RADIUS
        // Calculate physical distance from robot to current target point
        double dist_to_target = sqrt(pow(xd_k - X_state[0], 2) + pow(yd_k - X_state[1], 2));
        
        // Increased from 0.15m to 0.25m to prevent the robot from spiraling 
        // if it slightly misses the exact coordinate center.
        if (dist_to_target < 0.25) { 
            traj_index++;
        }
    }

    // Write Final PWM
    ledcWrite(ENA, final_pwmR);
    ledcWrite(ENB, final_pwmL);
  }
}
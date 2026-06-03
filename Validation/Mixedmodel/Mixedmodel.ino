// =============================================================================
// Mixedmodel.ino  —  ESP32 Sensor / Localisation Node
// =============================================================================
// Role in the system
// ------------------
// This node runs at 200 Hz on Core 1.  Every 5 ms it:
//   1. Reads the MPU9250 (accel + gyro + mag)
//   2. Runs the Attitude EKF  → Roll, Pitch, Yaw  (degrees) + gyro biases
//   3. Reads quadrature encoders → wheel velocities, arc-length
//   4. Runs the XY-Position EKF → (X, Y, Θ) in world frame
//   5. Assembles an 8-float state vector and sends it to the STM32 via UART
//
// IMU axis remapping (IMPORTANT for understanding bias signs)
// ------------------------------------------------------------
// The MPU9250 is physically mounted so that its sensor-X axis aligns with the
// robot's PITCH axis (forward/backward tilt) and its sensor-Y axis aligns with
// the robot's ROLL axis. To keep the EKF in a standard NED/body frame the axes
// are swapped before being passed to attitude_EKF():
//
//   EKF  ax = mpu.getAccelY_mss()     EKF  gx = mpu.getGyroY_rads()
//   EKF  ay = mpu.getAccelX_mss()     EKF  gy = mpu.getGyroX_rads()  ← pitch axis
//   EKF  az = -mpu.getAccelZ_mss()    EKF  gz = -mpu.getGyroZ_rads() ← yaw axis (negated)
//
// The EKF state is x = [q0, q1, q2, q3, bx, by, bz] where:
//   bx → bias of EKF gx → bias of mpu.getGyroY_rads()  (roll  rate sensor)
//   by → bias of EKF gy → bias of mpu.getGyroX_rads()  (PITCH rate sensor)  ← used for phi_dot
//   bz → bias of EKF gz → bias of −mpu.getGyroZ_rads() (yaw   rate sensor)
//
// EKF pitch is extracted as asin(2*(q0*q2 − q1*q3)), which is rotation around
// the EKF Y axis = sensor X axis. Therefore pitch RATE (phi_dot) must be
// corrected with the EKF's *by* bias, not *bx*.
// =============================================================================

#include <Wire.h>
#include <MPU9250.h>
#include "src/EKF.h"
#include "src/src2/XY_EKF.h"

// ==========================================
// PINS & HARDWARE
// ==========================================
const int LEFT_ENC_A  = 32;   // Left  encoder channel A  — interrupt source
const int LEFT_ENC_B  = 35;   // Left  encoder channel B  — direction sense
const int RIGHT_ENC_A = 4;    // Right encoder channel A  — interrupt source
const int RIGHT_ENC_B = 5;    // Right encoder channel B  — direction sense

// UART to STM32: Serial2 on the standard ESP32 pins
#define RXD2 16
#define TXD2 17

// -----------------------------------------------------------------------
// Mechanical parameters — adjust if hardware changes
// -----------------------------------------------------------------------
const float MOTOR_PPR           = 17.0f;   // Encoder pulses per motor revolution
const float GEARBOX_RATIO       = 25.0f;   // Motor gearbox ratio
const float TICKS_PER_WHEEL_REV = MOTOR_PPR * GEARBOX_RATIO;  // = 425 ticks/rev

const float WHEEL_RADIUS = 0.06f;   // metres
const float WHEEL_BASE   = 0.40f;   // metres (centre-to-centre)

// -----------------------------------------------------------------------
// Packet framing — must match STM32 defines exactly
// -----------------------------------------------------------------------
// Packet layout (35 bytes):
//   [0]     0xAA  — sync byte 0
//   [1]     0x55  — sync byte 1
//   [2..5]  x_robot  (float32, little-endian)
//   [6..9]  y_robot
//   [10..13] phi   (pitch, radians, mechanical offset already subtracted)
//   [14..17] s     (cumulative arc-length, metres)
//   [18..21] theta (yaw, radians, tared to startup heading)
//   [22..25] v     (forward velocity, m/s)
//   [26..29] omega (yaw rate, rad/s, bias-corrected)
//   [30..33] phi_dot (pitch rate, rad/s, bias-corrected)
//   [34]    XOR checksum of bytes [2..33]
#define SYNC_BYTE_0   0xAA
#define SYNC_BYTE_1   0x55
#define PAYLOAD_FLOATS  8
#define PAYLOAD_BYTES   (PAYLOAD_FLOATS * 4)   // 32
#define PACKET_SIZE     (2 + PAYLOAD_BYTES + 1) // 35

// -----------------------------------------------------------------------
// Static pitch-offset correction
// -----------------------------------------------------------------------
// The IMU may not sit perfectly level on the chassis.  Measure the mean
// pitch reading while the robot is stationary and balanced, then set this.
// Units: RADIANS.  Subtracted from phi before transmission.
// Re-measure if the IMU is re-mounted.
const float PITCH_OFFSET_RAD = 0.13f;   // ~7.4° — verified for current hardware

// ==========================================
// GLOBAL STATE
// ==========================================
MPU9250 mpu(Wire, 0x68);

// Attitude (from EKF) — degrees
float currentRoll  = 0.0f;
float currentPitch = 0.0f;
float currentYaw   = 0.0f;

// Live gyro biases exported by the Attitude EKF each frame.
// Axis mapping (see header comment):
//   live_bx → bias of mpu.getGyroY_rads()  (roll  axis)
//   live_by → bias of mpu.getGyroX_rads()  (PITCH axis) — used for phi_dot
//   live_bz → bias of −mpu.getGyroZ_rads() (yaw   axis, sign-included)
float live_bx = 0.0f, live_by = 0.0f, live_bz = 0.0f;

// Yaw tare — zeroed to the heading at power-on
float yawOffset    = 0.0f;
bool  is_yaw_tared = false;

// Quadrature encoder tick counters — written in ISR, read & reset in loop()
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

// Kinematic derived quantities
float vel_left       = 0.0f;
float vel_right      = 0.0f;
float s_displacement = 0.0f;   // cumulative arc-length (metres)

// XY-EKF state and covariance
float X_state[3]  = {0.0f, 0.0f, 0.0f}; // [X, Y, Θ]  (metres, metres, radians)
float P_matrix[9] = {1,0,0, 0,1,0, 0,0,1};

// Loop timing
unsigned long previousLocMicros = 0;

// ==========================================
// DEBUG TASK INFRASTRUCTURE
// ==========================================
// All data the debug task needs is copied into this struct under a mutex.
// Core 1 (control loop) writes it at 10 Hz.
// Core 0 (debug task) reads and prints — completely isolated from Serial2
// and the EKFs, so any Serial stall is invisible to the control loop.
struct DebugSnapshot {
  float x_robot;
  float y_robot;
  float phi_rad;
  float s_disp;
  float yaw_rad;
  float phi_dot;
  float v_forward;
  float omega;
};

static DebugSnapshot      debugSnap;
static SemaphoreHandle_t  debugMutex    = nullptr;
static volatile bool      debugSnapReady = false;

// Debug task — pinned to Core 0, priority 1 (lowest useful).
// Serial.print happens here and only here.
void debugTask(void* pvParameters) {
  for (;;) {
    if (xSemaphoreTake(debugMutex, portMAX_DELAY) == pdTRUE) {
      if (debugSnapReady) {
        DebugSnapshot snap = debugSnap;   // local copy — release mutex fast
        debugSnapReady = false;
        xSemaphoreGive(debugMutex);

        Serial.print("X:");       Serial.print(snap.x_robot,   4);
        Serial.print(", Y:");     Serial.print(snap.y_robot,   4);
        Serial.print(", phi:");   Serial.print(snap.phi_rad,   4);
        Serial.print(", s:");     Serial.print(snap.s_disp,    4);
        Serial.print(", yaw:");   Serial.print(snap.yaw_rad,   4);
        Serial.print(", pd:");    Serial.print(snap.phi_dot,   4);
        Serial.print(", v:");     Serial.print(snap.v_forward, 4);
        Serial.print(", om:");    Serial.println(snap.omega,   4);
      } else {
        xSemaphoreGive(debugMutex);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));   // 5 ms yield — adequate for 10 Hz output
  }
}

// ==========================================
// ENCODER INTERRUPTS
// ==========================================
// 1× quadrature decode: count on rising edge of A, use B for direction.
// Convention: B HIGH on A↑ → positive tick (forward wheel rotation).
// Verify with your actual motor wiring; swap ++ / -- if velocities are inverted.
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(LEFT_ENC_B) == HIGH) leftTicks++; else leftTicks--;
}
void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(RIGHT_ENC_B) == HIGH) rightTicks++; else rightTicks--;
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(460800);
  // UART to STM32 — baud rate MUST match MX_USART1_UART_Init() on STM32 side
  Serial2.begin(460800, SERIAL_8N1, RXD2, TXD2);

  // Encoder pins — internal pull-ups avoid floating inputs
  pinMode(LEFT_ENC_A,  INPUT_PULLUP);
  pinMode(LEFT_ENC_B,  INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);

  // Attach interrupts on rising edge of channel A only (1× decode)
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A),  leftEncoderISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  // MPU9250 init via I2C
  Wire.begin();
  if (mpu.begin() < 0) {
    Serial.println("MPU Init Failed! Halting.");
    while (1) {}
  }

  // Initialise the Attitude EKF state & covariance
  init_biped_EKF();

  // Create mutex before spawning the debug task
  debugMutex = xSemaphoreCreateMutex();
  if (debugMutex == nullptr) {
    Serial.println("Mutex creation failed! Halting.");
    while (1) {}
  }

  // Debug task on Core 0, priority 1 — below FreeRTOS timer tasks (prio 2)
  xTaskCreatePinnedToCore(
    debugTask,     // function
    "DebugPrint",  // name
    2048,          // stack words — ample for Serial.print
    nullptr,       // parameter
    1,             // priority
    nullptr,       // handle not needed
    0              // Core 0
  );

  // -----------------------------------------------------------------------
  // EKF WARMUP — 900 frames × 5 ms = 4.5 seconds
  // DO NOT MOVE THE ROBOT during this period.
  // The EKF needs ~200+ frames to converge gyro biases from the 70×I
  // initial covariance.  900 frames gives comfortable margin.
  // -----------------------------------------------------------------------
  Serial.println("Warming up EKF (4.5 s) — DO NOT MOVE ROBOT.");

  float t_r, t_p, t_y, t_bx, t_by, t_bz;
  for (int i = 0; i < 900; i++) {
    mpu.readSensor();
    rpy_EKF(
      mpu.getAccelY_mss(), mpu.getAccelX_mss(), -mpu.getAccelZ_mss(),
      mpu.getGyroY_rads(), mpu.getGyroX_rads(), -mpu.getGyroZ_rads(),
      mpu.getMagY_uT(),    mpu.getMagX_uT(),    -mpu.getMagZ_uT(),
      0.005f, 0.0f, /*is_startup=*/true,
      &t_r, &t_p, &t_y, &t_bx, &t_by, &t_bz
    );
    // Keep live biases current so frame 0 of the main loop starts clean
    live_bx = t_bx;
    live_by = t_by;
    live_bz = t_bz;
    delay(5);
  }

  Serial.println("EKF calibration complete. Starting control loop.");
  previousLocMicros = micros();
}

// ==========================================
// MAIN LOOP — Core 1 only, 200 Hz
// ==========================================
void loop() {
  // ---- Non-blocking read of any STM32 replies (currently diagnostic only) ----
  static char rx_buffer[64];
  static int  rx_index = 0;
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    if (c == '\n') {
      rx_buffer[rx_index] = '\0';
      rx_index = 0;
    } else if (c != '\r' && rx_index < 63) {
      rx_buffer[rx_index++] = c;
    }
  }

  unsigned long currentMicros = micros();

  // ---- 200 Hz control tick (5000 µs period) ----------------------------------
  if (currentMicros - previousLocMicros >= 5000) {
    float dt = (float)(currentMicros - previousLocMicros) / 1000000.0f;
    previousLocMicros = currentMicros;

    // ===========================================================
    // STEP 1 — CAPTURE BIASES *BEFORE* THE EKF UPDATES THEM
    // ===========================================================
    // The EKF will produce updated bias estimates (live_bx/by/bz) that reflect
    // the *new* frame.  But phi_dot and omega must be corrected with the bias
    // that was active *during the integration window* being measured.
    // Capturing them here (pre-EKF) is the correct approach.
    //
    // Axis reminder:
    //   prev_bx → bias of getGyroY_rads() (roll  axis, EKF gx)
    //   prev_by → bias of getGyroX_rads() (PITCH axis, EKF gy) — used for phi_dot
    //   prev_bz → bias of −getGyroZ_rads() (yaw axis, sign included) — used for omega
    float prev_bx = live_bx;   // kept in case needed for diagnostics
    float prev_by = live_by;   // PITCH axis bias  ← phi_dot correction
    float prev_bz = live_bz;   // YAW   axis bias  ← omega correction

    // ===========================================================
    // STEP 2 — READ RAW SENSOR DATA (single readSensor() call caches all values)
    // ===========================================================
    mpu.readSensor();

    // Capture the two raw gyro values used for phi_dot and omega BEFORE the
    // EKF call.  Capturing here and passing by variable (not inline function
    // calls) makes the data-flow unambiguous.
    float raw_gyro_x = mpu.getGyroX_rads();   // sensor X = PITCH axis (EKF gy)
    float raw_gyro_y = mpu.getGyroY_rads();   // sensor Y = roll  axis (EKF gx)
    float raw_gyro_z = mpu.getGyroZ_rads();   // sensor Z = yaw   axis (negated into EKF)

    // ===========================================================
    // STEP 3 — RUN ATTITUDE EKF
    // ===========================================================
    // Axis swap (sensor X↔Y) is intentional — see file header.
    // Outputs: currentRoll/Pitch/Yaw in DEGREES, live_bx/by/bz updated.
    rpy_EKF(
      mpu.getAccelY_mss(), mpu.getAccelX_mss(), -mpu.getAccelZ_mss(),
      raw_gyro_y,          raw_gyro_x,           -raw_gyro_z,
      mpu.getMagY_uT(),    mpu.getMagX_uT(),     -mpu.getMagZ_uT(),
      dt, 0.0f, /*is_startup=*/false,
      &currentRoll, &currentPitch, &currentYaw,
      &live_bx, &live_by, &live_bz
    );

    // ===========================================================
    // STEP 4 — TARE YAW (first live frame only)
    // ===========================================================
    if (!is_yaw_tared) {
      yawOffset  = currentYaw;
      is_yaw_tared = true;
    }
    currentYaw -= yawOffset;
    // Wrap to (−180, +180]
    while (currentYaw >  180.0f) currentYaw -= 360.0f;
    while (currentYaw <= -180.0f) currentYaw += 360.0f;

    // ===========================================================
    // STEP 5 — READ ENCODERS (atomic snapshot, then reset)
    // ===========================================================
    noInterrupts();
    long curLeftTicks  = leftTicks;
    long curRightTicks = rightTicks;
    leftTicks  = 0;
    rightTicks = 0;
    interrupts();

    // Convert ticks → wheel distances (metres) → velocities (m/s)
    float dist_left  = ((float)curLeftTicks  / TICKS_PER_WHEEL_REV) * (2.0f * PI * WHEEL_RADIUS);
    float dist_right = ((float)curRightTicks / TICKS_PER_WHEEL_REV) * (2.0f * PI * WHEEL_RADIUS);
    vel_left  = dist_left  / dt;
    vel_right = dist_right / dt;

    // Accumulate arc-length (always positive magnitude of average displacement)
    s_displacement += (dist_left + dist_right) * 0.5f;

    // ===========================================================
    // STEP 6 — RUN XY POSITION EKF
    // ===========================================================
    // Prediction:  differential-drive kinematics
    // Correction:  yaw from Attitude EKF (angle wrap handled inside XY_EKF)
    float X_new[3], P_new[9];
    float currentYawRad = currentYaw * (PI / 180.0f);
    XY_EKF(X_state, P_matrix,
           vel_right, vel_left,      // v_R, v_L
           currentYawRad,            // yaw measurement (rad)
           dt, WHEEL_BASE,
           X_new, P_new);
    for (int i = 0; i < 3; i++) X_state[i] = X_new[i];
    for (int i = 0; i < 9; i++) P_matrix[i] = P_new[i];

    // ===========================================================
    // STEP 7 — ASSEMBLE STATE VECTOR
    // ===========================================================
    float x_robot  = X_state[0];
    float y_robot  = X_state[1];

    // Convert pitch to radians and subtract mechanical mounting offset
    float phi_rad  = currentPitch * (PI / 180.0f) - PITCH_OFFSET_RAD;

    // ---------------------------------------------------------------
    // phi_dot — pitch RATE, bias-corrected
    // ---------------------------------------------------------------
    // Pitch is rotation around the EKF Y axis = sensor X axis.
    // The EKF's "by" (live_by / prev_by) is the estimated bias of
    // that axis's gyro (mpu.getGyroX_rads() = raw_gyro_x).
    // We use prev_by (captured before the EKF ran) because it
    // corresponds to the bias active during this dt window.
    float phi_dot  = raw_gyro_x - prev_by;

    // ---------------------------------------------------------------
    // omega — yaw RATE, bias-corrected
    // ---------------------------------------------------------------
    // The EKF received gz = −raw_gyro_z, so bz is the bias of −raw_gyro_z.
    // Corrected gz  = −raw_gyro_z − bz
    // Therefore omega (in the same sign convention) = −raw_gyro_z − prev_bz.
    float omega    = -raw_gyro_z - prev_bz;

    float v_forward = (vel_right + vel_left) * 0.5f;

    // ===========================================================
    // STEP 8 — BUILD & SEND 35-BYTE UART PACKET TO STM32
    // ===========================================================
    uint8_t tx_packet[PACKET_SIZE];
    tx_packet[0] = SYNC_BYTE_0;
    tx_packet[1] = SYNC_BYTE_1;

    float payload[PAYLOAD_FLOATS] = {
      x_robot,       // bytes [2..5]
      y_robot,       // bytes [6..9]
      phi_rad,       // bytes [10..13]   pitch (rad), offset corrected
      s_displacement,// bytes [14..17]   arc-length (m)
      currentYawRad, // bytes [18..21]   yaw (rad), tared
      v_forward,     // bytes [22..25]   forward velocity (m/s)
      omega,         // bytes [26..29]   yaw rate (rad/s), bias corrected
      phi_dot        // bytes [30..33]   pitch rate (rad/s), bias corrected
    };
    memcpy(&tx_packet[2], payload, PAYLOAD_BYTES);

    // XOR checksum over payload bytes only (indices 2..33 inclusive)
    uint8_t calc_crc = 0;
    for (int i = 2; i < 2 + PAYLOAD_BYTES; i++) {
      calc_crc ^= tx_packet[i];
    }
    tx_packet[PACKET_SIZE - 1] = calc_crc;

    Serial2.write(tx_packet, PACKET_SIZE);

    // ===========================================================
    // STEP 9 — PUSH DEBUG SNAPSHOT TO CORE 0 AT 10 Hz
    // ===========================================================
    // Non-blocking TakeWithTimeout=0 means we never stall the control loop:
    // if the debug task holds the mutex we simply skip this snapshot.
    static unsigned long lastSnapMicros = 0;
    if (currentMicros - lastSnapMicros >= 100000UL) {   // 100 ms = 10 Hz
      lastSnapMicros = currentMicros;
      if (xSemaphoreTake(debugMutex, 0) == pdTRUE) {
        debugSnap.x_robot   = x_robot;
        debugSnap.y_robot   = y_robot;
        debugSnap.phi_rad   = phi_rad;
        debugSnap.s_disp    = s_displacement;
        debugSnap.yaw_rad   = currentYawRad;
        debugSnap.phi_dot   = phi_dot;
        debugSnap.v_forward = v_forward;
        debugSnap.omega     = omega;
        debugSnapReady = true;
        xSemaphoreGive(debugMutex);
      }
    }
  }
}
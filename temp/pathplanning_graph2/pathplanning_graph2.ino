// ============================================================
// PROJECT: Bipedal Sensor Fusion & Path Planning Bridge
// PLATFORM: ESP32-S3
// SENSORS: 2x Ultrasonics + LD06 Time-of-Flight LiDAR
// ============================================================

// --- Hardware Pins ---
#define RXD2 4 
#define TXD2 5 
#define PWM_LIDAR 9  

#define TRIG_LEFT 6
#define ECHO_LEFT 7
#define TRIG_RIGHT 15
#define ECHO_RIGHT 8

// --- Sensor Data Arrays ---
float lidar_distances[360]; 

// --- Stage 1: Dual Ultrasonic Kalman Variables ---
float x_us = 0.0;    
float P_us = 1.0;    
float Q_us = 0.05;   
float R2_us = 0.40;  

// --- Stage 2: Final Fusion Kalman Variables ---
float x_final = 0.0;   
float P_final = 1.0;
float Q_final = 1.5;

unsigned long last_fusion_time = 0; 

void setup() {
  Serial.begin(115200);
  delay(1000); 
  
  pinMode(PWM_LIDAR, OUTPUT);
  analogWrite(PWM_LIDAR, 255); // Max speed
  
  Serial2.begin(230400, SERIAL_8N1, RXD2, TXD2);
  
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
  
  Serial.println("==================================================");
  Serial.println("Bipedal Vision Node Started!");
  Serial.println("Generating Bounding Boxes at 20Hz...");
  Serial.println("==================================================");
}

void loop() {
  // ==========================================
  // BACKGROUND PROCESS: Continuous LD06 Parser
  // ==========================================
  if (Serial2.available()) {
    uint8_t b1 = Serial2.read();
    if (b1 == 84) { 
      int timeout = 0;
      while (!Serial2.available() && timeout < 100) { delay(1); timeout++; }

      if (Serial2.available()) {
        uint8_t b2 = Serial2.read();
        if (b2 == 44) { 
          uint8_t packet[45];
          int bytesRead = Serial2.readBytes(packet, 45);

          if (bytesRead == 45) {
            float start_angle = (packet[2] + (packet[3] << 8)) / 100.0;
            float end_angle   = (packet[40] + (packet[41] << 8)) / 100.0;
            float diff = end_angle - start_angle;
            if (diff < 0) diff += 360.0; 
            float step = diff / 11.0; 

            for (int i = 0; i < 12; i++) {
              int base = 4 + (i * 3); 
              uint16_t dist_mm = packet[base] + (packet[base + 1] << 8);
              float exact_angle = start_angle + (step * i);
              if (exact_angle >= 360.0) exact_angle -= 360.0;
              int angle_idx = (int)round(exact_angle) % 360;
              
              lidar_distances[angle_idx] = dist_mm / 10.0; 
            }
            
            // ==========================================
            // FOREGROUND PROCESS: The Math (Runs every 50ms)
            // ==========================================
            if (millis() - last_fusion_time >= 50) {
              executeKalmanFusion();
              last_fusion_time = millis();
            }
          }
        }
      }
    }
  }
}

// ============================================================
// THE SENSOR FUSION & PATH PLANNING ENGINE
// ============================================================
void executeKalmanFusion() {
  // --- STAGE 1: READ & FUSE ULTRASONIC SENSORS ---
  float z1 = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  float z2 = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
  
  if (z1 <= 0 || z1 > 250) z1 = 250;
  if (z2 <= 0 || z2 > 250) z2 = 250;
  
  float x_pred_us = z1;
  float P_pred_us = P_us + Q_us;
  float K_us = P_pred_us / (P_pred_us + R2_us);
  x_us = x_pred_us + K_us * (z2 - x_pred_us);
  P_us = (1.0 - K_us) * P_pred_us;
  
  float zU_fused = x_us; 

  // --- STAGE 2: READ LiDAR ROI ---
  float zL = 250.0; 
  for (int i = 0; i <= 15; i++) {
    // Apply physical offset to align LiDAR with front of robot
    float d = lidar_distances[i] - LIDAR_OFFSET_CM; 
    
    // Ensure 'd' is still valid after offset (prevent negative distances)
    if (d > 5.0 && d <= 250.0 && d < zL) zL = d;
  }
  for (int i = 345; i < 360; i++) {
    // Apply physical offset to align LiDAR with front of robot
    float d = lidar_distances[i] - LIDAR_OFFSET_CM; 
    
    // Ensure 'd' is still valid after offset
    if (d > 5.0 && d <= 250.0 && d < zL) zL = d;
  }
  // --- STAGE 3: DYNAMIC TRUST LOGIC ---
  float RU, RL;
  if (zU_fused < 150 && zL > (zU_fused + 40)) {
    RU = 1.0;      
    RL = 10000.0;  
  } else if ((zL < 200.0) && (zU_fused > zL + 40.0)) {
    RU = 10000;
    RL = 1;
  } else if (zU_fused < 30) {
    RU = 5.0; 
    RL = 50.0;     
  } else {
    RU = 25.0; 
    RL = 5.0;      
  }

  // --- STAGE 4: SEQUENTIAL KALMAN FILTER ---
  float x_pred_final = x_final; 
  float P_pred_final = P_final + Q_final; 
  
  float K_u = P_pred_final / (P_pred_final + RU);
  float x_upd1 = x_pred_final + K_u * (zU_fused - x_pred_final);
  float P_upd1 = (1.0 - K_u) * P_pred_final;
  
  float K_l = P_upd1 / (P_upd1 + RL);
  x_final = x_upd1 + K_l * (zL - x_upd1);
  P_final = (1.0 - K_l) * P_upd1;

  // ==========================================
  // STAGE 5: PATH PLANNING LOGIC BRIDGE
  // ==========================================
  float r_out = x_final;
  int theta_start = -15; 
  int theta_end = 15;

  // CONDITION A: Elevation Override (Only US sees the low object)
  if (zU_fused < 160.0 && zL > (zU_fused + 40.0)) {
    theta_start = -15;
    theta_end = 15;
  } 
  // CONDITION B: Standard Fusion (LiDAR sees the object)
  else {
    int min_t = 999;
    int max_t = -999;
    
    // Sweep Left Side (-15 to -1 degrees, mapped from array 345 to 359)
    for (int i = 345; i < 360; i++) {
      float d = lidar_distances[i];
      if (d > 5.0 && d < 250.0) {
        int ang = i - 360; 
        if (ang < min_t) min_t = ang;
        if (ang > max_t) max_t = ang;
      }
    }
    
    // Sweep Right Side (0 to 15 degrees)
    for (int i = 0; i <= 15; i++) {
      float d = lidar_distances[i];
      if (d > 5.0 && d < 250.0) {
        int ang = i;
        if (ang < min_t) min_t = ang;
        if (ang > max_t) max_t = ang;
      }
    }
    
    // If we found valid hits, update the boundaries
    if (min_t != 999 && max_t != -999) {
      theta_start = min_t;
      theta_end = max_t;
    }
  }
// --- PRINT UNIFIED MEGA PACKET ---
  Serial.print("DATA -> US:");  Serial.print(zU_fused);
  Serial.print(", L:");         Serial.print(zL);
  Serial.print(", F:");         Serial.print(x_final);
  Serial.print(", r:");         Serial.print(r_out);
  Serial.print(", t_start:");   Serial.print(theta_start);
  Serial.print(", t_end:");     Serial.println(theta_end);
}

// ============================================================
// NON-BLOCKING ULTRASONIC READ (HARD CAPPED AT 2.5m)
// ============================================================
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duration = pulseIn(echoPin, HIGH, 15000); 
  
  if (duration == 0) return 250.0; 
  return (duration * 0.0343) / 2.0;
}
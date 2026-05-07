void setup() {
  // 1. USB Serial to PC (Open Serial Monitor at 115200)
  Serial.begin(115200); 
  
  // 2. Hardware Serial to STM32 (Pins 16 and 17)
  // RX=16 (Connect to STM32 TX), TX=17 (Connect to STM32 RX)
  Serial2.begin(115200, SERIAL_8N1, 16, 17); 
  
  // Set a short timeout so the code doesn't hang if a newline is missing
  Serial2.setTimeout(50);

  Serial.println("--- ESP32 UART Monitor Started ---");
  Serial.println("Make sure STM32 is powered and Ground is shared.");
}

void loop() {
  // Check if data has arrived from the STM32
  if (Serial2.available()) {
    // Read until newline or timeout
    String stm32Data = Serial2.readStringUntil('\n');
    
    // Clean up any weird characters (like \r)
    stm32Data.trim();

    if (stm32Data.length() > 0) {
      Serial.print("[STM32 Says]: ");
      Serial.println(stm32Data);
    }
  }

  // Optional: Send a test byte to STM32 every 2 seconds
  static uint32_t lastMillis = 0;
  if (millis() - lastMillis > 2000) {
    Serial2.println("ESP32_ALIVE");
    lastMillis = millis();
  }
}
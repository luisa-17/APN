/*
 * ════════════════════════════════════════════════════════════════
 *  SIMPLE VOLTAGE SENSOR TEST
 * ════════════════════════════════════════════════════════════════
 * 
 *  Sensor: Voltage Sensor Module (with voltage divider built-in)
 *          OR DIY voltage divider (27kΩ + 10kΩ)
 * 
 *  Wiring:
 *    Sensor VCC  →  ESP32 3.3V or 5V (check your module)
 *    Sensor GND  →  ESP32 GND
 *    Sensor OUT  →  ESP32 GPIO 32
 *    Sensor IN+  →  Voltage to measure (e.g., 12V battery +)
 *    Sensor IN-  →  GND of voltage source
 * 
 *  For testing WITHOUT 12V:
 *    Just connect OUT to GPIO 32, power the module
 *    Leave IN+/IN- disconnected (should read ~0V)
 * 
 * ════════════════════════════════════════════════════════════════
 */

#define VOLTAGE_PIN 32

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  analogReadResolution(12);
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);
  
  Serial.println("\n========================================");
  Serial.println("   SIMPLE VOLTAGE SENSOR TEST");
  Serial.println("========================================");
  Serial.println("Pin: GPIO 32");
  Serial.println("========================================\n");
}

void loop() {
  // Read sensor
  int raw = analogRead(VOLTAGE_PIN);
  float voltage = (raw * 3.3) / 4095.0;
  
  // Calculate input voltage (if using voltage divider)
  // For 27k + 10k divider: multiply by 3.7
  // For 30k + 7.5k divider: multiply by 5
  // Adjust based on your module
  float inputVoltage = voltage * 3.7;  // Change multiplier for your divider
  
  Serial.println("────────────────────────────────────");
  Serial.print("Raw ADC:      ");
  Serial.println(raw);
  Serial.print("ADC Voltage:  ");
  Serial.print(voltage, 3);
  Serial.println(" V");
  Serial.print("Input Voltage: ");
  Serial.print(inputVoltage, 2);
  Serial.println(" V");
  Serial.println("────────────────────────────────────");
  
  // Simple status
  if (raw < 50) {
    Serial.println("Status: ⚪ No voltage / Disconnected");
  } 
  else if (raw >= 50 && raw < 4000) {
    Serial.println("Status: ✅ Sensor is working!");
  } 
  else if (raw >= 4000) {
    Serial.println("Status: ⚠️  Reading maxed out (too high)");
  }
  
  Serial.println("\n");
  
  delay(1000);
}
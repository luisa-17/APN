/*
 * ========================================
 * TEMPERATURE SENSOR TEST - 2x LM35
 * ========================================
 * LM35 outputs 10mV per °C
 * 
 * Wiring:
 *   Pin 1 (left)   → VCC (5V)
 *   Pin 2 (middle) → Signal → GPIO
 *   Pin 3 (right)  → GND
 * 
 * Pins:
 *   Sensor 1 → GPIO 26
 *   Sensor 2 → GPIO 27
 * 
 * Note: These are ADC2 pins - read before WiFi init
 * ========================================
 */

#define TEMP_SENSOR_1  25
#define TEMP_SENSOR_2  26

// LM35 settings
const float ADC_REF = 3.3;
const int ADC_RES = 4095;
const float MV_PER_C = 0.01;  // 10mV per °C

// Temperature thresholds
const float TEMP_WARNING = 45.0;
const float TEMP_CRITICAL = 60.0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  analogReadResolution(12);
  analogSetPinAttenuation(TEMP_SENSOR_1, ADC_11db);
  analogSetPinAttenuation(TEMP_SENSOR_2, ADC_11db);
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║   TEMPERATURE SENSOR TEST - 2x LM35    ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║ Pins:                                  ║");
  Serial.println("║   Sensor 1: GPIO 26                    ║");
  Serial.println("║   Sensor 2: GPIO 27                    ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║ Scale: 10mV per °C                     ║");
  Serial.println("║ Max readable: ~330°C                   ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();
}

float readTemp(int pin) {
  long sum = 0;
  for (int i = 0; i < 20; i++) {
    sum += analogRead(pin);
    delay(5);
  }
  float avg = sum / 20.0;
  float voltage = (avg * ADC_REF) / ADC_RES;
  float tempC = voltage / MV_PER_C;
  return tempC;
}

void printTempStatus(float t) {
  Serial.print("│   Status: ");
  if (t < 0) Serial.println("❄️  Freezing            │");
  else if (t < 20) Serial.println("🌡️ Cool                │");
  else if (t < 35) Serial.println("✅ Normal              │");
  else if (t < TEMP_WARNING) Serial.println("🟡 Warm                │");
  else if (t < TEMP_CRITICAL) Serial.println("🟠 HOT - Warning!      │");
  else Serial.println("🔴 CRITICAL - Danger!  │");
}

void printTempBar(float temp) {
  int bars = map(constrain(temp * 10, 0, 600), 0, 600, 0, 15);
  Serial.print("[");
  for (int i = 0; i < 15; i++) {
    if (i < bars) {
      if (temp > TEMP_CRITICAL) Serial.print("█");
      else if (temp > TEMP_WARNING) Serial.print("▓");
      else Serial.print("░");
    } else {
      Serial.print("░");
    }
  }
  Serial.print("] ");
  Serial.print(temp, 1);
  Serial.print("°C");
}

void loop() {
  // Read raw ADC values
  int raw1 = analogRead(TEMP_SENSOR_1);
  int raw2 = analogRead(TEMP_SENSOR_2);
  
  // Read temperatures
  float t1 = readTemp(TEMP_SENSOR_1);
  float t2 = readTemp(TEMP_SENSOR_2);
  
  // Convert to Fahrenheit
  float t1f = (t1 * 9.0 / 5.0) + 32.0;
  float t2f = (t2 * 9.0 / 5.0) + 32.0;
  
  // Average
  float avgTemp = (t1 + t2) / 2.0;
  
  Serial.println("┌────────────────────────────────────┐");
  Serial.println("│    TEMPERATURE SENSOR READINGS     │");
  Serial.println("├────────────────────────────────────┤");
  
  // Sensor 1
  Serial.println("│ --- Sensor 1 (GPIO 26) ---         │");
  Serial.print("│   Raw ADC: "); Serial.println(raw1);
  Serial.print("│   Temp: ");
  Serial.print(t1, 1); Serial.print("°C / ");
  Serial.print(t1f, 1); Serial.println("°F");
  printTempStatus(t1);
  
  // Sensor 2
  Serial.println("│ --- Sensor 2 (GPIO 27) ---         │");
  Serial.print("│   Raw ADC: "); Serial.println(raw2);
  Serial.print("│   Temp: ");
  Serial.print(t2, 1); Serial.print("°C / ");
  Serial.print(t2f, 1); Serial.println("°F");
  printTempStatus(t2);
  
  Serial.println("├────────────────────────────────────┤");
  
  // Summary
  Serial.print("│ Average: "); Serial.print(avgTemp, 1); Serial.println("°C");
  Serial.print("│ Difference: "); Serial.print(abs(t1 - t2), 1); Serial.println("°C");
  
  Serial.println("├────────────────────────────────────┤");
  
  // Visual bars
  Serial.print("│ T1: "); printTempBar(t1); Serial.println("      │");
  Serial.print("│ T2: "); printTempBar(t2); Serial.println("      │");
  
  Serial.println("└────────────────────────────────────┘");
  
  // High temperature alert
  if (t1 > TEMP_CRITICAL || t2 > TEMP_CRITICAL) {
    Serial.println();
    Serial.println("🔥🔥🔥 CRITICAL TEMPERATURE! 🔥🔥🔥");
  } else if (t1 > TEMP_WARNING || t2 > TEMP_WARNING) {
    Serial.println();
    Serial.println("⚠️ High temperature warning!");
  }
  
  Serial.println();
  
  delay(1000);
}
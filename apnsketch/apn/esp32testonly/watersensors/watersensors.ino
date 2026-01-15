/*
 * ========================================
 * WATER SENSOR TEST - 4x Rain Sensor Modules
 * ========================================
 * Using Digital Output (DO) from LM393 module
 * 
 * Wiring per sensor:
 *   VCC → 3.3V or 5V
 *   GND → GND
 *   DO  → GPIO (we use this)
 *   AO  → Not used
 * 
 * DO Output:
 *   HIGH (1) = DRY
 *   LOW  (0) = WET
 * 
 * Pins:
 *   Sensor 1 → GPIO 15
 *   Sensor 2 → GPIO 2
 *   Sensor 3 → GPIO 4
 *   Sensor 4 → GPIO 5
 * ========================================
 */

#define WATER1_DO  15
#define WATER2_DO  17
#define WATER3_DO  4
#define WATER4_DO  5

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(WATER1_DO, INPUT);
  pinMode(WATER2_DO, INPUT);
  pinMode(WATER3_DO, INPUT);
  pinMode(WATER4_DO, INPUT);
  
  Serial.println("╔════════════════════════════════════════╗");
  Serial.println("║   WATER SENSOR TEST - 4 Sensors        ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║ Pins:                                  ║");
  Serial.println("║   Sensor 1: GPIO 15                    ║");
  Serial.println("║   Sensor 2: GPIO 2                     ║");
  Serial.println("║   Sensor 3: GPIO 4                     ║");
  Serial.println("║   Sensor 4: GPIO 5                     ║");
  Serial.println("╠════════════════════════════════════════╣");
  Serial.println("║ Logic: HIGH=DRY, LOW=WET               ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println();
}

void loop() {
  // Read sensors (inverted logic: LOW = water detected)
  bool water1 = !digitalRead(WATER1_DO);
  bool water2 = !digitalRead(WATER2_DO);
  bool water3 = !digitalRead(WATER3_DO);
  bool water4 = !digitalRead(WATER4_DO);
  
  int waterCount = water1 + water2 + water3 + water4;
  
  Serial.println("┌────────────────────────────────────┐");
  Serial.println("│      WATER SENSOR READINGS         │");
  Serial.println("├────────────────────────────────────┤");
  
  Serial.print("│ Sensor 1 (GPIO 15): ");
  Serial.println(water1 ? "💧 WET!        │" : "✅ DRY         │");
  
  Serial.print("│ Sensor 2 (GPIO 2):  ");
  Serial.println(water2 ? "💧 WET!        │" : "✅ DRY         │");
  
  Serial.print("│ Sensor 3 (GPIO 4):  ");
  Serial.println(water3 ? "💧 WET!        │" : "✅ DRY         │");
  
  Serial.print("│ Sensor 4 (GPIO 5):  ");
  Serial.println(water4 ? "💧 WET!        │" : "✅ DRY         │");
  
  Serial.println("├────────────────────────────────────┤");
  
  // Visual status
  Serial.print("│ Status: [");
  Serial.print(water1 ? "W" : "-");
  Serial.print("][");
  Serial.print(water2 ? "W" : "-");
  Serial.print("][");
  Serial.print(water3 ? "W" : "-");
  Serial.print("][");
  Serial.print(water4 ? "W" : "-");
  Serial.print("]  ");
  Serial.print(waterCount);
  Serial.println("/4 wet     │");
  
  Serial.println("├────────────────────────────────────┤");
  
  // Alert level
  Serial.print("│ Alert: ");
  if (waterCount == 0) {
    Serial.println("✅ ALL CLEAR              │");
  } else if (waterCount == 1) {
    Serial.println("⚠️  Water detected!        │");
  } else if (waterCount <= 3) {
    Serial.println("🟠 Multiple sensors wet!   │");
  } else {
    Serial.println("🔴 FLOODING - ALL WET!     │");
  }
  
  Serial.println("└────────────────────────────────────┘");
  Serial.println();
  
  delay(1000);
}
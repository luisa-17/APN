
/*
 * ════════════════════════════════════════════════════════════════
 *  SIMPLE GAS SENSOR TEST - MQ Series
 * ════════════════════════════════════════════════════════════════
 * 
 * Wiring:
 *   VCC → 5V (IMPORTANT: Heater needs 5V, not 3.3V!)
 *   GND → GND
 *   AO  → GPIO 25 (Analog Output)
 *   DO  → Not used (Digital Output - optional)
 * 
 * Calibration Steps:
 *   1. Power on sensor in CLEAN AIR
 *   2. Wait 3+ minutes for warmup
 *   3. Note the "baseline" reading
 *   4. Adjust potentiometer if needed (clockwise = less sensitive)
 *   5. Gas detection = reading significantly ABOVE baseline
 * 
 * Pin: GPIO 25
 * ════════════════════════════════════════════════════════════════
 */

#define GAS_SENSOR  27

// ══════════════════════════════════════════════════════════════
//  CALIBRATION VALUES - ADJUST THESE BASED ON YOUR ENVIRONMENT!
// ══════════════════════════════════════════════════════════════

// Baseline is set automatically after warmup
// Detection threshold = baseline + DETECTION_MARGIN
const int DETECTION_MARGIN = 300;    // How much above baseline = gas detected
const int WARNING_MARGIN = 600;      // Warning level above baseline
const int DANGER_MARGIN = 1000;      // Danger level above baseline

// Warmup time in seconds (minimum 180 = 3 minutes)
const int WARMUP_TIME = 180;

// ══════════════════════════════════════════════════════════════
//  VARIABLES
// ══════════════════════════════════════════════════════════════

unsigned long startTime;
bool warmedUp = false;
bool calibrated = false;

int baseline = 0;           // Clean air reading (set after warmup)
int currentReading = 0;     // Current sensor reading
int deviation = 0;          // How much above/below baseline

// For stable baseline calculation
int baselineReadings[10];
int baselineIndex = 0;
int stableCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  startTime = millis();
  
  // Configure ADC
  analogReadResolution(12);  // 12-bit resolution (0-4095)
  analogSetPinAttenuation(GAS_SENSOR, ADC_11db);  // Full 0-3.3V range
  
  printHeader();
}

void printHeader() {
  Serial.println("\n");
  Serial.println("╔════════════════════════════════════════════════════════╗");
  Serial.println("║         SIMPLE GAS SENSOR TEST - MQ Series             ║");
  Serial.println("╠════════════════════════════════════════════════════════╣");
  Serial.println("║  Pin: GPIO 25                                          ║");
  Serial.println("║  Power: 5V (important for heater!)                     ║");
  Serial.println("╠════════════════════════════════════════════════════════╣");
  Serial.println("║  ⚠️  KEEP SENSOR IN CLEAN AIR DURING WARMUP!           ║");
  Serial.println("║  ⚠️  Wait for baseline calibration before testing      ║");
  Serial.println("╠════════════════════════════════════════════════════════╣");
  Serial.println("║  TIP: If too sensitive, turn potentiometer CLOCKWISE   ║");
  Serial.println("╚════════════════════════════════════════════════════════╝");
  Serial.println();
}

int readGasSensor() {
  // Take multiple readings and average for stability
  long sum = 0;
  const int samples = 30;
  
  for (int i = 0; i < samples; i++) {
    sum += analogRead(GAS_SENSOR);
    delay(2);
  }
  
  return sum / samples;
}

void calibrateBaseline() {
  // Collect readings for stable baseline
  baselineReadings[baselineIndex] = currentReading;
  baselineIndex = (baselineIndex + 1) % 10;
  
  // Check if readings are stable (within ±50 of each other)
  if (baselineIndex == 0) {
    int minVal = baselineReadings[0];
    int maxVal = baselineReadings[0];
    long sum = 0;
    
    for (int i = 0; i < 10; i++) {
      sum += baselineReadings[i];
      if (baselineReadings[i] < minVal) minVal = baselineReadings[i];
      if (baselineReadings[i] > maxVal) maxVal = baselineReadings[i];
    }
    
    // If readings are stable (range < 100), set baseline
    if (maxVal - minVal < 100) {
      baseline = sum / 10;
      calibrated = true;
      
      Serial.println();
      Serial.println("╔════════════════════════════════════════════════════════╗");
      Serial.println("║         ✅ CALIBRATION COMPLETE!                       ║");
      Serial.println("╠════════════════════════════════════════════════════════╣");
      Serial.print("║  Baseline (clean air): ");
      Serial.print(baseline);
      Serial.println("                            ║");
      Serial.print("║  Gas Detection at:     > ");
      Serial.print(baseline + DETECTION_MARGIN);
      Serial.println("                         ║");
      Serial.print("║  Warning Level at:     > ");
      Serial.print(baseline + WARNING_MARGIN);
      Serial.println("                         ║");
      Serial.print("║  Danger Level at:      > ");
      Serial.print(baseline + DANGER_MARGIN);
      Serial.println("                         ║");
      Serial.println("╠════════════════════════════════════════════════════════╣");
      Serial.println("║  You can now expose sensor to gas for testing!        ║");
      Serial.println("╚════════════════════════════════════════════════════════╝");
      Serial.println();
    }
  }
}

String getGasStatus() {
  if (!calibrated) {
    return "CALIBRATING";
  }
  
  deviation = currentReading - baseline;
  
  if (deviation < DETECTION_MARGIN) {
    return "CLEAN";
  } else if (deviation < WARNING_MARGIN) {
    return "GAS DETECTED";
  } else if (deviation < DANGER_MARGIN) {
    return "WARNING";
  } else {
    return "DANGER";
  }
}

void loop() {
  // Read sensor
  currentReading = readGasSensor();
  
  // Calculate elapsed time
  unsigned long elapsedSec = (millis() - startTime) / 1000;
  int warmupRemaining = WARMUP_TIME - elapsedSec;
  
  // Check warmup status
  if (warmupRemaining <= 0 && !warmedUp) {
    warmedUp = true;
    Serial.println("\n✅ Warmup complete! Starting calibration...\n");
  }
  
  // Calibrate baseline after warmup
  if (warmedUp && !calibrated) {
    calibrateBaseline();
  }
  
  // Calculate deviation from baseline
  if (calibrated) {
    deviation = currentReading - baseline;
  }
  
  // Get status
  String status = getGasStatus();
  
  // ══════════════════════════════════════════════════════════════
  //  DISPLAY OUTPUT
  // ══════════════════════════════════════════════════════════════
  
  Serial.println("┌──────────────────────────────────────────────────────┐");
  Serial.println("│              GAS SENSOR READING                      │");
  Serial.println("├──────────────────────────────────────────────────────┤");
  
  // Raw reading
  Serial.print("│  Raw Value:     ");
  Serial.print(currentReading);
  printPadding(currentReading);
  
  // Voltage
  float voltage = currentReading * (3.3 / 4095.0);
  Serial.print("│  Voltage:       ");
  Serial.print(voltage, 2);
  Serial.println(" V                                │");
  
  // Warmup or calibration status
  if (!warmedUp) {
    Serial.println("├──────────────────────────────────────────────────────┤");
    Serial.print("│  ⏱️  WARMING UP: ");
    Serial.print(warmupRemaining);
    Serial.println(" seconds remaining                  │");
    
    // Progress bar
    int progress = map(elapsedSec, 0, WARMUP_TIME, 0, 30);
    Serial.print("│  [");
    for (int i = 0; i < 30; i++) {
      if (i < progress) Serial.print("█");
      else Serial.print("░");
    }
    Serial.println("]               │");
    Serial.println("│                                                      │");
    Serial.println("│  ⚠️  Keep sensor in CLEAN AIR during warmup!         │");
  }
  else if (!calibrated) {
    Serial.println("├──────────────────────────────────────────────────────┤");
    Serial.println("│  🔄 CALIBRATING... Waiting for stable readings       │");
    Serial.println("│     Keep sensor in CLEAN AIR!                        │");
  }
  else {
    // Calibrated - show full info
    Serial.println("├──────────────────────────────────────────────────────┤");
    
    // Baseline
    Serial.print("│  Baseline:      ");
    Serial.print(baseline);
    Serial.println("  (clean air reference)               │");
    
    // Deviation
    Serial.print("│  Deviation:     ");
    if (deviation >= 0) Serial.print("+");
    Serial.print(deviation);
    printDeviationPadding(deviation);
    
    Serial.println("├──────────────────────────────────────────────────────┤");
    
    // Status with icon
    Serial.print("│  STATUS:        ");
    
    if (status == "CLEAN") {
      Serial.println("✅ CLEAN - No gas detected              │");
    } else if (status == "GAS DETECTED") {
      Serial.println("🟡 GAS DETECTED - Low level             │");
    } else if (status == "WARNING") {
      Serial.println("🟠 WARNING - Significant gas level!     │");
    } else if (status == "DANGER") {
      Serial.println("🔴 DANGER - High gas concentration!     │");
    }
    
    // Visual level bar
    Serial.println("├──────────────────────────────────────────────────────┤");
    Serial.print("│  Level: ");
    printGasBar(deviation);
  }
  
  Serial.println("└──────────────────────────────────────────────────────┘");
  
  // Alert for high gas
  if (calibrated && status == "DANGER") {
    Serial.println();
    Serial.println("  🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨");
    Serial.println("  🚨                                      🚨");
    Serial.println("  🚨   DANGER! HIGH GAS CONCENTRATION!    🚨");
    Serial.println("  🚨   Ventilate area immediately!        🚨");
    Serial.println("  🚨                                      🚨");
    Serial.println("  🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨");
  }
  
  Serial.println();
  
  delay(1000);  // Update every second
}

void printPadding(int value) {
  if (value < 10) Serial.println("                                   │");
  else if (value < 100) Serial.println("                                  │");
  else if (value < 1000) Serial.println("                                 │");
  else Serial.println("                                │");
}

void printDeviationPadding(int value) {
  int absVal = abs(value);
  if (absVal < 10) Serial.println("                                    │");
  else if (absVal < 100) Serial.println("                                   │");
  else if (absVal < 1000) Serial.println("                                  │");
  else Serial.println("                                 │");
}

void printGasBar(int deviation) {
  // Map deviation to bar length (0 to 30 bars)
  int maxDeviation = DANGER_MARGIN + 500;
  int bars = map(constrain(deviation, 0, maxDeviation), 0, maxDeviation, 0, 30);
  
  Serial.print("[");
  
  for (int i = 0; i < 30; i++) {
    if (i < bars) {
      // Color code the bars
      if (i < 10) Serial.print("░");       // Clean zone
      else if (i < 20) Serial.print("▒");  // Warning zone
      else Serial.print("█");               // Danger zone
    } else {
      Serial.print(" ");
    }
  }
  
  Serial.println("]          │");
  
  // Legend
  Serial.println("│          |----CLEAN----|--WARNING--|--DANGER--|          │");
}

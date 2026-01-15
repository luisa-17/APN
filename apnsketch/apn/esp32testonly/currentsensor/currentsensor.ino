/*
  ESP32 + Voltage Sensor + ACS712T-5A (with 68k + 100k divider)

  Wiring summary:

  VOLTAGE SENSOR (to measure ~0–25V):
    OUT  -> GPIO 34 (VOLTAGE_PIN)
    VCC  -> 5V
    GND  -> GND

  ACS712T-5A CURRENT SENSOR:
    VCC  -> 5V
    GND  -> GND
    OUT  -> 68k ->*JUNCTION*-> GPIO 35 (CURRENT_PIN)
                             -> 100k -> GND

    12V(+) -> IP+ / IP-  (depending on how you wired, but current must pass THROUGH the sensor)
    Sensor other terminal -> Load(+) -> Load(-) -> 12V(-)

  Notes:
    - Make sure all grounds are common: ESP32 GND, 12V(-), sensor GND.
    - Run with NO LOAD connected during the ACS zero calibration (at startup).
*/

#define VOLTAGE_PIN 34
#define CURRENT_PIN 35

// -------- ADC SETTINGS --------
const float ADC_REF = 3.3;        // ESP32 ADC reference (approx.)
const int   ADC_MAX = 4095;       // 12-bit ADC

// -------- VOLTAGE SENSOR SETTINGS --------
// Your voltage sensor is scaled for 0–25V input (typical module or divider).
// VOLT_CAL is a calibration factor to match the multimeter.
const float INPUT_MAX_VOLTAGE = 25.0;   // voltage at sensor input for full-scale ADC
const float VOLT_CAL          = 0.59;   // CALIBRATION: adjust so code matches your multimeter

// -------- ACS712-5A CURRENT SENSOR SETTINGS --------
const float DIVIDER_RATIO = 0.595f; 

// UPDATED CALIBRATION
const float ACS_SENS = 0.18f;   // Lower value = Higher current reading

// ACS zero (midpoint) will be measured at startup (no load)
float acsZero = 0.0f;

// Threshold below which current is considered noise
const float CURRENT_NOISE_THRESHOLD = 0.20f;  // 0.15A

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);
  analogSetPinAttenuation(CURRENT_PIN, ADC_11db);

  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(CURRENT_PIN, INPUT);

  Serial.println(F("Voltage + Current Sensor Monitor"));
  Serial.println(F("================================"));
  Serial.println(F("IMPORTANT: Make sure NO LOAD is connected during calibration."));
  Serial.println(F("Calibration starts in 2 seconds...\n"));
  delay(2000);

  calibrateACSZero();
}

void calibrateACSZero() {
  const int N = 2000;
  long sum_mV = 0;  // sum in mV to keep precision

  Serial.println(F("Calibrating ACS712 zero point..."));

  for (int i = 0; i < N; i++) {
    int rawI = analogRead(CURRENT_PIN);

    // Voltage at ESP32 pin from divider
    float vAdc = (rawI * ADC_REF) / ADC_MAX;   // V at GPIO 35

    // Undo the 68k/100k divider to get actual ACS712 output
    float vSensor = vAdc / DIVIDER_RATIO;      // V at ACS OUT pin

    sum_mV += (long)(vSensor * 1000.0f);       // accumulate in mV
    delay(2);
  }

  acsZero = (sum_mV / (float)N) / 1000.0f;     // back to volts

  Serial.print(F("Calibrated ACS zero point: "));
  Serial.print(acsZero, 3);
  Serial.println(F(" V"));
  Serial.println(F("--------------------------------"));
}

float readVoltage() {
  const int NS = 100;
  long sum = 0;

  for (int i = 0; i < NS; i++) {
    sum += analogRead(VOLTAGE_PIN);
    delayMicroseconds(100);
  }

  float rawAvg = sum / (float)NS;

  // Map ADC reading to input voltage range
  float voltage = (rawAvg / ADC_MAX) * INPUT_MAX_VOLTAGE;
  voltage *= VOLT_CAL; 

  // Update this filter to hide the 2.5V ghost reading
  if (voltage < 3.0f) {
    voltage = 0.0f;
  }
  return voltage;
}

float readCurrent() {
  const int NS = 500;
  long sum = 0;

  for (int i = 0; i < NS; i++) {
    sum += analogRead(CURRENT_PIN);
    delayMicroseconds(80);
  }

  float rawAvg = sum / (float)NS;

  // Voltage at ESP32 pin (after divider)
  float vAdc = (rawAvg * ADC_REF) / ADC_MAX;  // V at GPIO 35

  // Undo divider to get actual ACS712 output voltage
  float vSensor = vAdc / DIVIDER_RATIO;      // V at ACS OUT pin

  // If your ACS output goes DOWN with current (as seen in your data):
  float current = (acsZero - vSensor) / ACS_SENS;  // A

  // Always positive for now
  current = fabs(current);

  // Noise filter
  if (current < CURRENT_NOISE_THRESHOLD) {
    current = 0.0f;
  }

  return current;
}

void loop() {
  float voltage = readVoltage();
  float current = readCurrent();
  float power   = voltage * current;

  Serial.print(F("Voltage: "));
  Serial.print(voltage, 2);
  Serial.print(F(" V  |  "));

  Serial.print(F("Current: "));
  Serial.print(current, 3);
  Serial.print(F(" A  ("));
  Serial.print(current * 1000.0f, 0);
  Serial.print(F(" mA)  |  "));

  Serial.print(F("Power: "));
  Serial.print(power, 2);
  Serial.println(F(" W"));

  delay(1000);
}
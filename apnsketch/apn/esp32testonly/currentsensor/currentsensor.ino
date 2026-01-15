// -------- PINS --------
#define VOLTAGE_PIN 34
#define CURRENT_PIN 35

// -------- ADC --------
const float ADC_REF = 3.3;
const int ADC_MAX = 4095;

// -------- VOLTAGE SENSOR --------
const float INPUT_MAX_VOLTAGE = 25.0;
const float VOLT_CAL = 1.15;   // adjust later if needed

// -------- ACS712-5A --------
const float ACS_ZERO = 2.50;   // V at 0A
const float ACS_SENS = 0.185;  // V per A (5A version)

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);
  analogSetPinAttenuation(VOLTAGE_PIN, ADC_11db);
  analogSetPinAttenuation(CURRENT_PIN, ADC_11db);

  Serial.println("Voltage + Current Sensor Test");
  Serial.println("--------------------------------");
}

void loop() {
  // ----- VOLTAGE -----
  int rawV = analogRead(VOLTAGE_PIN);
  float voltage = (rawV * ADC_REF / ADC_MAX) * (INPUT_MAX_VOLTAGE / ADC_REF);
  voltage *= VOLT_CAL;

  // ----- CURRENT -----
  int rawI = analogRead(CURRENT_PIN);
  float vOut = (rawI * ADC_REF) / ADC_MAX;
  float current = (vOut - ACS_ZERO) / ACS_SENS;

  // ----- OUTPUT -----
  Serial.print("V_ADC: ");
  Serial.print(rawV);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V");

  Serial.print(" || I_ADC: ");
  Serial.print(rawI);
  Serial.print(" | Current: ");
  Serial.print(current, 3);
  Serial.println(" A");

  delay(1000);
}
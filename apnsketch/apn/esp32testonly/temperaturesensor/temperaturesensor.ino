// TEST: TEMP SENSOR ON GPIO 26 (NO WIFI)
#define TEMP_PIN 26  // your TEMP1 pin

void setup() {
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(TEMP_PIN, INPUT);

  Serial.println("TEMP SENSOR TEST ON GPIO 26 (NO WIFI)");
}

void loop() {
  int raw = analogRead(TEMP_PIN);
  float voltage = raw * 3.3 / 4095.0;      // ADC -> Volts
  float tempC   = voltage * 100.0;         // e.g. LM35-style: 10mV/°C

  Serial.print("RAW: ");
  Serial.print(raw);
  Serial.print(" | V: ");
  Serial.print(voltage, 3);
  Serial.print(" V | T: ");
  Serial.print(tempC, 1);
  Serial.println(" °C");

  delay(1000);
}
#include <Wire.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\nI2C Scanner");
  Wire.begin(21, 22);  // SDA=21, SCL=22

  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found device at 0x");
      Serial.print(addr, HEX);
      if (addr == 0x68) Serial.print("  <- MPU6050 (AD0=GND)");
      if (addr == 0x69) Serial.print("  <- MPU6050 (AD0=VCC)");
      Serial.println();
      count++;
      delay(5);
    }
  }

  if (count == 0) {
    Serial.println("No I2C devices found. Check wiring.");
  }
}

void loop() {}
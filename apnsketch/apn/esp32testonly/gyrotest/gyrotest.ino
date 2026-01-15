#include <Wire.h>

#define MPU_ADDR 0x68  // I2C address of the MPU-6050

int16_t gx, gy, gz;
const float GYRO_SHAKE_THRESHOLD = 50.0; // deg/s, adjust as needed

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // ESP32 default I2C pins: SDA=21, SCL=22

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Set to zero (wakes up the MPU-6050)
  Wire.endTransmission();

  Serial.println("MPU6050 Gyro Only Test");
}

void loop() {
  // Request gyro data from MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // Starting register for Gyro readings (GYRO_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  // Convert to deg/s
  float gxD = gx / 131.0;
  float gyD = gy / 131.0;
  float gzD = gz / 131.0;

  Serial.print("Gyro (deg/s): X=");
  Serial.print(gxD, 1);
  Serial.print(" Y=");
  Serial.print(gyD, 1);
  Serial.print(" Z=");
  Serial.print(gzD, 1);

  // Check for shake
  if (abs(gxD) > GYRO_SHAKE_THRESHOLD ||
      abs(gyD) > GYRO_SHAKE_THRESHOLD ||
      abs(gzD) > GYRO_SHAKE_THRESHOLD) {
    Serial.print("  <-- SHAKING!");
  }

  Serial.println();
  delay(200);
}
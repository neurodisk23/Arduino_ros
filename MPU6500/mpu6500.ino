#include <Wire.h>

#define MPU_ADDR 0x68 // MPU6500 I2C address

void setup() {
  Serial.begin(9600);
  Wire.begin();
  
  // Initialize MPU6500
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // Wake up MPU6500
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); // Gyroscope configuration register
  Wire.write(0x08); // Set full scale range to +/- 500 degrees/s
  Wire.endTransmission(true);
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); // Accelerometer configuration register
  Wire.write(0x08); // Set full scale range to +/- 4g
  Wire.endTransmission(true);
}

void loop() {
  // Read accelerometer data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Starting register address for accelerometer readings
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU_ADDR, 6, true); // Request 6 bytes of data
  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  
  // Read gyroscope data
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); // Starting register address for gyroscope readings
  Wire.endTransmission(false);
  
  Wire.requestFrom(MPU_ADDR, 6, true); // Request 6 bytes of data
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();
  
  Serial.print("ax: "); Serial.print(ax);
  Serial.print(", ay: "); Serial.print(ay);
  Serial.print(", az: "); Serial.print(az);
  Serial.print(", gx: "); Serial.print(gx);
  Serial.print(", gy: "); Serial.print(gy);
  Serial.print(", gz: "); Serial.println(gz);
  
  delay(100);
}

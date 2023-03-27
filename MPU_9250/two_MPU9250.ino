#include <Wire.h>
#include <MPU9250.h>

#define MPU9250_ADDRESS_1 0x68 // I2C address of first MPU9250
#define MPU9250_ADDRESS_2 0x69 // I2C address of second MPU9250

MPU9250 imu1(Wire, MPU9250_ADDRESS_1); // Create an instance of the MPU9250 for the first sensor
MPU9250 imu2(Wire, MPU9250_ADDRESS_2); // Create an instance of the MPU9250 for the second sensor

void setup() {
  Serial.begin(115200);

  Wire.begin();

  imu1.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imu1.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  imu1.setFilterBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  imu1.begin();

  imu2.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  imu2.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  imu2.setFilterBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
  imu2.begin();
}

void loop() {
  imu1.readSensor();
  imu2.readSensor();

  Serial.print("Sensor 1: ");
  Serial.print("Acc X: ");
  Serial.print(imu1.getAccelX_mss());
  Serial.print("\tY: ");
  Serial.print(imu1.getAccelY_mss());
  Serial.print("\tZ: ");
  Serial.print(imu1.getAccelZ_mss());
  Serial.print("\tGyro X: ");
  Serial.print(imu1.getGyroX_rads());
  Serial.print("\tY: ");
  Serial.print(imu1.getGyroY_rads());
  Serial.print("\tZ: ");
  Serial.print(imu1.getGyroZ_rads());

  Serial.print("\tSensor 2: ");
  Serial.print("Acc X: ");
  Serial.print(imu2.getAccelX_mss());
  Serial.print("\tY: ");
  Serial.print(imu2.getAccelY_mss());
  Serial.print("\tZ: ");
  Serial.print(imu2.getAccelZ_mss());
  Serial.print("\tGyro X: ");
  Serial.print(imu2.getGyroX_rads());
  Serial.print("\tY: ");
  Serial.print(imu2.getGyroY_rads());
  Serial.print("\tZ: ");
  Serial.print(imu2.getGyroZ_rads());

  Serial.println("");
  
  delay(100);
}

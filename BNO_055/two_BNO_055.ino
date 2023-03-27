#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno1 = Adafruit_BNO055(55, 0x28);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29);

void setup() {
  Serial.begin(9600);

  // Initialize first BNO055 sensor
  if(!bno1.begin())
  {
    Serial.print("Ooops, no BNO055 detected at I2C address 0x28 ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  bno1.setExtCrystalUse(true);

  // Initialize second BNO055 sensor
  if(!bno2.begin())
  {
    Serial.print("Ooops, no BNO055 detected at I2C address 0x29 ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  bno2.setExtCrystalUse(true);
}

void loop() {
  // Read data from first sensor
  sensors_event_t event1;
  bno1.getEvent(&event1);

  // Read accelerometer data
  sensors_event_t accelEvent1;
  bno1.getEvent(&accelEvent1, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  Serial.print("Sensor 1: ");
  Serial.print("Accelerometer X: ");
  Serial.print(accelEvent1.acceleration.x);
  Serial.print("\tY: ");
  Serial.print(accelEvent1.acceleration.y);
  Serial.print("\tZ: ");
  Serial.print(accelEvent1.acceleration.z);

  // Read gyro data
  sensors_event_t gyroEvent1;
  bno1.getEvent(&gyroEvent1, Adafruit_BNO055::VECTOR_GYROSCOPE);

  Serial.print("\tGyro X: ");
  Serial.print(gyroEvent1.gyro.x);
  Serial.print("\tY: ");
  Serial.print(gyroEvent1.gyro.y);
  Serial.print("\tZ: ");
  Serial.print(gyroEvent1.gyro.z);

  Serial.print("\t");

  // Read data from second sensor
  sensors_event_t event2;
  bno2.getEvent(&event2);

  // Read accelerometer data
  sensors_event_t accelEvent2;
  bno2.getEvent(&accelEvent2, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  Serial.print("Sensor 2: ");
  Serial.print("Accelerometer X: ");
  Serial.print(accelEvent2.acceleration.x);
  Serial.print("\tY: ");
  Serial.print(accelEvent2.acceleration.y);
  Serial.print("\tZ: ");
  Serial.print(accelEvent2.acceleration.z);

  // Read gyro data
  sensors_event_t gyroEvent2;
  bno2.getEvent(&gyroEvent2, Adafruit_BNO055::VECTOR_GYROSCOPE);

  Serial.print("\tGyro X: ");
  Serial.print(gyroEvent2.gyro.x);
  Serial.print("\tY: ");
  Serial.print(gyroEvent2.gyro.y);
  Serial.print("\tZ: ");
  Serial.print(gyroEvent2.gyro.z);

  Serial.println("");
  
  delay(100);
}

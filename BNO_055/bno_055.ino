#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055();

void setup() {
  Serial.begin(9600);
  
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  bno.setExtCrystalUse(true);
}

void loop() {
  sensors_event_t event;
  bno.getEvent(&event);

  // Read accelerometer data
  sensors_event_t accelEvent;
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  Serial.print("Accelerometer X: ");
  Serial.print(accelEvent.acceleration.x);
  Serial.print("\tY: ");
  Serial.print(accelEvent.acceleration.y);
  Serial.print("\tZ: ");
  Serial.print(accelEvent.acceleration.z);

  // Read gyro data
  sensors_event_t gyroEvent;
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

  Serial.print("\tGyro X: ");
  Serial.print(gyroEvent.gyro.x);
  Serial.print("\tY: ");
  Serial.print(gyroEvent.gyro.y);
  Serial.print("\tZ: ");
  Serial.print(gyroEvent.gyro.z);

  Serial.println("");
  
  delay(100);
}

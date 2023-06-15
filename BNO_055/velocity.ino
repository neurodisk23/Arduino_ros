#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.print('In the setup loop');
  myIMU.begin();
  delay(1000);
  int8_t temp = myIMU.getTemp();
  myIMU.setExtCrystalUse(true);
}


void loop() {
  uint8_t system, gyro_calib, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro_calib, &accel, &mg);
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  Serial.print(acc.x());
  Serial.print(",");
  Serial.print(acc.y());
  Serial.print(",");
  Serial.print(acc.z());
  Serial.print(",");
  Serial.print(gyro.x());
  Serial.print(",");
  Serial.print(gyro.y());
  Serial.print(",");
  Serial.print(gyro.z());
  Serial.print(",");
  Serial.print(mag.x());
  Serial.print(",");
  Serial.print(mag.y());
  Serial.print(",");
  Serial.print(mag.z());
  Serial.print(",");
  Serial.print(accel);
  Serial.print(",");
  Serial.print(gyro_calib);
  Serial.print(",");
  Serial.print(mg);
  Serial.print(",");
  Serial.println(system);
  if (system == 3) {digitalWrite(LED_BUILTIN, HIGH);}
  else {digitalWrite(LED_BUILTIN, LOW);}


}

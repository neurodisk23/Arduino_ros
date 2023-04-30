#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

float calib_acc_x, calib_acc_y, calib_acc_z, calib_gyro_x, calib_gyro_y, calib_gyro_z = 0;

bool run_once = true;
#define BNO055_SAMPLERATE_DELAY_MS (100)

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
  if (run_once == true) {

    for (int i = 0; i < 2000; i++) {
      imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
      imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      Serial.print("Calibration in progress ");
      Serial.print(i);
      calib_acc_x += acc.x();
      calib_acc_y += acc.y();
      calib_acc_z += acc.z();
      calib_gyro_x += gyro.x();
      calib_gyro_y += gyro.y();
      calib_gyro_z += gyro.z();
      Serial.print(" ");
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
      Serial.println(",");
      delay(50);
    }

    calib_acc_x /= 2000;
    calib_acc_y /= 2000;
    calib_acc_z /= 2000;
    calib_gyro_x /= 2000;
    calib_gyro_y /= 2000;
    calib_gyro_z /= 2000;

    run_once = false;
  }
  Serial.print(acc.x() - calib_acc_x);
  Serial.print(",");
  Serial.print(acc.y() - calib_acc_y);
  Serial.print(",");
  Serial.print(acc.z() - calib_acc_z);
  Serial.print(",");
  Serial.print(gyro.x() - calib_gyro_x);
  Serial.print(",");
  Serial.print(gyro.y() - calib_gyro_y);
  Serial.print(",");
  Serial.print(gyro.z() - calib_gyro_z);
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

  if (system == 3) {
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    digitalWrite(LED_BUILTIN, LOW);
  }

  delay(BNO055_SAMPLERATE_DELAY_MS);
}

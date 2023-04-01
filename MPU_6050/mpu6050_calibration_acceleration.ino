#include<Wire.h>
float RateRoll, RatePitch, RateYaw ;

float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw ;
int RateCalibrationNumber ;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch ;
float LoopTimer ;

float KalmanAngleRoll = 0 , KalmanUncertaintyAngleRoll = 2*2 ;
float KalmanAnglePitch = 0 , KalmanUncertaintyAnglePitch = 2*2 ;


float Kalman1DOutput[]={0,0};


void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState=KalmanState+0.004*KalmanInput;
  KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
  KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}
void gyro_signals(void) {

  Wire.beginTransmission(0x68);
  Wire.write(0x1A); // turn on low pass filter
  Wire.write(0x05); //  Low pass filter with a cuttoff freq of 10 Hz
  Wire.endTransmission();


  Wire.beginTransmission(0x68);
  Wire.write(0x1C); // turn on acceleration
  Wire.write(0x10); //  Low pass filter with a cuttoff freq of 10 Hz
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68,6);
  int16_t  AccXLSB = Wire.read() << 8| Wire.read();
  int16_t  AccYLSB = Wire.read() << 8| Wire.read();
  int16_t  AccZLSB = Wire.read() << 8| Wire.read();

  
  // Setting the sensitivity factor
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // turn on Sensitivity
  Wire.write(0x8); // Plus the sensitivity to +- 500degree/sec
  Wire.endTransmission() ;
  // Access the register storing the gyro value
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();


  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  //Convert the unit to degreee/sec
  RateRoll  = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw   = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB/4096 + 0.07;
  AccY = (float)AccYLSB/4096 - 0.12;
  AccZ = (float)AccZLSB/4096 - 0.03;

  AngleRoll  = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))*1/(3.142/180);
  AnglePitch = atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))*1/(3.142/180);
  


}




void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);

  Wire.endTransmission();

  for (RateCalibrationNumber = 0 ; RateCalibrationNumber < 2000 ; RateCalibrationNumber  ++)
  {

    gyro_signals();
    RateCalibrationRoll  += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw   += RateYaw ;
    delay(1);
  }

  RateCalibrationRoll  /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw   /= 2000;
  LoopTimer = micros();

}

void loop() {

  gyro_signals();
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw ;
  Serial.print(RateRoll);
  Serial.print(',');
  Serial.print(RatePitch);
  Serial.print(',');
  Serial.print(RateYaw);
  Serial.print(',');
  Serial.print(AccX);
  Serial.print(',');
  Serial.print(AccY);
  Serial.print(',');
  Serial.print(AccZ);
  Serial.print(',');
  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  Serial.print(KalmanAngleRoll);
  Serial.print(',');
  Serial.println(KalmanAnglePitch);
  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
  delay(50);

}

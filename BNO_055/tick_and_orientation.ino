#include<ros.h>
#include<std_msgs/Bool.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>


//#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 myIMU = Adafruit_BNO055();

ros::NodeHandle nh ;

std_msgs::Bool digital_msg_left ;
std_msgs::Bool digital_msg_right ;
std_msgs::Float32 orientation ;
std_msgs::Int8 imu_calibration_level ;

ros::Publisher digital_pub_left("digital_topic_left", &digital_msg_left);
ros::Publisher digital_pub_right("digital_topic_right", &digital_msg_right);
ros::Publisher orientation_pub("orientation", &orientation);
ros::Publisher imu_calib_status_pub("imu_calibraion_level", &imu_calibration_level);


const int digitalPin_left = 2;
const int digitalPin_right = 3;

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
  nh.advertise(digital_pub_left);
  nh.advertise(digital_pub_right);
  nh.advertise(orientation_pub);
  nh.advertise(imu_calib_status_pub);
  myIMU.begin();
  delay(1000);
  int8_t temp = myIMU.getTemp();
  myIMU.setExtCrystalUse(true);

}

void loop() {

  uint8_t system_, gyro_calib, accel, mg = 0;
  myIMU.getCalibration(&system_, &gyro_calib, &accel, &mg);
  imu::Vector<3> euler = myIMU.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> mag = myIMU.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  int value_left = digitalRead(digitalPin_left);
  int value_right  = digitalRead(digitalPin_right);


  digital_msg_left.data = (value_left == HIGH);
  digital_msg_right.data = (value_right == HIGH );
  orientation.data = euler.x();
  imu_calibration_level.data = system_ ; 
  

  digital_pub_right.publish(&digital_msg_right);
  digital_pub_left.publish(&digital_msg_left);
  orientation_pub.publish(&orientation);
  imu_calib_status_pub.publish(&imu_calibration_level);
  

  nh.spinOnce();

  delay(100);
  // put your main code here, to run repeatedly:

}

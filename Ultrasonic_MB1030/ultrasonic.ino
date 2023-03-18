#include "SonarEZ0pw.h"
#include <ros.h>
#include <std_msgs/Float32.h>
std_msgs::Float32 ultrasonic_distance;
ros::Publisher pub_distance("ultrasonic_dist", &ultrasonic_distance);
ros::NodeHandle nh;

SonarEZ0pw Sonar(7); // pin D7
float cm_dis=0.00;
void setup() {
// put your setup code here, to run once:
Serial.begin(9600);
nh.initNode() ;
nh.advertise(pub_distance) ;
}

void loop() {
// put your main code here, to run repeatedly:
ultrasonic_distance.data = Sonar.Distance(cm);
pub_distance.publish(&ultrasonic_distance);

Serial.println("Distance " );
//Serial.print(Inch_dis);
//Serial.println(" inch ");
Serial.print(cm_dis);
Serial.println(" cm ");

nh.spinOnce();
}

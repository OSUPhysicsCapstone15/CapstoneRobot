/* 
Arduino code to control the motors using input from ROS
 */

#include <ros.h>
#include <std_msgs/Float64.h>
// Initialize the motors
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md1;

ros::NodeHandle  nh;

void motorCallback( const std_msgs::Float64& msg){
  // Set the speeds given by ROS in the topic "Motors"
  mdl.setM1Speed(msg.leftMotor);
  mdl.setM2Speed(msg.rightMotor);
}

ros::Subscriber<std_msgs::Float64> sub("Motors", &motorCallback );

void setup()
{ 
  mdl.init();
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}


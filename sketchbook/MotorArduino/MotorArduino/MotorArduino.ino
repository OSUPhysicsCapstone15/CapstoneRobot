/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
// Include all the standard message types
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
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
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}


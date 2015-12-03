/* 
 Arduino code to control the motors using input from ROS
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <SoftwareSerial.h> 
// Initialize the motors
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md1;

const int bluetoothTx = 3;  // TX-O pin of bluetooth mate, Arduino D2
const int bluetoothRx = 5;  // RX-I pin of bluetooth mate, Arduino D3
boolean paused = false; // Whether or not the motors have received a paused command
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

ros::NodeHandle nh;
std_msgs::Float32 leftReturn_msg;
std_msgs::Float32 rightReturn_msg;
std_msgs::Bool paused_msg;
ros::Publisher pub_leftReturn("LeftReturn", &leftReturn_msg); // for nonmotor testing
ros::Publisher pub_rightReturn("RightReturn", &rightReturn_msg); // for nonmotor testing
ros::Publisher pub_paused("Paused", &paused_msg); // report a paused command


void leftMotor(const std_msgs::Float32& msg){
  // Set the left motor speed given by ROS in the topic "Motors"
  //leftReturn_msg.data = 0;
  //pub_leftReturn.publish( &leftReturn_msg);
}

void rightMotor(const std_msgs::Float32& msg){
  // Set the right motor speed given by ROS in the topic "M
  //rightReturn_msg.data = 0;
  //pub_rightReturn.publish( &rightReturn_msg);
}

ros::Subscriber<std_msgs::Float32> subLeft("LeftMotors", &leftMotor);
ros::Subscriber<std_msgs::Float32> subRight("RightMotors", &rightMotor);

void setup()
{ 
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); // LED is on unless we are moving
  nh.initNode();
  nh.subscribe(subLeft); // Subscribe to "LeftMotors"
  nh.subscribe(subRight); // Subscribe to "RightMotors"
  nh.advertise(pub_leftReturn); // publish left motor value for testing
  nh.advertise(pub_rightReturn); // publish right motor value for testing
  nh.advertise(pub_paused);
  paused = false;
}

void loop()
{
  nh.spinOnce(); // Check for updates with ROS
  delay(0.1);
}



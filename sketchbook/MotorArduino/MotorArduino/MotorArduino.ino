/* 
Arduino code to control the motors using input from ROS
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <SoftwareSerial.h> 
// Initialize the motors
#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md1;

const int bluetoothTx = 3;  // TX-O pin of bluetooth mate, Arduino D2
const int bluetoothRx = 5;  // RX-I pin of bluetooth mate, Arduino D3
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

ros::NodeHandle nh;

void leftMotor(const std_msgs::Float32& msg){
  // Set the left motor speed given by ROS in the topic "Motors"
md1.setM1Speed(msg.data);

}

void rightMotor(const std_msgs::Float32& msg){
  // Set the right motor speed given by ROS in the topic "Motors"
  md1.setM2Speed(msg.data);
}

ros::Subscriber<std_msgs::Float32> subLeft("LeftMotors", &leftMotor);
ros::Subscriber<std_msgs::Float32> subRight("RightMotors", &rightMotor);

void setup()
{ 
  pinMode(13, OUTPUT);
  bluetooth.begin(115200);
  // Initialize the motors and the ROS node
  md1.init();
  nh.initNode();
  nh.subscribe(subLeft); // Subscribe to "LeftMotors"
  nh.subscribe(subRight); // Subscribe to "RightMotors"
}

void loop()
{  
  if(bluetooth.available())
  {
    int command=(int)bluetooth.read();
    digitalWrite(13, LOW);
    if (command==107) { // If "k" is read
        md1.setM1Speed(0);
        md1.setM2Speed(0);
        digitalWrite(13,HIGH);
        while (command != 103) // Until "g" is read
        {command = (int)bluetooth.read();}
      }
  }
  nh.spinOnce(); // Check for updates with ROS
  delay(1);
}


/* 
 Arduino code to control the motors using input from ROS
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <SoftwareSerial.h>
#include <Servo.h> 
// Initialize the motors

boolean paused = false; // Whether or not the motors have received a paused command
boolean armDone = false;

ros::NodeHandle nh;
std_msgs::Float32 leftReturn_msg;
std_msgs::Float32 rightReturn_msg;
std_msgs::Bool paused_msg;
std_msgs::Bool grab_msg;
ros::Publisher pub_leftReturn("LeftReturn", &leftReturn_msg); // for nonmotor testing
ros::Publisher pub_rightReturn("RightReturn", &rightReturn_msg); // for nonmotor testing
ros::Publisher pub_paused("Paused", &paused_msg); // report a paused command
ros::Publisher pub_grab("GrabFinished", &grab_msg); // report a paused command

// Setup the motors
Servo rightVictor;
Servo leftVictor;
int rightVictorPin = 10;
int leftVictorPin = 11;

void leftMotor(const std_msgs::Float32& msg){
  // Set the left motor speed given by ROS in the topic "Motors"
  if (!paused) {
    leftVictor.write((int)((msg.data*80.0)+90));
    leftReturn_msg.data = msg.data;
    pub_leftReturn.publish( &leftReturn_msg);
  } 
  else {
    leftReturn_msg.data = 0;
    pub_leftReturn.publish( &leftReturn_msg);
  }
}

void rightMotor(const std_msgs::Float32& msg){
  // Set the right motor speed given by ROS in the topic "Motors"
  if (!paused) {
    rightVictor.write((int)(-1*(msg.data*80.0)+90));
    rightReturn_msg.data = msg.data;
    pub_rightReturn.publish( &rightReturn_msg);
  } 
  else {
    rightReturn_msg.data = 0;
    pub_rightReturn.publish( &rightReturn_msg);
  }
}

void grabPin(const std_msgs::Bool& msg){
   if(msg.data) {
      digitalWrite(5, HIGH);
   } else {
     digitalWrite(5, LOW);
   }
}

ros::Subscriber<std_msgs::Float32> subLeft("LeftMotors", &leftMotor);
ros::Subscriber<std_msgs::Float32> subRight("RightMotors", &rightMotor);
ros::Subscriber<std_msgs::Bool> subGrab("GrabObject", &grabPin);

void setup()
{ 
  pinMode(13, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(3, INPUT);
  digitalWrite(13, LOW); // LED is on unless we are moving
  digitalWrite(5, LOW);
  rightVictor.attach(rightVictorPin);
  leftVictor.attach(leftVictorPin);
  rightVictor.write(90);
  leftVictor.write(90);
  nh.initNode();
  nh.subscribe(subLeft); // Subscribe to "LeftMotors"
  nh.subscribe(subRight); // Subscribe to "RightMotors"
  nh.subscribe(subGrab);
  nh.advertise(pub_leftReturn); // publish left motor value for testing
  nh.advertise(pub_rightReturn); // publish right motor value for testing
  nh.advertise(pub_paused);
  nh.advertise(pub_grab);
  paused = false;
}

void loop()
{  

  
  if(digitalRead(3) && !armDone) {
    armDone = true; 
    grab_msg.data = 1;
    pub_grab.publish( &grab_msg);
  } else if(!digitalRead(3)){
    armDone = false;  
  }
  nh.spinOnce(); // Check for updates with ROS
  delay(0.1);
}



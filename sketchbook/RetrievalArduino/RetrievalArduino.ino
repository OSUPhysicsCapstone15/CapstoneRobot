/* This code is a skeleton to implement ROS control of the retrieval arm */
#define USE_USBCON
#include <ros.h>
#include <std_msgs/Bool.h>
#include<Servo.h>    //call servo library

//Standard PWM DC control
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;    //M1 Direction Control
int M2 = 7;    //M1 Direction Control

Servo myServo;      //declare servo motor
 
void back_off(char a,char b )          //Move linear actuator backwards
  {
    analogWrite (E1,a);      //Pulse width modulator Speed Control
    digitalWrite(M1,HIGH);   //Motor power on
  }  
void advance(char a,char b)          //Move linear actuator forward
  {
    analogWrite (E1,a);
    digitalWrite(M1,LOW);   //Motor power off
  }
  
// Set up ROS publishing
ros::NodeHandle nh;
std_msgs::Bool finished_msg;
ros::Publisher pub_finished("GrabFinished", &finished_msg); // Report back when retrieval is done

void grabObject(const std_msgs::Bool& msg) {
 if (msg.data == true) {
      myServo.attach(14);     //set myServo to pin 14
      advance(255,255);
      delay(9000);
      myServo.write(30); 
      delay(9000);
      delay(9000);
      delay(9000);
      myServo.write(160);
      delay(9000);
      back_off(255,255);
      //Do retrieval stuff
      finished_msg.data = true;
      pub_finished.publish( &finished_msg); // Report back that the retrieval is done
  }
  else {
    // Do nothing when not needed
    finished_msg.data = false;
    pub_finished.publish( &finished_msg); // Report back that the retrieval is done
  }
}

ros::Subscriber<std_msgs::Bool> subGrab("GrabObject", &grabObject);


void setup()
{
  nh.initNode();
  nh.subscribe(subGrab); // Subscribe to "GrabObject"
  pinMode(14, OUTPUT);    //set pin 14 as output

 /* The rest of the setup */ 
}

void loop()
{  
  nh.spinOnce(); // Check with ROS for updates
}


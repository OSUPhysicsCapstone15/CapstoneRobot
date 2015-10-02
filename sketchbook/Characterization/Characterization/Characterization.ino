/* This code is meant to be used to characterize the robot.
   The control functions are included from GoToPoint, so put
   commands into void loop(){...} to make it do stuff */

#include <PID.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include "DualVNH5019MotorShield.h"



DualVNH5019MotorShield md1;
ros::NodeHandle  nh;
std_msgs::Bool readyForData;
ros::Publisher beaconRequest("beacon_requestM",&readyForData);



//Initialize PID parameters:
// robot(countratio,wheel diameter,wheel-to-wheel distance,Kp,Ki,Kd,Kdiff,Kidiff)
PID robot(1200, 8, 10, .01, .01, .01, .01,.5);

// Function that uses beacon data to determine the direction the robot is facing
// Turns the robot and sends it forward 150 (cm?) at speed 15 
void beaconCallback( const std_msgs::Float64& angle)
{
  //readyForData.data=false;
  //beaconRequest.publish(&readyForData);
  //3 feet 
  double distance=150;
  // If the needed angle is more than 30 degrees off, turn and check again
  if (angle.data>=30 || angle.data<=-30)
  {
    //turnInPlace(0,45,150);
  }
  // If the angle is >0, turn right til straight
  else if(angle.data>0){
    //Turn to the angle and go
    turnInPlace(0,angle.data,250);
    robot.movexinches(distance, 15);
  }
  // If the angle is <0, turn left til straight
  else{
    turnInPlace(1,angle.data,250);
    robot.movexinches(distance, 15);
  }
  
  readyForData.data=true;
  beaconRequest.publish(&readyForData);
    
}

//Command 0=straight   data[1]=distance  data[2]=speed
//Command 1=turnRight  data[1]=angle     data[2]=speed
//Command 2=turnLeft   data[1]=angle     data[2]=speed
void commandCallback(const std_msgs::Float64MultiArray& command)
{
  if(command.data[0]==0)
  {
    robot.movexinches(command.data[1],command.data[2]);
  }else if(command.data[0]==1){
    turnRight(command.data[1],command.data[2]);  
  }else if(command.data[0]==2){
    turnLeft(command.data[1],command.data[2]);
  }
}


// Function that turns the robot without moving
// dir={left of right} 
// angle={desired turn angle}
// setspeed={speed at which the turn takes place}
void turnInPlace(int dir, double angle, double setspeed){
  int state = 0, laststate = 0, pulseratio = 1200;
  double wheeldiam = 8;

  double m1speed = setspeed, m2speed = setspeed;

  // Set wheels to turn in opposing directions at speed "setspeed"
  if(dir==1){
    m1speed *= -1;  // 1 means left
  }
  else {
    m2speed *= -1; // not 1 means right
  }

  // Converts given angle to encoder counts
  int countmax = pulseratio*angle/180*36/(wheeldiam);
  int counter = 0;
  
  // Sets the wheels to turn at equal but opposite speeds
  md1.setM1Speed(m1speed);
  md1.setM2Speed(m2speed);

  // this loop counts the shaft encoder pulses and stops when it thinks the turn is complete
  while(counter < countmax){
    if(analogRead(A0) > 500)
      {
        state = 1;
      }
    else
      {
        state = 0;
      }
    if (laststate != state)
      {
        counter++;
      }
    laststate = state;
  }  

  // Reset the wheel speeds to zero
  md1.setM1Speed(0);
  md1.setM2Speed(0);
}


// Function that makes the robot turn right by changing the left wheel's speed
void turnRight(double angle, double setspeed)
{
  int state = 0, laststate = 0, pulseratio = 600;
  double wheeldiam = 8;

  int countmax = pulseratio*angle/180*36*2/(3.1415927*wheeldiam);
  int counter = 0;

  // Set left wheels speed 
  md1.setM2Speed(setspeed);

  // This loop counts the shaft encoder pulses
  while(counter < countmax){
    if(analogRead(A1) > 500)
      {
        state = 1;
      }
    else
      {
        state = 0;
      }
    if (laststate != state)
      {
        counter++;
      }
      laststate = state;
  }  

  // Returns left wheel speed to zero
  md1.setM2Speed(0);
}


// Function that makes the robot turn left by changing the right wheel's speed
void turnLeft(double angle, double setspeed)
{
  int state = 0, laststate = 0, pulseratio = 600;
  double wheeldiam = 8;

  int countmax = pulseratio*angle/180*36*2/(3.1415927*wheeldiam);
//  Serial.print("\nTurn left countmax: \n");
//  Serial.print(countmax);
  int counter = 0;
  
  // Sets the right wheel speed
  md1.setM1Speed(setspeed);

  // This loop counts the shaft encoder pulses
  while(counter < countmax){
    if(analogRead(A0) > 500)
      {
        state = 1;
      }
     else
      {
        state = 0;
      }
    if (laststate != state)
      {
        counter++;
//        Serial.print("\nRight wheel: ");
//        Serial.print(counter);
      }
    laststate = state;
  }  
  
  // Return the right wheel speed to zero
  md1.setM1Speed(0);
}

ros::Subscriber<std_msgs::Float64> sub("beacon_data",&beaconCallback); 
ros::Subscriber<std_msgs::Float64MultiArray> comndSub("command",&commandCallback);



void setup()
{ 
//  nh.initNode();
//  nh.subscribe(sub);
//  nh.advertise(beaconRequest);
  md1.init();
  
}
int count=0;
//int counter=0;
//int state=0;
//int laststate=0;

void loop()
{ 
  // Just go with no encoder feedback
//  md1.setM1Speed(400);
//  md1.setM2Speed(400);
//  delay(10000);
//  md1.setM1Speed(0);
//  md1.setM2Speed(0);
//  delay(5000);
//  md1.setM1Speed(400);
//  md1.setM2Speed(-400);
//  delay(1000);
//  md1.setM1Speed(-400);
//  md1.setM2Speed(400);
//  delay(1000);
  // Move and turn with encoder feedback
// delay(1000);
// robot.movexinches(90,15);
 delay(1000);
 turnInPlace(0,360,250);
 delay(1000);
 turnInPlace(1,360,250);
  
}


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



//Initialize PID parameters
PID robot(600, 8, 10, .01, .01, .01, .01,.5);


void beaconCallback( const std_msgs::Float64& angle)
{
  //readyForData.data=false;
  //beaconRequest.publish(&readyForData);
  //3 feet 
  double distance=150;
  if (angle.data>=30 || angle.data<=-30)
  {
    //turnInPlace(0,45,150);
  }else if(angle.data>0){
    //Turn to the angle and go
    turnInPlace(0,angle.data,250);
    robot.movexinches(distance, 15);
  }else{
    turnInPlace(1,angle.data,250);
    robot.movexinches(distance, 15);
  }
  
  readyForData.data=true;
  beaconRequest.publish(&readyForData);
    
}

//Command 0=straight data[1]=distance data[2]=speed
//Command 1=turnRight data[1]=angle data[2]=speed
//Command 2=turnLeft data[1]=angle data[2]=speed
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

void turnInPlace(int dir, double angle, double setspeed){
  int state = 0, laststate = 0, pulseratio = 600;
double wheeldiam = 8;

double m1speed = setspeed, m2speed = setspeed;
if(dir==1){
m1speed *= -1;  // 1 means left
}
else {
m2speed *= -1; // not 1 means right
}

int countmax = pulseratio*angle/180*36/(wheeldiam);
int counter = 0;

md1.setM1Speed(m1speed);
md1.setM2Speed(m2speed);

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
}  // this loop counts the shaft encoder pulses

md1.setM1Speed(0);
md1.setM2Speed(0);
}

void turnRight(double angle, double setspeed)
{
int state = 0, laststate = 0, pulseratio = 600;
double wheeldiam = 8;

int countmax = pulseratio*angle/180*36*2/(3.1415927*wheeldiam);
int counter = 0;

md1.setM2Speed(setspeed);

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
}  // this loop counts the shaft encoder pulses

md1.setM2Speed(0);
}


void turnLeft(double angle, double setspeed)
{
int state = 0, laststate = 0, pulseratio = 600;
double wheeldiam = 8;

int countmax = pulseratio*angle/180*36*2/(3.1415927*wheeldiam);
int counter = 0;

md1.setM1Speed(setspeed);

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
}  // this loop counts the shaft encoder pulses

md1.setM1Speed(0);
}

ros::Subscriber<std_msgs::Float64> sub("beacon_data",&beaconCallback); 
ros::Subscriber<std_msgs::Float64MultiArray> comndSub("command",&commandCallback);



void setup()
{ 
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(beaconRequest);
  md1.init();
  
}
int count=0;
void loop()
{ 
 
  nh.spinOnce();
  delay(1);
}


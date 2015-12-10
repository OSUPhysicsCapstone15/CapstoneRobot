//master.cpp
//
//the master node will synchronize all the nodes and relay data back and forth
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <unistd.h>
#include <cmath>
using namespace ros;
using namespace std;

std_msgs::Float64 beacon;
std_msgs::Bool beacon_request;
bool ready;
///////////////////////////////////////////////////
////////////.. FUNCTION DECLARATIONS.. ///////////////
//////////////////////////////////////////////////////

void chatterbeacon_data(const std_msgs::Float64::ConstPtr& beacon_data);
void chatterbeacon_request(const std_msgs::Bool::ConstPtr& beacon_requestM);

//////////////////////////////////////////////////////
//////////////.. CLASS DECLARATIONS ..////////////////
//////////////////////////////////////////////////////

//////////////////////////////////////////////////////
////////////////////.. MAIN BODY ..///////////////////
//////////////////////////////////////////////////////
int main(int argc, char **argv)
  {
  beacon.data = 0;	
  beacon_request.data = 1;
  init(argc, argv, "master");
  NodeHandle n; 
  Publisher beacon_reqV = n.advertise<std_msgs::Bool>("beacon_request", 1);
  Subscriber beacon_data = n.subscribe("beacon_data", 1, chatterbeacon_data);
  Subscriber beacon_reqM=n.subscribe("beacon_requestM",1000,chatterbeacon_request);	
  Rate loop_rate(10);
  ready = true; 
  int sendInitial=1;
	
  while(ok())
    {
    /*if(sendInitial)
      {
       beacon_reqV.publish(beacon_request);
       sendInitial=0;
      }*/
    if(ready)
      {
      ROS_INFO("%d", ready);
      beacon_reqV.publish(beacon_request);
      //ready=0;
      }
    usleep(5000000);
    spinOnce();
    loop_rate.sleep();
    }

  return(0);
  }

//////////////////////////////////////////////////////
/////////////.. FUNCTION DEFINITIONS.. ///////////////
//////////////////////////////////////////////////////

//action for receiving beacon data from vision node
void chatterbeacon_data(const std_msgs::Float64::ConstPtr& beacon_data)
  {
  ROS_INFO("\nBeacon Data: [%f]\n", beacon_data->data);

  //stores beacon data
  beacon = *beacon_data;
  
  //changes request state to true;
  beacon_request.data = 1;
  }

//.....................................................

//action for receiving beacon request from arduino node
void chatterbeacon_request(const std_msgs::Bool::ConstPtr& motion_request)
 {
 ready=true;
 ROS_INFO("\nBeacon Data: [%d]\n", motion_request->data);	 
 }


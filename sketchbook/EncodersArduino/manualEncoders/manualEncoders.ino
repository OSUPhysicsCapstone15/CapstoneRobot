/* This code is meant to be used to characterize the robot.
   The control functions are included from GoToPoint, so put
   commands into void loop(){...} to make it do stuff */

#include <ros.h>
#include <std_msgs/Int32.h>
#include <SoftwareSerial.h> 

// Set up ROS publishing
ros::NodeHandle  nh;  
std_msgs::Int32 leftencoder_msg;
std_msgs::Int32 rightencoder_msg;
ros::Publisher pub_LeftEncoder("LeftEncoder", &leftencoder_msg);
ros::Publisher pub_RightEncoder("RightEncoder", &rightencoder_msg);

// Static constants
int freqDiv = 1;
const int bluetoothTx = 3;  // TX-O pin of bluetooth mate, Arduino D2
const int bluetoothRx = 5;  // RX-I pin of bluetooth mate, Arduino D3
boolean paused = false; // Whether or not the motors have received a paused command
SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

// Global Variables
long long countR, countL; // Encoder count
bool stateR, stateL, lastStateR, lastStateL, left, right;

void FreqDiv(const std_msgs::Int32& msg) {
  freqDiv = msg.data;
}


ros::Subscriber<std_msgs::Int32> fd("FreqDiv", &FreqDiv);

void setup()
{
  nh.initNode();  
  nh.subscribe(fd);
  nh.advertise(pub_LeftEncoder);
  nh.advertise(pub_RightEncoder);
  lastStateR = 0;
  lastStateL = 0;
  countR = 0;
  countL = 0;
  nh.spinOnce();
}

void loop () {
  if (analogRead(A2 ) > 500)
    {
      stateL = 1;
    }
    else
    {
      stateL = 0;
    }
    if ((lastStateL != stateL) && stateL)
    {
      countL++;
    }
    lastStateL = stateL;
    if (countL % 10 == 0 || !(freqDiv)) {
       leftencoder_msg.data = countL; 
       pub_LeftEncoder.publish( &leftencoder_msg);
       nh.spinOnce();
    }
    
    if (analogRead(A3 ) > 500)
    {
      stateR = 1;
    }
    else
    {
      stateR = 0;
    }
    if ((lastStateR != stateR) && stateR)
    {
      countR++;
    }
    lastStateR = stateR;
    if (countR % 10 == 0 || !(freqDiv)) {
       rightencoder_msg.data = countR; 
       pub_RightEncoder.publish( &rightencoder_msg);
       nh.spinOnce();
    }
}


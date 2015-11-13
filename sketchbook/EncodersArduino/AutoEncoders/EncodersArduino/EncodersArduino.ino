/*
 *  Arduino code to read the encoders and give that information to ROS
 */


#include <ros.h>
#include <Encoder.h>
#include <std_msgs/Int32.h>

// Set up ROS publishing
ros::NodeHandle  nh;
std_msgs::Int32 leftencoder_msg;
std_msgs::Int32 rightencoder_msg;
std_msgs::Int32 confirmheartbeat_msg;
ros::Publisher pub_LeftEncoder("LeftEncoder", &leftencoder_msg);
ros::Publisher pub_RightEncoder("RightEncoder", &rightencoder_msg);
ros::Publisher pub_confirmHeartbeat("Heartbeat", &confirmheartbeat_msg);

Encoder leftEnc(A0, A2); // Create left encoder object
Encoder rightEnc(A1, A3); // Create right encoder object

int freqDiv = 1;
int heartbeat = 0;

int newLeftPos = leftEnc.read();
int newRightPos = rightEnc.read();
int oldLeftPos = 0;
int oldRightPos = 0;

long countL = 0;
long countR = 0;

void FreqDiv(const std_msgs::Int32& msg) {
  freqDiv = msg.data;
}

void HeartbeatCheck(const std_msgs::Int32& msg) {
  heartbeat = msg.data;
}

ros::Subscriber<std_msgs::Int32> fd("FreqDiv", &FreqDiv);
ros::Subscriber<std_msgs::Int32> checkHeartbeat("Heartbeat", &HeartbeatCheck);

void setup()
{  
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(fd);
  nh.advertise(pub_LeftEncoder);
  nh.advertise(pub_RightEncoder);
  nh.spinOnce();
}



void loop()
{
  nh.spinOnce();
  if (heartbeat != 0)
  {
    confirmheartbeat_msg.data=heartbeat;
    pub_confirmHeartbeat.publish( &confirmheartbeat_msg);
    heartbeat=0;
  }
  newLeftPos = leftEnc.read();
  newRightPos = rightEnc.read();
  // If the encoder positions have changed, update their values with ROS
  if (newLeftPos != oldLeftPos) {
    oldLeftPos = newLeftPos;
    countL++;
  }
  if (countL % 10 == 0 || !(freqDiv)) {
    leftencoder_msg.data = countL;
    pub_LeftEncoder.publish( &leftencoder_msg);
    nh.spinOnce();
  }

  if (newRightPos != oldRightPos) {
    oldRightPos = newRightPos;
    countR++;
  }
  if (countR % 10 == 0 || !(freqDiv)) {
    rightencoder_msg.data = countR;
    pub_RightEncoder.publish( &rightencoder_msg);
    nh.spinOnce();
  }
}

#include <DualVNH5019MotorShield.h>
#include <PID.h>
#include <ros.h>
#include <std_msgs/Int32.h>


ros::NodeHandle  nh;
std_msgs::Int32 msg;
ros::Publisher chatter("chatter",&msg);
PID robot(1,2,3,4,5,6,7);
void beaconCallback( const std_msgs::Int32& state){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  int beaconStatus=state.data;//Message From Vision
  msg.data=beaconStatus;
  chatter.publish(&msg);
  //robot.movexinches(distance, 15);
  //delay(5000);
  //robot.movexinches(distance, -15);
  //delay(5000);  
}

ros::Subscriber<std_msgs::Int32> sub("beacon", &beaconCallback); 
void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);
}

void loop()
{  
  nh.spinOnce();
  delay(1);
}


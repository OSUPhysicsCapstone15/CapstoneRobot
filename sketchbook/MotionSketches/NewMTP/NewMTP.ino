/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;
std_msgs::String msg;
ros::Publisher chatter("chatter",&msg);

void m( const std_msgs::String& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  msg=toggle_msg;
  chatter.publish(&msg);
}

ros::Subscriber<std_msgs::String> sub("toggle_led", &m ); 
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


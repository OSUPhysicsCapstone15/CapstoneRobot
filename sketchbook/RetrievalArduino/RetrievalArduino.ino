/* This code is a skeleton to implement ROS control of the retrieval arm */

#include <ros.h>
#include <std_msgs/Bool.h>

// Set up ROS publishing
ros::NodeHandle nh;
std_msgs::Bool finished_msg;
ros::Publisher pub_finished("GrabFinished", &finished_msg); // Report back when retrieval is done

void grabObject(const std_msgs::Bool& msg) {
  if (msg.data == true) {
    /* Do retrieval stuff */
    finished_msg.data = true;
    pub_finished.publish( &finished_msg); // Report back that the retrieval is done
  }
  else {
    // Do nothing when not needed
    finished_msg.data = false;
  }
}

ros::Subscriber<std_msgs::Bool> subGrab("GrabObject", &grabObject);


void setup()
{
  nh.initNode();
  nh.subscribe(subGrab); // Subscribe to "GrabObject"
 
 /* The rest of the setup */ 
}

void loop()
{  
  nh.spinOnce(); // Check with ROS for updates
}


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
ros::Publisher beaconRequest("beacon_requestM", &readyForData);

// Static constants
static int pulseRatio = 1200; // Number of encoder counts per rotation
static double wheelDiameter = 7.75; // Diameter of the wheels in inches
static double wheelBase = 36; // Distance between wheels in inches

// Global Variables
long long countR, countL; // Encoder count
bool stateR, stateL, lastStateR, lastStateL, left, right;
double currentAngle, targetAngle; // The current angle of the robot, relative to start. Want to use gyro for this eventually
unsigned long lastTime; // Last recorded time in milliseconds

// Function prototypes
void turnInPlace(int dir, double angle, double setspeed);
void turnRightPivot(double setspeed);
void turnLeftPivot(double setspeed);
void stopMotors();
void zeroEncoders();
double encounterToDistance(int count);

void setup()
{
  //  nh.initNode();
  //  nh.subscribe(sub);
  //  nh.advertise(beaconRequest);
  md1.init();
  Serial.begin(9600);
  //Initialize PID parameters:
  PID robot(pulseRatio, wheelDiameter, wheelBase, .01, .01, .01, .01, .5); // Wheel diameter used to be set to 10, unsure of why
  zeroEncoders();
  currentAngle = 0;
  left = 0;
  right = 0;
  lastTime = millis();
  targetAngle = 45; // Dangerous to have this here, but used in testing
}

// This code will run continuously while the arduino is powered
void loop()
{
  /*if(millis() - lastTime > 10000){
    targetAngle = -targetAngle;
    zeroEncoders();
    lastTime = millis();
  }*/

  if(abs(targetAngle - currentAngle) > 5) { // With 5 degrees of precision
    if(targetAngle > currentAngle) {
      left = 0;
      right = 1;
    } else {
      left = 1;
      right = 0;
    }
  } else {
    left = 0;
    right = 0;
  }
  
  if (left) {
    turnLeftPivot(120);
  } else if (right) {
    turnRightPivot(120);
  } else {
    stopMotors();
  }
  
  // Update encoders
  stateL = (analogRead(A0) > 500); 
  if (lastStateL != stateL && stateL) { // On change in pulse, increment
    countL++;
  }
  lastStateL = stateL;
  
  stateR = (analogRead(A1) > 500); 
  if (lastStateR != stateR && stateL) { // On change in pulse, increment
    countR++;
  }
  lastStateR = stateR;
  currentAngle = 360.0 * (encoderToDistance(countL)-encoderToDistance(countR))/(3.14159*wheelDiameter);
}

void stopMotors() {
  md1.setM1Speed(0);
  md1.setM2Speed(0);
}

// Function that makes the robot turn right by changing the left wheel's speed
void turnRightPivot(double setspeed)
{
  // Set left wheels speed
  md1.setM1Speed(setspeed);
  md1.setM2Speed(0);
}

// Function that makes the robot turn right by changing the left wheel's speed
void turnLeftPivot(double setspeed)
{
  // Set left wheels speed
  md1.setM1Speed(0);
  md1.setM2Speed(setspeed);
}

void zeroEncoders() {
  countL = 0;
  countR = 0;
  lastStateR = 0;
  lastStateL = 0;
  stateR = 0;
  stateL = 0;
}

double encoderToDistance(int counts) {
  return (counts/pulseRatio*1.0)*3.14159*wheelDiameter;
}

//ros::Subscriber<std_msgs::Float64> sub("beacon_data", &beaconCallback);
//ros::Subscriber<std_msgs::Float64MultiArray> comndSub("command", &commandCallback);




// OLD, UNUSED CODE

/*
 
// Function that turns the robot without moving
// dir={0:left or 1:right}
// angle={desired turn angle}
// setspeed={speed at which the turn takes place}
void turnInPlace(int dir, double angle, double setspeed) {
  int state = 0, laststate = 0;

  double leftSpeed = setspeed, rightSpeed = setspeed;

  // Set wheels to turn in opposing directions at speed "setspeed"
  if (dir == 0) {
    leftSpeed *= -1;  // 1 means left
  }
  else {
    rightSpeed *= -1; // not 1 means right
  }

  // Converts given angle to encoder counts
  int countmax = (pulseRatio*wheelBase*(angle/360))/wheelDiameter; // Numerator does not have 2, since we are pivoting halfway through r 
  int counter = 0;

  // Sets the wheels to turn at equal but opposite speeds
  md1.setM1Speed(leftSpeed);
  md1.setM2Speed(rightSpeed);

  // this loop counts the shaft encoder pulses and stops when it thinks the turn is complete
  while (counter < countmax) {
    if (analogRead(A0) > 500) // if we read a pulse
    {
      state = 1;
    }
    else
    {
      state = 0;
    }
    if (laststate != state) // On down pulse, increment
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
  int state = 0, laststate = 0;

  int countmax = (2*pulseRatio*wheelBase*(angle/360)/wheelDiameter);
  int counter = 0;
  int i = 0;

  // Set left wheels speed
  md1.setM1Speed(setspeed);

  // This loop counts the shaft encoder pulses
  while (counter < countmax) {
    if (analogRead(A0) > 500)
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
    i++;
  }

  // Returns left wheel speed to zero
  md1.setM1Speed(0);
}


// Function that makes the robot turn left by changing the right wheel's speed
void turnLeft(double angle, double setspeed)
{
  int state = 0, laststate = 0;

  int countmax = (2*pulseRatio*wheelBase*(angle/360)/wheelDiameter);
  //  Serial.print("\nTurn left countmax: \n");
  //  Serial.print(countmax);
  int counter = 0;

  // Sets the right wheel speed
  md1.setM2Speed(setspeed);

  // This loop counts the shaft encoder pulses
  while (counter < countmax) {
    if (analogRead(A1) > 500)
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
  md1.setM2Speed(0);
}

// Function that uses beacon data to determine the direction the robot is facing
// Turns the robot and sends it forward 150 (cm?) at speed 15
void beaconCallback( const std_msgs::Float64& angle)
{
  //readyForData.data=false;
  //beaconRequest.publish(&readyForData);
  //3 feet
  double distance = 150;
  // If the needed angle is more than 30 degrees off, turn and check again
  if (angle.data >= 30 || angle.data <= -30)
  {
    //turnInPlace(0,45,150);
  }
  // If the angle is >0, turn right til straight
  else if (angle.data > 0) {
    //Turn to the angle and go
    turnInPlace(0, angle.data, 250);
    robot.movexinches(distance, 15);
  }
  // If the angle is <0, turn left til straight
  else {
    turnInPlace(1, angle.data, 250);
    robot.movexinches(distance, 15);
  }

  readyForData.data = true;
  beaconRequest.publish(&readyForData);

}


//Command 0=straight   data[1]=distance  data[2]=speed
//Command 1=turnRight  data[1]=angle     data[2]=speed
//Command 2=turnLeft   data[1]=angle     data[2]=speed
void commandCallback(const std_msgs::Float64MultiArray& command)
{
  if (command.data[0] == 0)
  {
    robot.movexinches(command.data[1], command.data[2]);
  } else if (command.data[0] == 1) {
    turnRight(command.data[1], command.data[2]);
  } else if (command.data[0] == 2) {
    turnLeft(command.data[1], command.data[2]);
  }
}


*/

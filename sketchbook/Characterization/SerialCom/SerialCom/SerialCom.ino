/*
  Example Bluetooth Serial Passthrough Sketch
 by: Jim Lindblom, Edited by Capstone OSU
 SparkFun Electronics
 date: February 26, 2013
 license: Public domain

 This example sketch converts an RN-42 bluetooth module to
 communicate at 9600 bps (from 115200), and passes any serial
 data between Serial Monitor and bluetooth module.
 */

#include <PID.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <SoftwareSerial.h> 
#include <Encoder.h>
#include "DualVNH5019MotorShield.h"


int bluetoothTx = 3;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 5;  // RX-I pin of bluetooth mate, Arduino D3

DualVNH5019MotorShield md1;
ros::NodeHandle  nh;
std_msgs::Bool readyForData;
ros::Publisher beaconRequest("beacon_requestM", &readyForData);



// Static constants
static int pulseRatio = 600; // Number of encoder counts per rotation (or 1200?)
static double wheelDiameter = 7.5; // 7.75 real wheel diameter
static double wheelBase = 38; // Distance between wheels in inches

PID robot(pulseRatio, wheelDiameter, wheelBase, .01, .01, .01, .01, .5); // Wheel diameter used to be set to 10, unsure of why

// Global Variables
long long countR, countL; // Encoder count
bool stateR, stateL, lastStateR, lastStateL, left, right;
double currentAngle, targetAngle, currentDistance, targetDistance; // The current angle of the robot, relative to start. Want to use gyro for this eventually
unsigned long lastTime; // Last recorded time in milliseconds

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);
void turnLeft(double angle, double setspeed);
void turnRight(double angle, double setspeed);

void setup()
{
  md1.init();
  Serial.begin(9600);  // Begin the serial monitor at 9600bps

  bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
//  bluetooth.print("$");  // Print three times individually
//  bluetooth.print("$");
//  bluetooth.print("$");  // Enter command mode
//  delay(100);  // Short delay, wait for the Mate to send back CMD
//  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
//  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
//  bluetooth.begin(9600);  // Start bluetooth serial at 9600
}

void loop()
{
  /*md1.setM1Speed(150);
  md1.setM2Speed(150);
  delay(1000);
  md1.setM1Speed(0);
  md1.setM2Speed(0);*/
  if(bluetooth.available())  // If the bluetooth sent any characters
  { 
    // Send any characters the bluetooth prints to the serial monitor
    int command=(int)bluetooth.read();
    if(command==103){ // If "g" is read
      Serial.println("Motors on.");
      goForward(27, 150);
    }
    else if (command==107) { // If "k" is read
      Serial.println("Motors off.");
      md1.setM1Speed(0);
      md1.setM2Speed(0);
    }
    else if (command==108) { // If "l" is read
      Serial.println("l");
      turnLeft(45, 170);
    }
    else if (command==114) { // If "r" is read
      Serial.println("r");
      turnRight(45, 170);
    }
    Serial.println((int)command);
  }
//  if(Serial.available())  // If stuff was typed in the serial monitor
//  {
//    // Send any characters the Serial monitor prints to the bluetooth
//    bluetooth.print((char)Serial.read());
//  }
  // and loop forever and ever!
}

// Function that makes the robot turn left by changing the right wheel's speed
void turnLeft(double angle, double setspeed)
{
  int state = 0, laststate = 0;

  int countmax = (2*pulseRatio*wheelBase*(angle/360)/wheelDiameter);
  Serial.print("\nTurn left countmax: \n");
  Serial.print(countmax);
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
              //Serial.print("\nRight wheel: ");
              //Serial.print(counter);
    }
    laststate = state;
  }
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
  md1.setM1Speed(0);

  // Returns left wheel speed to zero
  md1.setM1Speed(0);
}

void goForward(double inches, double setspeed) {
  int stateL = 0, laststateL = 0;
  int stateR = 0, laststateR = 0;
  double setLspeed = setspeed, setRspeed = setspeed;

  int countmax = inches/((3.14159)*wheelDiameter)*pulseRatio;
  int counterL = 0, counterR=0;
  int notdoneL = 1, notdoneR=1;
  

  // Set left wheels speed
  md1.setM1Speed(setspeed+15);//l
  md1.setM2Speed(setspeed);//r
  Serial.print("TotalForward: ");
  Serial.println(countmax);

  // This loop counts the shaft encoder pulses
  while (notdoneL || notdoneR) {
    if (analogRead(A0) > 500)
    {
      stateL = 1;
    }
    else
    {
      stateL = 0;
    }
    if (laststateL != stateL && stateL)
    {
      counterL++;
    }
    laststateL = stateL;

    if (analogRead(A1) > 500)
    {
      stateR = 1;
    }
    else
    {
      stateR = 0;
    }
    if (laststateR != stateR && stateR)
    {
      counterR++;
    }
    laststateR = stateR;

    /*if(abs(counterL-counterR) > 50){
      if(counterR>counterL && counterL<countmax){
        md1.setM1Speed(setLspeed+20);
      } else if (counterR<countmax) {
        md1.setM2Speed(setRspeed+20);
      }
    } else {
      md1.setM1Speed(setLspeed);
      md1.setM2Speed(setRspeed);
    }*/
    
    if(counterR >= countmax){
      setRspeed = 0;
      md1.setM2Speed(setRspeed);
      notdoneR = 0;
    }
    if(counterL >= countmax){
      setLspeed = 0;
      md1.setM1Speed(setLspeed);
      notdoneL = 0;
    }
  }
  Serial.println(counterL);
  Serial.println(counterR);
  md1.setM1Speed(0);

  // Returns left wheel speed to zero
  md1.setM1Speed(0);

}


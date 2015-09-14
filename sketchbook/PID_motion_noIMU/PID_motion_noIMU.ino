#include <PID.h>
#include <DualVNH5019MotorShield.h>



PID robot(600, 8, 10, .01, .01, .01, .01, .1); 
DualVNH5019MotorShield md1;

void turnleft(double angle, double setspeed);
void turnright(double angle, double setspeed);
void turninplace(int dir, double angle, double setspeed);
void setup() {
  // initialize serial communication at 9600 bits per second:
    Serial.begin(9600);
    md1.init();
    }
    
// the loop routine runs over and over again forever:
void loop() {
 turninplace(1, 90, 150);
 turnright(30, 150);
 turnleft(30, 150);
 robot.movexinches(20, 30); // inches, in/s
 delay(5000);

}

void turnleft(double angle, double setspeed)
{
int state = 0, laststate = 0, pulseratio = 600;
double wheeldiam = 8;

int countmax = pulseratio*angle/180*36*2/(wheeldiam);
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

void turnright(double angle, double setspeed)
{
int state = 0, laststate = 0, pulseratio = 600;
double wheeldiam = 8;

int countmax = pulseratio*angle/180*36*2/(wheeldiam);
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

void turninplace(int dir, double angle, double setspeed){
  int state = 0, laststate = 0, pulseratio = 600;
double wheeldiam = 8;

double m1speed = setspeed, m2speed = setspeed;
if(dir = 1){
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

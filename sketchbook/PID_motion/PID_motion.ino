#include <PID.h>
#include "DualVNH5019MotorShield.h"

PID robot(600, 8, 10, .01, .01, .01, .01, .1); 
// counts ratio, wheel diam, wheel2wheel, kp, ki, kd, kdiff, kidiff

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}
// the loop routine runs over and over again forever:
void loop() {
 
 robot.movexinches(20000, 30); // inches, in/s
 delay(5000);
 robot.movexinches(20000, -30);
 delay(5000); 
}

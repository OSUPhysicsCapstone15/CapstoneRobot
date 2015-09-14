#include <PID.h>
#include "DualVNH5019MotorShield.h"

PID robot(600, 5, 10, .01, .01, .01, .01);

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}
// the loop routine runs over and over again forever:
void loop() {
 
 robot.movexinches(1000, 15);
 delay(5000);
 robot.movexinches(1000, -15);
 delay(5000); 
}

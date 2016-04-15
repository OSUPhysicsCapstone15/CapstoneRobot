#include "DualVNH5019MotorShield.h"
DualVNH5019MotorShield md1;

void setup() {
  md1.init();
}

void loop() {
  md1.setM1Speed(400);
  md1.setM2Speed(0);
  delay(40000);
  md1.setM1Speed(0);
  md1.setM2Speed(0);
  delay(10000);
}

// PID.h library for PID with shaft encoders

#ifndef PID_h
#define PID_h

#include "Arduino.h"

class PID
{
  public:
   PID(double countsratio, double WheelDiam, double wheel2wheel, double kp, double ki, double kd, double kdiff, double kidiff);
   void movexinches(double stopdistance, double SetSpeed);

  private:
   double _countsratio;
   double _WheelDiam;
   double _wheel2wheel;
   double _kp, _ki, _kd, _kdiff, _kidiff;
   double _state1, _state2, _laststate1, _laststate2, _freq1, _freq2;
   double _counter1, _counter2;
   void CalcFreq();
   
};

#endif
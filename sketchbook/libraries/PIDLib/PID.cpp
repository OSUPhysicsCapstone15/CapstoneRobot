#include "Arduino.h"
#include "PID.h"
#include "DualVNH5019MotorShield.h"

PID::PID(double countsratio, double WheelDiam, double wheel2wheel, double kp, double ki, double kd, double kdiff, double kidiff)
{
  Serial.begin(9600);
  _countsratio = countsratio;
  _WheelDiam = WheelDiam;
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _kdiff = kdiff;
  _kidiff = kidiff;
}

void PID::movexinches(double stopdistance, double SetSpeed){
  
  DualVNH5019MotorShield md;
  md.init();
  
  double SetFreq = SetSpeed * _countsratio / (3.1415927 * _WheelDiam), Nturns = stopdistance / (3.1415927 * _WheelDiam);
  double errSum1 = 0, errSum2 = 0, lastErr1 = 0, lastErr2 = 0, dErr1 = 0, dErr2 = 0;
  long Ncounts = Nturns * _countsratio;
  int iteration = 0;
  unsigned int lasttime = millis(), time;

  double diff_err_sum = 0;
  
  Serial.print("setFreq = ");
  Serial.println(SetFreq);
  Serial.println("freq_1  freq_2   motorV_1  motoeV_2");
  
  _counter1 = 0;
  _counter2 = 0;

  double motorspeedinit = 150;


  md.setM1Speed(motorspeedinit); // start motors at 100 mV
  md.setM2Speed(motorspeedinit);

  while(_counter1 < Ncounts)
  {
  CalcFreq();
  time = millis();
  double timeChange = (double)(time - lasttime) / 1000; // time change since last iteration
  double error1 = SetFreq - _freq1; // calculate error terms
  double error2 = SetFreq - _freq2;
  errSum1 += (error1 * timeChange);
  errSum2 += (error2 * timeChange); // add to integral terms

  diff_err_sum += (_freq1 - _freq2) * timeChange;

  if(iteration != 0)
  {
    dErr1 = (error1 - lastErr1) / timeChange; // find derivative terms
    dErr2 = (error2 - lastErr2) / timeChange;
  }
  
  double diff = _freq1 - _freq2; // find difference in wheel frequencies
   
  double motorspeed1 = _kp * error1 + _ki * errSum1 + _kd * dErr1 - _kdiff*diff - diff_err_sum * _kidiff;
  double motorspeed2 = _kp * error2 + _ki * errSum2 + _kd * dErr2; // calculate new motor speed
  
  if(abs(error1 > error2))
  {
    double motorspeed1 = _kp * error1 + _ki * errSum1 + _kd * dErr1;
    double motorspeed2 = _kp * error2 + _ki * errSum2 + _kd * dErr2 + _kdiff*diff + diff_err_sum * _kidiff; // calculate new motor speed
  }
 
  if(abs(motorspeed2 - motorspeed1) > 150) // to prevent crazy oscillations
  {
    motorspeed2 = motorspeedinit;
    motorspeed1 = motorspeedinit;
  }
  
  if(_counter1 < Ncounts)
  {
  md.setM1Speed(motorspeedinit + motorspeed1); // set motor speeds
  md.setM2Speed(motorspeed2 + motorspeedinit);
  }
  
  Serial.print(_freq1);
  Serial.print("   ");
  Serial.print(_freq2);
  Serial.print("   ");
  Serial.print(motorspeed1);
  Serial.print("   ");
  Serial.println(motorspeed2);
  
  lastErr1 = error1;
  lastErr2 = error2;
  lasttime = time;
  
  if(analogRead(A0) > 500) // see if wheels have turned more
    {
      _state1 = 1;
    }
  else
    {
      _state1 = 0;
    }
  if (_laststate1 != _state1)
    {
      _counter1++;
    }
      if(analogRead(A1) > 500)
    {
      _state2 = 1;
    }
  else
    {
      _state2 = 0;
    }
  if (_laststate2 != _state2)
    {
      _counter2++;
    }
    _laststate1 = _state1;
    _laststate2 = _state2;
    iteration++;

  }
  md.setM1Speed(0); // stop motors
  md.setM2Speed(0);
  delay(2000);
}

void PID::CalcFreq() //Calculate pulse frequency
{
  long subcounter1 = 0, subcounter2 = 0;
  unsigned long starttime = millis();
  unsigned long interval = 250;
  
  while(millis() - starttime < interval)
  {
    if(analogRead(A0) > 500)
    {
      _state1 = 1;
    }
    else
    {
      _state1 = 0;
    }
  if (_laststate1 != _state1)
    {
      subcounter1++;
    }
    _laststate1 = _state1;
    if(analogRead(A1) > 500)
    {
      _state2 = 1;
    }
    else
    {
      _state2 = 0;
    }
  if (_laststate2 != _state2)
    {
      subcounter2++;
    }
    _laststate2 = _state2;
  }  
  
  unsigned long stoptime = millis();
  _counter1 += subcounter1;
  _counter2 += subcounter2;
  _freq2 = subcounter2 * 1000.0 / (stoptime - starttime);
  _freq1 = subcounter1 * 1000.0 / (stoptime - starttime);
}
  

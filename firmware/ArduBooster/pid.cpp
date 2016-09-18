#include <Arduino.h>

#include "pid.h"

PID::PID(double p, double i, double d)
{
  mKp = p;
  mKi = i;
  mKd = d;
  
  mISum = 0;
  
  mFirstRun = true;
}

double PID::process(int input_val, int setpoint_val)
{
  if(mFirstRun) {
    mFirstRun = false;
    mLastTime = millis();
    mLastInput = input_val;
  }
  
  float delta_time = (millis() - mLastTime) / 1000.0;
  
  // "input to target" error
  int error = setpoint_val - input_val;
  
  // P
  double p = error * mKp;
  
  // I
  mISum += error * mKi * delta_time;
  
  // D
  double d = (mLastInput - input_val) * mKd / delta_time;
  
  mLastTime = millis();
  mLastInput = input_val;
  
  // PID
  return p + mISum + d;
}

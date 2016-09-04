#include <Arduino.h>

#include "pid.h"

PID::PID(double p, double i, double d)
{
  // default min/max output (to works with PWM)
  mMinOut = 0;
  mMaxOut = 255;
  
  mSampleTime = 50; // ms
  
  mLastTime = millis() - mSampleTime;
}

double PID::process(int input_val, int setpoint_val)
{
  
}

bool PID::setLimits(int min_out, int max_out)
{
  if(min_out >= max_out) 
    return false;
  mMinOut = min_out;
  mMaxOut = max_out;
  return true;
}

void PID::setTime(int delta_time)
{
  
}

#ifndef PID_h
#define PID_h

class PID
{
  public:
  
    PID(double p, double i, double d);
    double process(int input_val, int setpoint_val);
    
  private:
  
    double mKp, mKi, mKd;
    double mISum;
    
    unsigned long mLastTime;

    int mLastInput;    
    
    bool mFirstRun;
    
};
#endif

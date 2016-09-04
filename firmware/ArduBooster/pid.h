#ifndef PID_h
#define PID_h

class PID
{
  public:
  
    PID(double p, double i, double d);
    
    double process(int input_val, int setpoint_val);
    bool setLimits(int min_out, int max_out);
    void setTime(int delta_time);
    
  private:
  
    double mP, mI, mD;
    double mISum;
    
    unsigned long mLastTime;

    int mLastInput;    
    int mSampleTime;
    int mMinOut, mMaxOut;
    
};
#endif

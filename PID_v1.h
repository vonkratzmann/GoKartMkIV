#ifndef PID_v1_h
#define PID_v1_h

class PID
{
  public:
    PID(double*, double*, double*, double, double, double); //constructor.  links the PID to the Input, Output, Setpoint.  Initial tuning parameters are also set here.
    bool Compute();                         //performs the PID calculation. Calculation frequency can be set using SetSampleTime
    void SetOutputLimits(double, double);   //clamps the output to a specific range. 0-255 by default, bu
    void SetSampleTime(int);                //sets the frequency, in Milliseconds, with which the PID calculation is performed.  default is 100
   void SetTunings(double, double, double); //set tuning parameters

  private:
    double kp;                              //(P)roportional Tuning Parameter
    double ki;                              //(I)ntegral Tuning Parameter
    double kd;                              //(D)erivative Tuning Parameter

    double *myInput;                        //Pointers to the Input, Output, and Setpoint variables
    double *myOutput;                       //This creates a hard link between the variables and the
    double *mySetpoint;                     //PID, freeing the user from having to constantly tell us
    double errSum;
    double lastErr;
    double error;
    double output;

    unsigned long lastTime;
    unsigned long SampleTime;
    double outMin, outMax;
};
#endif


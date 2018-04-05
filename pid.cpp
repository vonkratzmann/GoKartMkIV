/**********************************************************************************************
   Modified version of 
   Arduino PID Library - Version 1.2.1
   by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com

   This Library is licensed under the MIT License

   Reason for the modifcations were to:
   - originally allow to enter some prints to the Serial monitor to debug the program and tune the PID
   - simplified as did not need fuctions such as multiple constructors, SetMode(), SetControllerDirection(), display tuning parameters
    - in compute() simplified eg removed code for Add Proportional on Measurement and others
    - added code to clear error variables when setpoint was zero and input was not zero, as would ocassionally get an output power pulse
 **********************************************************************************************/
#include "GoKartMkIV.h"
#include "pid.h"

/*Constructor (...)*********************************************************
      The parameters specified here are those for for which we can't set up
      reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
         double Kp, double Ki, double Kd)
{
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;

  PID::SetOutputLimits(0, 255);				              //default output limit corresponds to the arduino pwm limits
  SampleTime = 100;							                    //default Controller Sample Time is 0.1 seconds
  PID::SetTunings(Kp, Ki, Kd);                      //set tuning parameters

  lastTime = millis() - SampleTime;   //set up ready for next compute()
}


/* Compute() **********************************************************************
       This, as they say, is where the magic happens.  this function should be called
     every time "void loop()" executes.  the function will decide for itself whether a new
     pid Output needs to be computed.  returns true when the output is computed,
     false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime);
  if (timeChange >= SampleTime)
  {
    /*Compute all the working error variables*/
    double error = *mySetpoint - *myInput;
    
    errSum += error;
    double dErr = (error - lastErr);
    
/* ignore case of setpoint == 0, but moving ie input non zero, as can get a short power pulse sent to the motor, 
 * because sometimes "error-lasterror" can be postive */
    if (!*mySetpoint && *myInput) {           
      error = 0;
      errSum = 0;
      dErr = 0;
    }
    /*Compute PID Output*/
    output = kp * error + ki * errSum + kd * dErr;
    
    PID_DEBUG_PRINT1("error:", error," errSum:", errSum, " dErr:", dErr," output", output);        //if  PID_DEBUG_PRINT1 defined print to serial monitor
   
    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;
    *myOutput = output;

    /*Remember some variables for next time*/
    lastErr = error;
    lastTime = now;
    return true;
  }
  else return false;
}


/* SetSampleTime(...) *********************************************************
   sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime)
{
  if (NewSampleTime > 0)
  {
    double ratio  = (double)NewSampleTime
                    / (double)SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long)NewSampleTime;
  }
}
void PID::SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime) / 1000;
  kp = Kp;
  ki = Ki * SampleTimeInSec;
  kd = Kd / SampleTimeInSec;
}


/* SetOutputLimits(...)****************************************************

 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max)
{
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;
}





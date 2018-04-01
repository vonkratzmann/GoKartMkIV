/**********************************************************************************************
   Arduino PID Library - Version 1.2.1
   by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com

   This Library is licensed under the MIT License
 **********************************************************************************************/

#include "Arduino.h"
#include "PID_v1.h"

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
    //Serial.print("error:"); Serial.println(error);

    errSum += error;
    //Serial.print("errSum:"); Serial.println(errSum);
    double dErr = (error - lastErr);
    //Serial.print("dErr:"); Serial.println(dErr);

    if (!*mySetpoint && *myInput) {           //no setpoint but moving
      error = 0;
      errSum = 0;
      dErr = 0;
    }

    /*Compute PID Output*/
    output = kp * error + ki * errSum + kd * dErr;

    //Serial.print("PIDOutput:"); Serial.println(output); Serial.println();

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





#include "GoKartMkIV.h"
#include "joystick.h"

/* JoyStick constructor
     initialses current x and y positions to the centre of the joystick
*/
JoyStick::JoyStick ()
{
  x_Cur = y_Cur = 511;                       //10 bit ADC, forwards 0 to 511,backwards 512 to 1023
  x_New = y_New = 511;                       //set joy stick to center position ie stopped
}


/* JoyStick get x axis or position
    reads joystick, returns raw value unprocessed
*/
unsigned int  JoyStick::get_X_Axis(void) {
  return  (unsigned int) analogRead(Xaxis_JoystickAnalogPin); //read joystick x position
}                                                             //end of get_X_Axis()


/* JoyStick get y axis or position
  reads joystick,  returns raw value unprocessed
*/
unsigned int  JoyStick::get_Y_Axis(void) {
  return  (unsigned int) analogRead(Yaxis_JoystickAnalogPin); //read joystick y position
}                                                             //end of get_Y_Axis()


/* JoyStick check x position
    reads joystick, compares current and new
    masks bottom bits to prevent unwanted noise and doing unnecessary processing
    limits rate of change
    returns true if position has changed
*/
bool JoyStick::check_X_Axis (void)               //check joystick for any changes
{
  bool x_Chnged = false;                        //flag joy stick position has changed
  x_New = analogRead(Xaxis_JoystickAnalogPin); //read joystick x position and put into x_New

  JOYSTICK_DEBUG_X_EQUALS_256                   //if debug is defined, sets a fixed value for x for debugging

  x_New &= Noise_Mask;                          //zero bottom bits to prevent unnecessary calls in case of noise on ADC input
  if ( x_Cur != x_New)                          //Check if changed from last read
  {
    x_Chnged = true;                            //yes, set flag to say it has changed
    diff = x_New - x_Cur;
    if (abs(diff) > JoyStick_Max_ROC)           //check if difference greater then max rate of change (ROC)
    {
      if (diff > 0)                             //yes, limit max acceleration to max rate of change, check if reading is increasing or decreasing
        x_Cur += JoyStick_Max_ROC;              //reading has gone up,so limit acceleration by adding max rate of change
      else
        x_Cur -= JoyStick_Max_ROC;              //reading has gone down,so limit acceleration by subtracting max rate of change
    }
    else
    {
      x_Cur = x_New;                          // change less than max rate of change, so accept new value
    }
    JOYSTICK_DEBUG_PRINT(__FUNCTION__, " x_New:", x_New, " x_Cur:", x_Cur);  // if joystick debug defined prints out position of joystick on serial monitor
  }
  return x_Chnged;
}


/* JoyStick check y position
   reads joystick, compares current and new
   masks bottom bits to prevent unwanted noise and doing unnecessary processing
   limits rate of change
   returns true if position has changed
*/
bool JoyStick::check_Y_Axis (void)               //check joystick for any changes
{
  bool y_Chnged = false;                        //flag joy stick position has changed
  y_New = analogRead(Yaxis_JoystickAnalogPin);   //read joystick y position and put into y_new

  JOYSTICK_DEBUG_Y_EQUALS_256                  //if debug is defined, sets a fixed value for x for debugging

  y_New &= Noise_Mask;                          //zero bottom bits to prevent unnecessary calls in case of noise on ADC input
  if ( y_Cur != y_New)                          //Check if changed from last read
  {
    y_Chnged = true;                             //yes, set flag to say it has changed
    diff = y_New - y_Cur;
    if (abs(diff) > JoyStick_Max_ROC)       //check if difference greater then max rate of change (ROC)
    {
      if (diff > 0)                         //yes, limit max acceleration to max rate of change, check if reading is increasing or decreasing
        y_Cur += JoyStick_Max_ROC;          //reading has gone up,so limit acceleration by adding max rate of change
      else
        y_Cur -= JoyStick_Max_ROC;          //reading has gone down,so limit acceleration by subtracting max rate of change
    }
    else
    {
      y_Cur = y_New;                        // change less than max rate of change, so accept new value
    }
    JOYSTICK_DEBUG_PRINT(__FUNCTION__, " y_New:", y_New, " y_Cur:", y_Cur);  //if joystick debug defined prints out position of joystick on serial monitor
  }
  return y_Chnged;
}


/* JoyStick::process_X
   accepts two pointer arguments to allow update of amount of turn requested and direction
   checks if its in stopped range, if yes sets turning percent to zero
   else checks requested direction and updates direction
*/
void JoyStick::process_X(unsigned int *turningDegrees, bool *new_Dir)            //process change for x axis of joystick
{
  if (x_Cur <= Stopped_High && x_Cur >= Stopped_Low)                   //check if in the stopped range
  {
    *turningDegrees = 0;                                                //yes, so update turning percent to 0 precent, no need to set direction
    JOYSTICK_PROCX_DEBUG_PRINT(__FUNCTION__, " (stopped)", " turningDegrees:", *turningDegrees, " ", " ");  //if joystick_procx_debug defined prints out results on serial monitor
  }
  else                                                          //no, joystick requesting movement
  {
    if (x_Cur > Stopped_High)                                    //is joystick asking to move to left
    {
      *new_Dir = LEFT;                                          //yes, moving to left
      *turningDegrees = map(x_Cur, Stopped_Low - 1, 0, 0, 90);  //Scale joystick position to turning range of 0 to 90 degrees
      JOYSTICK_PROCX_DEBUG_PRINT(__FUNCTION__, " (low)", " turningDegrees:", *turningDegrees, " new_Dir:", *new_Dir);  //if joystick_procx_debug defined prints out results on serial monitor
    }
    else                                                            //no, request to move to right
    {
      *new_Dir = RIGHT;
      *turningDegrees = map(x_Cur, Stopped_High + 1, 1023, 0, 90); //Scale joystick position to turning range of 0 to 90 degrees
      JOYSTICK_PROCX_DEBUG_PRINT(__FUNCTION__, " (high)", " turningDegrees:", *turningDegrees, " new_Dir:", *new_Dir);
    }
  }
}


/* JoyStick::process_Y
   accepts two pointer arguments to allow update of new speed and direction
   checks if its in stopped range, if yes sets speed to zero
   else checks requested direction and updates direction
   then scales the new speed bewteen the min and max speeds based on joystick position
*/
void JoyStick::process_Y(unsigned int *new_Spd, bool *new_Dir)            //process change for Y axis of joystick
{
  if (y_Cur <= Stopped_High && y_Cur >= Stopped_Low)            //check if in the stopped range
  {
    *new_Spd = 0;                                               //yes, stopped so update speed to say stopped, no need to set direction
    JOYSTICK_PROCY_DEBUG_PRINT(__FUNCTION__, " (stopped)", " new_Spd:", *new_Spd, " new_Dir:", *new_Dir); //if joystick_procy_debug defined prints out results on serial monitor
  }
  else                                                          //no, joystick requesting movement
  {
    if (y_Cur < Stopped_Low)                                    //is joystick asking to skow down
    {
      *new_Dir = FORWARD;                                       //yes, moving forward
      /* Scale joystick position to speed in millimeters per second
          so that forward and reverse are scaled to the same range
          overcomes issue where "0 to Stopped_Low" is different to "Stopped_High to 1023"
      */
      *new_Spd = map(y_Cur, Stopped_Low - 1, 0, 0, MaxSpeedmmPerSec);
      JOYSTICK_PROCY_DEBUG_PRINT(__FUNCTION__, " (low)", " new_Spd:", *new_Spd, " new_Dir:", *new_Dir); //if joystick_procy_debug defined prints out results on serial monitor
    }
    else                                                        //no, reversing
    {
      *new_Dir = BACK;
      /* Scale joystick position to speed in millimeters per second
          so that forward and BACK are scaled to the same range
          overcomes issue where "0 to Stopped_Low" is different to "Stopped_High to 1023"
      */
      *new_Spd = map(y_Cur, Stopped_High + 1, 1023, 0, MaxSpeedmmPerSec);
      JOYSTICK_PROCY_DEBUG_PRINT(__FUNCTION__, " (high)", " new_Spd:", *new_Spd, " new_Dir:", *new_Dir); //if joystick_procy_debug defined prints out results on serial monitor
    }
  }
}

#include "GoKartMkIII.h"
#include "Motor.h"

/* Diagnostics for motor
   If set to 1 and switch state changes prints out the motor speed
   Normally set to zero
*/

/* Motor constructor
   constructor, set speed pin and direction pin
   set initial speed and direction
*/

Motor::Motor(uint8_t* par_pwm_reg, int par_direction_pin, uint8_t par_motor_maxspeed)
{
  currentSpeed   = 0;                   //set initial speed
  requestedSpeed = 0;                   //set initial speed
  motor_maxspeed = par_motor_maxspeed;  //set upper phsical speed for the motor
  requestedDir   = FORWARD;             //set initial direction
  currentDir     = FORWARD;             //set initial direction

  dir_Pin = par_direction_pin;            //store pin to be used to set the direction
  pwm_Reg = par_pwm_reg;                  //store reg to be used to control speed via PWM signal
  pinMode(dir_Pin, OUTPUT);               //sets the digital direction pin as output
  digitalWrite(dir_Pin, currentDir);      //sets the initial direction
}

/* set_Requested_Speed
   stores the new requested speed
   it is up to other logical routines to change the actual speed
*/
void  Motor::set_Requested_Speed(int par_new_speed)
{
  requestedSpeed = par_new_speed;

  MOTOR_DEBUG_FILE("Function: ");
  MOTOR_DEBUG_FILE(__FILE__);
  MOTOR_DEBUG_FILE(",");
  MOTOR_DEBUG_PRINT(__FUNCTION__);;
  MOTOR_DEBUG_PRINT(" requestedSpeed: ");
  MOTOR_DEBUG_PRINT(requestedSpeed);
}

/* get_Requested_Speed
   retrurns the requested speed, not speed the motor is actually doing
*/
int Motor::get_Requested_Speed(void)
{
  return requestedSpeed;
}

/* set_Requested_Dir
   stores the requested direction
   it is up to other logical routines to change the actual speed and direction
*/
void  Motor::set_Requested_Dir(int par_new_direction)
{
  requestedDir = par_new_direction;

  MOTOR_DEBUG_PRINT(" requestedDir: ");
  MOTOR_DEBUG_PRINTLN(requestedDir);
}

/* get_Requested_Dir
   returns the the requested direction of the motor
*/
int Motor::get_Requested_Dir(void)
{
  return requestedDir;
}

/* get_Requested_Speed
  returns the current speed, the speed the motor is actually doing
*/
int Motor::get_Current_Speed(void)
{
  return currentSpeed;
}

/* get_Requested_Dir
   returns the the current direction of the motor
*/
int Motor::get_Current_Dir(void)
{
  return currentDir;
}

/* update_Speed
   updates the current speed from the requested speed
   updates are limited to the maximum rate of change to prevent abrupt stops and starts
   for when the chain hits the reed limit switches
   scales the speed to the range for the PWM pulse
*/
void Motor::update_Speed()
{
  if (requestedSpeed != currentSpeed)    //Check if need to update speed
  {
    MOTOR_DEBUG_FILE("Function: ");
    MOTOR_DEBUG_FILE(__FILE__);
    MOTOR_DEBUG_FILE(",");
    MOTOR_DEBUG_PRINT(__FUNCTION__);
    MOTOR_DEBUG_PRINT(" currentSpeed: ");
    MOTOR_DEBUG_PRINT(currentSpeed);
    MOTOR_DEBUG_PRINT(" requestedSpeed: ");
    MOTOR_DEBUG_PRINT(requestedSpeed);

    int diff;                               //yes, calculate the difference
    diff = requestedSpeed - currentSpeed;
    if (abs(diff) > Motor_Max_ROC)          //check if difference greater then max rate of change (ROC)
    { //limit max acceleration to max rate of change
      if (diff > 0)                         //check if reading is increasing or decreasing
        currentSpeed += Motor_Max_ROC;     //reading has gone up,so limit acceleration by adding max rate of change
      else
        currentSpeed -= Motor_Max_ROC;     //reading has gone down,so limit acceleration by subtracting max rate of change
    }
    else
    {
      currentSpeed = requestedSpeed;      //change less than max rate of change, so accept new value
    }
    MOTOR_DEBUG_PRINT(" updated currentSpeed: ");
    MOTOR_DEBUG_PRINT(currentSpeed);
    /* scale speed to range of PWM. PWM range is 0 t0 255, which is stopped to full speed for the motor. If the upper motor speed is to be restricted,
      then MOTOR_MAXSPEED is set to something below 255 */
    uint8_t tmpSpeed = map(currentSpeed, MINSPEED, MAXSPEED, MOTOR_MINSPEED, motor_maxspeed);
    * pwm_Reg = tmpSpeed;                 //output duty cycle

    MOTOR_DEBUG_PRINT(" new scaled Speed: ");
    MOTOR_DEBUG_PRINTLN(tmpSpeed);
  }
}

/* update_Dir
   updates the current direction from the requested direction
   does not check if speed is appropriate to change direction
*/
void    Motor::update_Dir(void)
{
  currentDir = requestedDir;
  digitalWrite(dir_Pin, currentDir);
}

/* clearFwdTimer
  clear timer tracking how long motor has been moving forwards
*/
void Motor::clearFwdTimer(void)
{
  timeMovingForwards = 0;
}

/* clearRevTimer
  clear timer tracking how long motor has been moving backwards
*/
void Motor::clearRevTimer(void)
{
  timeMovingBackwards = 0;
}

/* getFwdTimer
  get time of how long motor has been moving forwards
*/
long  Motor::getFwdTimer(void)
{
  return  timeMovingForwards;
}

/* getRevTimer
  get time of how long motor has been moving backwards
*/
long  Motor::getRevTimer(void)
{
  return timeMovingBackwards;
}


/* setFwdTimer
  set timer to current time, so later you then can compare to see how long the motor has been moving
*/
void  Motor::setFwdTimer(void)
{
  timeMovingForwards = millis();
}

/* setRevTimer
  set timer to current time, so later you then can compare to see how long the motor has been moving
*/
void  Motor::setRevTimer(void)
{
  timeMovingBackwards = millis();
}

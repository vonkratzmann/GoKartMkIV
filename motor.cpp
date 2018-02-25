#include "GoKartMkIV.h"
#include "motor.h"

/* Diagnostics for motor
   If set to 1 and switch state changes prints out the motor speed
   Normally set to zero
*/

/* Motor constructor
   constructor, setup pwm register, speed pin, direction pin
   set initial speed and direction
*/

Motor::Motor(uint8_t* par_pwm_reg, uint8_t par_direction_pin, uint8_t par_pwm_pin)
{
  currentSpeed   = 0;                   //set initial speed
  requestedSpeed = 0;                   //set initial speed
  requestedDir   = FORWARD;             //set initial direction
  currentDir     = FORWARD;             //set initial direction

  dir_Pin = par_direction_pin;          //store pin to be used to set the direction
  pwm_Reg = par_pwm_reg;                //store reg to be used to control speed via PWM signa
  
  pinMode(par_pwm_pin, OUTPUT);         //set PWM pin as an output
  pinMode(dir_Pin, OUTPUT);             //sets the direction pin as output
  digitalWrite(dir_Pin, currentDir);    //sets the initial direction
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

    currentSpeed = requestedSpeed;      //change less than max rate of change, so accept new value

    MOTOR_DEBUG_PRINT(" updated currentSpeed: ");
    MOTOR_DEBUG_PRINT(currentSpeed);
    /* scale speed to range of PWM. PWM range is 0 to 255, which is stopped to full speed for the motor. If the upper motor speed is to be restricted,
      then MOTOR_maxPower is set to something below 255 */
    uint8_t tmpSpeed = map(currentSpeed, JoystickMin, JoystickMax, 0, 255);
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


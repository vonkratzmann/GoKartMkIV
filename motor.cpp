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

/* setRequestedtSpeed
   stores the new requested speed
   it is up to other logical routines to change the actual speed
*/
void  Motor::setRequestedSpeed(int par_new_speed)
{
  requestedSpeed = par_new_speed;
  MOTOR_DEBUG_PRINT(__FUNCTION__, " requestedSpeed: ", MOTOR_DEBUG_PRINT(requestedSpeed) );
}

/* getRequestedtSpeed
   retrurns the requested speed, not speed the motor is actually doing
*/
int Motor::getRequestedSpeed(void)
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
  MOTOR_DEBUG_PRINT(__FUNCTION__, " requestedDir: ", requestedDir );
}

/* getRequestedtDir
   returns the the requested direction of the motor
*/
int Motor::getRequestedDir(void)
{
  return requestedDir;
}

/* getRequestedtSpeed
  returns the current speed, the speed the motor is actually doing
*/
int Motor::getCurrentSpeed(void)
{
  return currentSpeed;
}

/* getRequestedtDir
   returns the the current direction of the motor
*/
int Motor::getCurrentDir(void)
{
  return currentDir;
}

/* updatePowerToMotor
   updates the current powerto the motor by wrtiing to PWM register
*/
void Motor::updatePowerToMotor(uint8_t power)
{
  MOTOR_DEBUG_PRINT(__FUNCTION__, " currentSpeed: ", currentSpeed);
  MOTOR_DEBUG_PRINT(__FUNCTION__, " requestedSpeed: ", requestedSpeed);
  MOTOR_DEBUG_PRINT(__FUNCTION__, " power: ", power);

  * pwm_Reg = power;                 //output power to update PWM duty cycle
}

/* updateDir
   updates the current direction from the requested direction
   does not check if speed is appropriate to change direction
*/
void    Motor::updateDir(void)
{
  currentDir = requestedDir;
  digitalWrite(dir_Pin, currentDir);
}


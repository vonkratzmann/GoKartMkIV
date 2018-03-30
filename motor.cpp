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
  myPower   = 0;                          //set initial speed
  myDir     = FORWARD;                    //set initial direction

  dir_Pin = par_direction_pin;          //store pin to be used to set the direction
  pwm_Reg = par_pwm_reg;                //store reg to be used to control speed via PWM signa

  pinMode(par_pwm_pin, OUTPUT);         //set PWM pin as an output
  pinMode(dir_Pin, OUTPUT);             //sets the direction pin as output
  digitalWrite(dir_Pin, myDir);         //sets the initial direction
}


/* getPower
   retrurns the current power
*/
uint8_t Motor::getPower(void)
{
  return myPower;
}


/* getDir
   returns the the direction of the motor
*/
bool Motor::getDir(void)
{
  return myDir;
}


/* updatePower
   updates the current power to the motor by writting to PWM register
*/
void Motor::updatePower(uint8_t power)
{
  myPower = power;
  * pwm_Reg = myPower;                 //output power to update PWM duty cycle

  MOTOR_DEBUG_PRINT(__FUNCTION__, " myPower:", myPower);
  MOTOR_DEBUG_PRINT(__FUNCTION__, " myDir:", myDir);
}


/* updateDir
   updates the current direction from the requested direction
   does not check if speed is appropriate to change direction
*/
void    Motor::updateDir(bool dir)
{
  myDir = dir;
  digitalWrite(dir_Pin, myDir);
}


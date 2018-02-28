/** Class motor

*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"


class Motor
{
  private:
    int	          currentSpeed;	       //store current speed
    int           requestedSpeed;      //store the new requested speed
    int	          currentDir;          //store current direction
    int           requestedDir;        //store requested direction
    boolean       stopped;             //flag to say if motor stopped or moving. true is stopped, false indicates moving
    long          timeMovingForwards;  //used to track how long moving forwards
    long          timeMovingBackwards; //used to track how long been moving backwards

    uint8_t*      pwm_Reg;	      //Ouput reg to load duty cycle for pwm pulse 0 to 255
    int           dir_Pin;	      //output pin to set direction to the motor

  public:
    Motor(uint8_t*, uint8_t, uint8_t);	            //constructor, set PWM comparision register address, direction pin, and maximum speed for the motor
    void	setRequestedSpeed(int);
    int	  getRequestedSpeed(void);
    void	set_Requested_Dir(int);
    int  	getRequestedDir(void);
    int   getCurrentSpeed();
    int   getCurrentDir(void);
    void  updatePowerToMotor(uint8_t);
    void  updateDir(void);
};

#endif

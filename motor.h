/** Class motor

*/
#ifndef Motor_h
#define Motor_h

#include "Arduino.h"


class Motor
{
  private:
    uint8_t     	myPower;
    bool          myDir;
    uint8_t*      pwm_Reg;	      //Ouput reg to load duty cycle for pwm pulse 0 to 255
    int           dir_Pin;	      //output pin to set direction to the motor

  public:
    Motor(uint8_t*, uint8_t, uint8_t);	            //constructor, set PWM comparision register address, direction pin, and maximum speed for the motor
    uint8_t   getPower();
    bool      getDir(void);
    void      updatePower(uint8_t);
    void      updateDir(bool);
};

#endif

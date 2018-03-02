/** Class SlottedWheel

*/
#ifndef SlottedDisk_h
#define SlottedDisk_h

#include "Arduino.h"
#include "GoKartMkIV.h"

const int slotDebounceTime             = 50;     //debounce time in millisecones
const unsigned int noOfSlots           = 8;      //number of slots in disk
const unsigned int diskWheelDia        = 300;    //diameter of wheel with slotted disk in mm
const unsigned int diskWheelCircum     = 3.141 * diskWheelDia; //circumference of wheel with slotted disk in mm
/* for information only
   times(rounded)in millseconds between slots for different maximum speed and wheel diameters for a 100 slot disc
   calculated by: circumference of wheel(mm) / (speed(mm/sec)/1000) / no of slots
   Wheel Dia(mm)    100   200   250   300   350
   Max Speed(km/hr)
         5          2     5     6     7     8
         10         1     2     3     3     4
         15         .8    2     2     2     3
         20         .6    1     2     2     2
*/

class SlottedDisk
{
  private:
    uint8_t   sensorDin;              //pin used to read presence or not of a slot under the sensor
    bool			sensorState;		        //stores if a slot is under the detector,
    bool      lastSensorState;        //stores previous state
    bool      debouncedState;         //stores deounced state
    unsigned long      myTimeBetweenSlots = 0;
    unsigned int       wheelSpeedmmPerSec = 0;    //wheel speed in millmeters per second, use mm/sec units as provides appropiate size numbers to use a type integer, eg 15km/hr = 4,167mm/sec

  public:
    /* Slotted wheel ISR variables*/
    unsigned long timeBetweenSlots, timeSinceStartSlot;
    bool validSlotUnderSensor;
    
    SlottedDisk(uint8_t);	                            //constructor
    unsigned int  calculateSpeed(unsigned int);       //calculate speed of the wheel
    unsigned int getSpeed(void);                      //get speed of the wheel

};
#endif



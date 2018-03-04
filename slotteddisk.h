/** Class SlottedWheel

*/
#ifndef SlottedDisk_h
#define SlottedDisk_h

#include "Arduino.h"
#include "GoKartMkIV.h"

const unsigned long NoisePeriod        = 5000;      //pulses must be longer than this many microseconds otherwise they are considered noise
/*********************************************************************************************************
*                                            *** NOTE ***                                                *
* slotDebounceTime   needs to be adjusted if the maximum speed, wheel size or number of slots is changed *
*                                                                                                        *
**********************************************************************************************************/
const int slotDebounceTime             = 10000;   //debounce time in microseconds
const unsigned int noOfSlots           = 8;      //number of slots in disk
const unsigned int diskWheelDia        = 300;    //diameter of wheel with slotted disk in mm
const unsigned int diskWheelCircum     = 3.141 * diskWheelDia; //circumference of wheel with slotted disk in mm
/* for information only
   time between slots(rounded)in millseconds for different maximum speed and wheel diameters
                       100 slot disc                  8 slot disc
   Wheel Dia(mm)    100 200 250 300 350            100 200 250 300 350
   Max Speed(km/hr)
         5           2   5   6   7   9              31  61  77  92  108
         10          1   2   3   4   4              15  31  38  46   54
         15         .8   2   2   2   3              10  20  26  31   36
         20         .6   1   1   2   2               8  15  19  23   27         
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
    unsigned int  calculateSpeed(unsigned long);       //calculate speed of the wheel
    unsigned int getSpeed(void);                      //get speed of the wheel

};
#endif



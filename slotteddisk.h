/** Class SlottedWheel

*/
#ifndef SlottedDisk_h
#define SlottedDisk_h

#include "Arduino.h"
#include "GoKartMkIV.h"

const unsigned long NoisePeriod        = 250ul;      //pulses must be longer than this many microseconds otherwise they are considered noise
/*********************************************************************************************************
*                                            *** NOTE ***                                                *
* slotDebounceTime needs to be adjusted if the maximum speed, wheel size or number of slots is changed   *
*                                                                                                        *
**********************************************************************************************************/
const int slotDebounceTime             = 1000;   //debounce time in microseconds
const unsigned int noOfSlots           = 8;      //number of slots in disk
const unsigned int diskWheelDia        = 120;    //diameter of wheel with slotted disk in mm
const unsigned int diskWheelCircum     = 3.141 * diskWheelDia; //circumference of wheel with slotted disk in mm

/**************************************************************************************************************
                                      for information only                                                    *
   time between slots(rounded)in millseconds for different maximum speed, wheel diameters and number of slots *
                                                                                                              *
                           Slots                        Slots                           Slots                 *
                            100                          20                               8                   *
Wheel diameter mm 75  100 200 250 300 350   75  100   200  250  300  350     75   100  200  250  300  350     *
Speed km/hr                                                                                                   *                             
          5       1.8 2.5 4.9 6.1 7.4 8.6   9.2 12.3 24.6 30.7 36.9 43.0    23.0 30.7 61.5 76.8 92.2 107.5    *
         10       0.9 1.2 2.5 3.1 3.7 4.3   4.6  6.1 12.3 15.4 18.4 21.5    11.5 15.4 30.7 38.4 46.1  53.8    *
         15       0.6 0.8 1.6 2.0 2.5 2.9   3.1  4.1  8.2 10.2 12.3 14.3     7.7 10.2 20.5 25.6 30.7  35.8    *
         20       0.5 0.6 1.2 1.5 1.8 2.2   2.3  3.1  6.1  7.7  9.2 10.8     5.8  7.7 15.4 19.2 23.0  26.9    *
                                                                                                              *
***************************************************************************************************************/

class SlottedDisk
{
  private:
    uint8_t   sensorDin;              //pin used to read presence or not of a slot under the sensor
    bool			sensorState;		        //stores if a slot is under the detector,
    bool      lastSensorState;        //stores previous state
    bool      debouncedState;         //stores deounced state
    unsigned int    wheelSpeedmmPerSec = 0;                 //speed in millmeters/second, used as provides appropiate size numbers using type integer, eg 15km/hr = 4,167mm/sec
    unsigned long   timeSinceLastSpeedCalculation;          //record time last calculated speed, used to check if gokart is stopped
  public:
    /* Slotted wheel ISR variables*/
    unsigned long timeBetweenSlots, timeSinceStartSlot;
    bool validSlotUnderSensor;
    
    SlottedDisk(uint8_t);	                                //constructor
    unsigned int  calculateSpeed(unsigned long);          //calculate speed of the wheel
    unsigned int  getSpeed(void);                         //get speed of the wheel
    boolean       getSensorState(void);                   //get state of sensor without and debouncing or noise processing
    unsigned long getTimeSinceLastSpeedCalculation(void); //get time since last calculated the speed
};
#endif



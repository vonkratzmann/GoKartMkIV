/** Class SlottedWheel

*/
#ifndef SlottedDisk_h
#define SlottedDisk_h

#include "Arduino.h"
#include "GoKartMkIV.h"
      
const int debounceTime    = 1;    //debounce time in millisecones
const int noOfSlots      = 100;  //number of slots in disk
const int diskWheelDia    = 300; //diameter of wheel with slotted disk in mm
const int diskWheelCircum = 3.141 * diskWheelDia; //circumference of wheel with slotted disk in mm
const int MaxRevsPerSec = MaxSpeedKmh * 1000 *1000 / diskWheelDia / 3600 ; //maximum speed in revs per sec of sensor wheel
/* for information only
 * times(rounded)in millseconds between slots for different maximum speed and wheel diameters for a 100 slot disc
 * calculated by: circumference of wheel(mm) / (speed(mm/sec)/1000) / no of slots
 * Wheel Dia(mm)    100   200   250   300   350      
 * Max Speed(km/hr)
 *       5          2     5     6     7     8
 *       10         1     2     3     3     4
 *       15         .8    2     2     2     3 
 *       20         .6    1     2     2     2
 */
/* Note the value of ValidTimeBetweenSlots is only valid for wheels 200mm diameter and larger, and speeds 20km/hr or slower */
const int  minTimeBetweenSlots = 1;    //time between slots in milliseconds has to be greater than this used for debouncing slot sensor, 

class SlottedDisk
{
  private:
    uint8_t   sensorDin;              //pin used to read presence or not of a slot under the sensor
    bool			sensorState;		        //stores if a slot is under the detector,
    bool      lastSensorState;        //stores previous state
    bool      debouncedState;         //stores deounced state
    long      lastTimeChanged;        //store in millesconds, time when the sensor changed state
    long      timeOfLastSlot1;        //time since last slot was under the sensor 
    long      timeOfLastSlot2;        //time since second last slot was under the sensor 
    int       wheelSpeed_mmPerSec;    //wheel speed in millmeters per second, use mm/sec units as provides appropiate size numbers to use a type integer, eg 15km/hr = 4,167mm/sec 
    int       validTimeBetweenSlots;  //last valid time between slots

  public:
    SlottedDisk(uint8_t);	            //constructor
    void sensor_Check(void);          //check sensor for any changes and update sensor state
    void calculate_Speed(void);       //calculate speed of the wheel
    int get_Speed(void);              //get speed of the wheel
   
};
#endif



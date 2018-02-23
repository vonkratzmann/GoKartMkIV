/** Class SlottedWheel

*/
#ifndef SlottedDisk_h
#define SlottedDisk_h

#include "Arduino.h"
#include "GoKartMkIV.h"

//debounce time in millisecones
#define debounceTime    1
//number of slots in disk
#define noOfSlots       100
//diameter of wheel with slotted disk in mm
#define diskWheelDia    300
//circumference of wheel with slotted disk in mm

const int diskWheelCircum = 3.141 * diskWheelDia; //circumference of wheel with slotted disk in metres
const int MaxRevsPerSec = MaxSpeedKmh * 1000 / diskWheelDia / 3600; //maximum speed in revs per sec of sensor wheel

class SlottedDisk
{
  private:
    uint8_t   sensorDin;              //pin used to read presence or not of a slot under the sensor
    bool			sensorState;		        //stores if a slot is under the detector,
    bool      lastSensorState;        //stores previous state
    bool      debouncedState;         //stores deounced state
    long      lastTimeChanged;        //store in millesconds, time when the sensor changed state



  public:
    SlottedDisk(uint8_t);	            //constructor
    void sensor_Check(void);          //check sensor for any changes and update sensor state
};
#endif



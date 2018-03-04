#include "GoKartMkIV.h"
#include "slotteddisk.h"

/*
   Sensor disk has slots and as the disk rotates, sensor picks up if a slot is present or not
   Reads sensor pin , debounces state of the sensor and calculates speed of the wheel
*/

/* SlottedDisk constructor
   the configuration of the pin and interrupt is done in Setup() while setting up the interrupts
*/

SlottedDisk::SlottedDisk(uint8_t parSensorDin)
{
  sensorDin = parSensorDin;           //store digital input to read the
}

//calculate speed of the wheel
unsigned int  SlottedDisk::calculateSpeed(unsigned long slotTime)
{
  /* speed(mm/sec) = wheel diameter(mm) * 1000(convert millisec to seconds) / (time between slots (millisec) * number of slots) */
  wheelSpeedmmPerSec =  (long) diskWheelCircum  * 1000 / (slotTime * (long) noOfSlots);
  SENSOR_DEBUG_PRINT(__FUNCTION__, " slotTime:", slotTime, " wheelSpeedmmPerSec:", wheelSpeedmmPerSec);   //if  SENSOR_DEBUG defined print out results
  return wheelSpeedmmPerSec;
}

//get speed of the wheel
unsigned int SlottedDisk::getSpeed(void)
{
  return wheelSpeedmmPerSec;
}

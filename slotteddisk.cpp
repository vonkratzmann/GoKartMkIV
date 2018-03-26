#include "GoKartMkIV.h"
#include "slotteddisk.h"

/*
   Sensor disk has slots and as the disk rotates, sensor picks up if a slot is present or not
   Reads sensor pin , debounces state of the sensor and calculates speed of the wheel
   Averages the last 3 readings for the time between slots to calculate the speed
   time between slots is calculated by the ISR from the sensor pin
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
  myTimeBetweenSlots[slotTimeCounter++ % 3] = slotTime;                                                       //store results so can average speed
  unsigned long tmp =  (myTimeBetweenSlots[0] +  myTimeBetweenSlots[1] +  myTimeBetweenSlots[2]) / 3;         //average the last three times
  
  /* speed(mm/sec) = wheel diameter(mm) * 1000000(convert microsecs to seconds) / (time between slots (microsecs) * number of slots) */
  wheelSpeedmmPerSec =  (long) diskWheelCircum  * 1000000 / (slotTime * (long) noOfSlots);
  SENSOR_DEBUG_PRINT(__FUNCTION__, " slotTime(msec):", tmp/1000, " wheelSpeedmmPerSec:", wheelSpeedmmPerSec); //if SENSOR_DEBUG defined, print out results
  return wheelSpeedmmPerSec;
}                                                                                                             //end of calculateSpeed()


//get speed of the wheel
unsigned int SlottedDisk::getSpeed(void)
{
  return wheelSpeedmmPerSec;
}                             //end of getSpeed()


//get sensor state
boolean SlottedDisk::getSensorState() {
  return digitalRead(SensorDiskPin);
}                            //end of getSensorState

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

//check sensor for any changes and update sensor state
//return true if new slot under the sensor
bool SlottedDisk::sensorCheck(void)
{
  bool changeDetected = false;            //clear flag
  cli();
  bool tmp = validSlotUnderSensor;
  sei();
  if (validSlotUnderSensor)              //is there a new valid slot been detected?
  {
    SENSOR_DEBUG_PRINT(__FUNCTION__, " timeBetweenSlots:", timeBetweenSlots, " ", " ");
    cli();                                   //yes get the time between slots
    validSlotUnderSensor = false;           //clear flag used by the ISR
    myTimeBetweenSlots = timeBetweenSlots;  //get the time calculated by the ISR
    sei();
    Serial.println(myTimeBetweenSlots);
    changeDetected = true;
  }
  return changeDetected;
}


//calculate speed of the wheel
void SlottedDisk::calculateSpeed(void)
{
   if (myTimeBetweenSlots >= minTimeBetweenSlots)                         //do debounce check by checking the time between the last two slots is greater than specified min
    validTimeBetweenSlots = myTimeBetweenSlots;                          //as time between slots is ok, save it, otherwise ignore it

  /* speed(mm/sec) = wheel diameter(mm) / (time between slots (millisec) * 100  / 1000) = wheel diameter * 10 / time between slots  */
  wheelSpeedmmPerSec =  diskWheelCircum * 10 / validTimeBetweenSlots;
  SENSOR_DEBUG_PRINT(__FUNCTION__, " validTimeBetweenSlots:", validTimeBetweenSlots, " wheelSpeedmmPerSec:", wheelSpeedmmPerSec);   //if  SENSOR_DEBUG defined print out results
  return;
}

//get speed of the wheel
unsigned int SlottedDisk::getSpeed(void)
{
  return wheelSpeedmmPerSec;
}

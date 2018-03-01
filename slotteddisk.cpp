#include "GoKartMkIV.h"
#include "slotteddisk.h"

/*
   Sensor disk has slots and as the disk rotates, sensor picks up if a slot is present or not
   Reads sensor pin , debounces state of the sensor and calculates speed of the wheel
*/

/* SlottedDisk constructor
  setup sensor read pin,
*/

SlottedDisk::SlottedDisk(uint8_t parSensorDin)
{
  sensorDin = parSensorDin;           //store digital input to read the
  pinMode(sensorDin, INPUT);          //set digital input as an input
}

//check sensor for any changes and update sensor state
//return true if new slot under the sensor
bool SlottedDisk::sensorCheck(void)
{
  bool changeDetected = false;            //clear flag
  sensorState = digitalRead(sensorDin);   //read sensor to see if there has been a change
  unsigned long tmp = millis();           //get current time
  if (sensorState != lastSensorState)     //is there change?
  {
    SENSOR_DEBUG_PRINT(__FUNCTION__, " sensorState != lastSensorState ", " ", " sensorState:", sensorState);
    SENSOR_DEBUG_PRINT(__FUNCTION__, " lastTimeChanged:", lastTimeChanged, " tmp:", tmp);
    lastTimeChanged = tmp;                //yes, record the time of the change
    lastSensorState = sensorState;        //update state
    return changeDetected;                //have to wait until stable for the debounce time, so cannot do any more here
  }
  else if ((sensorState != debouncedState) && (( tmp - lastTimeChanged) > debounceTime)) //if no change, check if stable for longer then debounce period?
  {
    debouncedState = sensorState;         //yes, update state
    if (sensorState)                      //check if a slot has been detected
    {
      timeOfLastSlot2 = timeOfLastSlot1;  //yes, update the times, move last time for a slot to the second last time for a slot
      timeOfLastSlot1 = tmp;              //store the time this occurred so later can calculate the speed of the wheel
      changeDetected =  true;
      SENSOR_DEBUG_PRINT(__FUNCTION__, " timeOfLastSlot2:", timeOfLastSlot2, " timeOfLastSlot1:", timeOfLastSlot1);  //yes
    }
  }
  return changeDetected;
}


//calculate speed of the wheel
void SlottedDisk::calculateSpeed(void)
{
  unsigned long timeBetweenSlots = timeOfLastSlot1 - timeOfLastSlot2;     //ignore overflow of millis() after so many days, as goKart will be used for a couple of hours
  if (timeBetweenSlots >= minTimeBetweenSlots)                            //do debounce check by checking the time between the last two slots is greater than specified min
    validTimeBetweenSlots = timeBetweenSlots;                             //as time between slots is ok, save it, otherwise ignore it

  /* speed(mm/sec) = wheel diameter(mm) / (time between slots (millisec) * 100  / 1000) = wheel diameter * 10 / time between slots  */
  wheelSpeedmmPerSec =  diskWheelCircum * 10 / validTimeBetweenSlots;
  SENSOR_DEBUG_PRINT(__FUNCTION__, " timeBetweenSlots:", timeBetweenSlots, " wheelSpeedmmPerSec:", wheelSpeedmmPerSec);   //if  SENSOR_DEBUG defined print out results
  return;
}

//get speed of the wheel
unsigned int SlottedDisk::getSpeed(void)
{
  return wheelSpeedmmPerSec;
}

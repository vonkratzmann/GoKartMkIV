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

void SlottedDisk::sensor_Check(void)
{
  sensorState = digitalRead(sensorDin);   //read sensor to see if there has been a change
  if (sensorState != lastSensorState)     //is there change?
  {
    lastTimeChanged = millis();           //yes, record the time of the change
    lastSensorState = sensorState;        //update state
    return;                               //have to wait until stable for the debounce time, so cannot do any more here
  }
  else if ((sensorState != debouncedState) && ((millis() - lastTimeChanged) > debounceTime)) //if no change, check if stable for longer then debounce period?
  {
    debouncedState = sensorState;         //yes, update state
    if (sensorState)                      //check if a slot has been detected
    {
            timeOfLastSlot2 = timeOfLastSlot1;  //yes, update the times, move last time for a slot to the second last time for a slot 
            timeOfLastSlot1 = millis();         //store the time this occurred so later can calculate the speed of the wheel
    }
    SENSOR_DEBUG_PRINT(__FUNCTION__);
    SENSOR_DEBUG_PRINT(" sensor_State: ");
    SENSOR_DEBUG_PRINTLN(sensor_State);
  }
  return;
}

//calcuate speed of the wheel
void SlottedDisk::calculate_Speed(void)
{
  int timeBetweenSlots = timeOfLastSlot1 - timeOfLastSlot2;
  if (timeBetweenSlots >= minTimeBetweenSlots)     //do debounce check by checking the time between the last two slots is greater than specified min
      validTimeBetweenSlots = timeBetweenSlots;    //as time between slots is ok, save it, otherwise ignore it
      
/* speed(mm/sec) = wheel diameter(mm) / (time between slots (millisec) * 100  / 1000) = wheel diameter * 10 / time between slots  */     
  wheelSpeed_mmPerSec =  diskWheelCircum * 10 / validTimeBetweenSlots;

    SENSOR_DEBUG_PRINT(__FUNCTION__);
    SENSOR_DEBUG_PRINT(" timeBewtweenSlots: ");
    SENSOR_DEBUG_PRINT(timeBewtweenSlots);
    SENSOR_DEBUG_PRINT(" wheelSpeed_mmPerSec: ");
    SENSOR_DEBUG_PRINTLN(wheelSpeed_mmPerSec);
  return;
}

//get speed of the wheel
unsigned int SlottedDisk::get_Speed(void)     
{
  return wheelSpeed_mmPerSec;
}

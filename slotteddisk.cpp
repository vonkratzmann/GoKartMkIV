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

//Check for any changes in the sensor

void SlottedDisk::sensor_Check(void)
{
  sensorState = digitalRead(sensorDin);   //read sensor to see if there has been a change
  if (sensorState != lastSensorState)     //if does not equal last state
  {
    lastTimeChanged = millis();           //record time of change
    lastSensorState = sensorState;        //update state
    return;                               //have to wait until stable for the debounce time, so cannot do any more here
  }
  else if ((sensorState != debouncedState) && ((millis() - lastTimeChanged) > debounceTime)) //if no change, check if stable for longer then debounce period?
  {
    debouncedState = sensorState;         //yes, update state

    SENSOR_DEBUG_FILE("Function: ");
    SENSOR_DEBUG_FILE(__FILE__);
    SENSOR_DEBUG_FILE(",");
    SENSOR_DEBUG_PRINT(__FUNCTION__);
    SENSOR_DEBUG_PRINT(" sensor_State: ");
    SENSOR_DEBUG_PRINTLN(sensor_State);
  }
  return;
}






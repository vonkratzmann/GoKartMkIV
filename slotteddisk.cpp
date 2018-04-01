#include "GoKartMkIV.h"
#include "slotteddisk.h"

/*
   Sensor disk has slots and as the disk rotates, sensor picks up if a slot is present or not
   ISR is set up in main loop which interrupts on a change of state of the sensor input
   The ISR reads the sensor pin, checks for noise, debounces the state of the sensor and then calculates the time bewteen slots
   Functions below, calculates speed of the wheel by averaging the last 3 readings for the time between slots
*/

/* SlottedDisk constructor
   the configuration of the pin and interrupt is done in Setup() while setting up the interrupts
*/

SlottedDisk::SlottedDisk(uint8_t parSensorDin) {
  sensorDin = parSensorDin;           //store digital input to read the
}


//calculate speed of the wheel
unsigned int  SlottedDisk::calculateSpeed(unsigned long slotTime) {
  myTimeBetweenSlots[slotTimeCounter++ % 3] = slotTime;                                                         //store results so can average speed
  unsigned long tmp =  (myTimeBetweenSlots[0] +  myTimeBetweenSlots[1] +  myTimeBetweenSlots[2]) / 3;           //average the last three times

  /* speed(mm/sec) = wheel diameter(mm) * 1000000(convert microsecs to seconds) / (time between slots (microsecs) * number of slots) */
  if (tmp)                                                                                                      //check tmp not zero before calculate wheel speed with divide
    wheelSpeedmmPerSec =  (long) diskWheelCircum  * 1000000 / (tmp * (long) noOfSlots);
  else                                                                                                          //tmp is zero, so speed is stopped
    wheelSpeedmmPerSec = 0;
  SENSOR_DEBUG_PRINT(__FUNCTION__, " slotTime(msec):", tmp / 1000, " wheelSpeedmmPerSec:", wheelSpeedmmPerSec); //if SENSOR_DEBUG defined, print out results
  timeSinceLastSpeedCalculation = millis();                                                                     //save time, if no sensor pulse for nominated period say wheel is stopped
  return wheelSpeedmmPerSec;
}                                                                                                               //end of calculateSpeed()


//get speed of the wheel
unsigned int SlottedDisk::getSpeed(void) {
  return wheelSpeedmmPerSec;
}                             //end of getSpeed()


//get sensor state
/* used to read sensor state, calculation of time between slots is done by the interrupt service routine
 * this function is not used by the ISR, this is used for hardware testing
 */
boolean SlottedDisk::getSensorState() {
  return digitalRead(SensorDiskPin);
}                            //end of getSensorState


//get Time Since Last Speed Calculation
unsigned long SlottedDisk::getTimeSinceLastSpeedCalculation(void) {
  return timeSinceLastSpeedCalculation;
}


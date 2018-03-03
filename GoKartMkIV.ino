/** Drives a small electric goKart
   Author:    Kirk Kratzmann
   Version:   1.0
   Date:      13/12/2017
*/


/** System Parameters
    Maximum speed 15km/hour in forward direction
    Maximum reverse speed is half the maximum forward speed

*/

/** System Harwdare Summary
   System consists of:
   Two 12v DC motors
   A joystick connected to two 10 bit ADCS, forwards 0 to 511,backwards 512 to 1023
   On/Off switch
   12V DC battery
   Microprocessor - Arduino Uno running a ATmega328
   Driver Printed Circuit Boards for the two DC motors
   8v DC power supply for the microprocessor
   Each motor via a sprocket drives a chain to the two rear wheels

   The program reads the joystick at a preddefined rate
   then converts these readings into a Pulse width Modulated output and direction to drive the motors direction and speed

   Uses counter 2 and interrupts to generate fast mode pwm pulses, freq is 1.960kHz
   Normal operation the diagnostic led fashes on and off every second via the ISR

   There are a number of simple diagnostics which can be enable by the #DEFINEs in GoKartMkIII.h
*/

/** Software Structure Overview
   The system consists of 6 files:
    GOKartMKIV.ino:  Variable declarations, oject definitions, ISR, Setup and main loop
    GOKartMKIV.h:    Diagnostic definitions, I/O and constants
    joystick.cpp:  Joystick class member functions
    joystick.h:    Joystick class decelerations
    motor.cpp:     Motor class member functions
    motor.h:       Motor class decelerations

    The main loop logic for the x axis is:
    check if time to flash the onboard led on or off, used to show program and isr are running
    check if time to read the joystick, then
    check if x axis of joystick has changed and check if y axis of joystick has changed by calling "check_X_Axis()" and "check_Y_Axis()"

    if either one has changed process the change,
    as y changes speed forwards or backwards, always process y before x
    use the y speed to firstly set the speed and direction for either forwards or backwards by calling process_Y()
    the reverse speed maximum has a seperate limit to the forward speed to prevent moving in reverse too quickly
    then use x to increase or decrease the speed of the left or right wheels to enable turning by calling process_X()
    to turn slow down the wheel of the direction you want to turn and leave the other wheel unchanged

    Note the value of X is scaled as a percentage of the current y speed to limit turning rate.

    Then calls:
      "left_Motor.update_Speed() to update motor speed from the requested speed
      "left_Motor.updateDir()" to update motor direction from the requested direction
      "right_Motor.update_Speed()" to update motor speed from the requested speed
      "right_Motor.updateDir()" to update motor direction from the requested directio
*/

#include "GoKartMkIV.h"

#include "joystick.h"
#include "motor.h"
#include "slotteddisk.h"
#include <PID_v1.h>

unsigned int  interrupt_Counter = 0;           //used in main loop to show the ISR is running
unsigned long joys_Time_Of_Last_Scan = 0;      //track when we last scanned for joystick changes
unsigned int  goKartSpeed = 0;                 //stores speed of GoKart

uint8_t led = LOW;                            //state of led, initially off

/* define objects */

/* define joystick */
JoyStick js;  //define joystick

/* define motors */
Motor left_Motor(leftPwmReg, leftDirPin, leftPwmPin);
Motor right_Motor(rightPwmReg, rightDirPin, rightPwmPin);

/* define slotted disk */
SlottedDisk mySlottedDisk(SensorDiskPin);

/* Interrupt Service Routine for when counter overflows in timer 2 */

ISR(TIMER2_OVF_vect)
{
  interrupt_Counter = interrupt_Counter + 1;        //used to show we are alive and ISR running
  return;
}

/*
   Slot detector ISR
   interrupts on change of state of input from slotted wheel sensor
   does a quick check for noise by ensuring pulses are above a specified size, if not ignorethat pulse
   when a slot is under the sensor, the signal is high
   on rising edge records the time
   on a falling edge, check how long it has been in the high state and if the sensor has been stable in the high state ie for the debounce period
   says that a valid slot has been under the sensor, calculates the time between slots, and sets the flag for the main loop to read and process
*/
unsigned int          timeSinceStartSlot;
unsigned int          timeSinceStartPreviousSlot;
volatile unsigned int timeBetweenSlots;
volatile bool         validSlotUnderSensor;
unsigned long         lastPinChangeTime ;

ISR(INT0_vect)
{
  unsigned long tmp = millis();                   //get the current time
  if (tmp - lastPinChangeTime <= NoisePeriod)    //check not a noisy pulse
  {
    lastPinChangeTime = tmp;              //yes just noise, ignore, save the time so we can compare aagainst the next pulse edge
    return;
  }
  lastPinChangeTime = tmp;                //save time
  if (digitalRead(SensorDiskPin))         //its high, so it is it a LOW to HIGH pin change?
  {
    timeSinceStartSlot = tmp;             //yes, save time of low to high change, ie the start of a slot
    validSlotUnderSensor = false;         //clear flag to say we have just had a valid slot detected pulse
  }
  else                                    //no, its low, so it is a HIGH to LOW pin change, ie the end of a slot
  {
    if (tmp - timeSinceStartSlot > slotDebounceTime)          //check if time pulse was high was greater than debounce period, if not ignore
    {
      validSlotUnderSensor = true;                            //yes, we've just had a valid slot pulse
      timeBetweenSlots = tmp - timeSinceStartPreviousSlot;    //work out time bewteen slots
      timeSinceStartPreviousSlot = timeSinceStartSlot;        //save time for next calculation
    }
  }
}
// PID Tuning parameters
float Kp = 1;   //Proportional Gain
float Ki = 2;   //Integral Gain
float Kd = 5;   //Differential Gain
double setpoint = 0, input = 0, output = 0;     //These are just variables for storing values, assume all stopped at startup

/* define PID Loops, Input is our PV, Output is our u(t), Setpoint is our SP */
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, P_ON_E, DIRECT);          //constructor also call set tunnings

/** Setup */
void setup(void)
{
  /* set up inputs using internal pull up resistors and set up outputs */
  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, HIGH);     // sets the LED on

  Serial.begin(9600);             //set up serial port for any debug prints

  /* Set up timer interrupt
    Timer 0, is 8 bits used by fuction millis();
    Timer 2, is 8 bits and is used to generate an interrupt approximately every 500 microseconds and to process pwm pulses to motors
    Timer is 16x10^6 (clock speed) / [prescaler x 255];  for prescaler of 32, frequeny of interrupts and PWM is 1.960kHz
    Use fast PWM mode, where the timer repeatedly counts from 0 to 255. The output turns on when the timer is at 0, and turns off when the timer matches the output compare register OCR2A and OCR2B.
    The higher the value in the output compare register, the higher the duty cycle.

    // ----- TIMER 2 -----
    // TCCR2A - Timer/Counter control register A
    // Bit  |    7     |    6     |    5     |    4     |  3  |  2  |    1    |    0    |
    //      |  COM2A1  |  COM2A0  |  COM2B1  |  COM2B0  |  -  |  -  |  WGM21  |  WGM20  |

    // TCCR2B - Timer/Counter control register B
    // Bit  |    7     |    6     |    5     |    4     |    3    |    2    |    1   |    0   |
    //      |  FOC2A   |  FOC2B   |    -     |    -     |  WGM22  |  CS22   |  CS21  |  CS20  |

    //TIMSK2 – Timer/Counter2 Interrupt Mask Register
    // Bit  |    7     |    6     |    5     |    4     |     3   |  2       |    1     |    0    |
    //      |    -     |    -     |    -     |    -     |    -    | OCIE2B   |  OCIE2A  |  TOIE2  |
  */
  cli();                                      //stop interrupts

  /*
    On TCCR2A, setting the COM2A bits and COM2B bits to 10 provides non-inverted PWM for outputs A and B,
    Clears OC2A & OC2B on Compare Match, set OC2A & OC2B at BOTTOM,
    setting the waveform generation mode bits WGM to 011 selects fast PWM
  */
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  /* On TCCR2B, setting the CS bits to 011 sets the prescaler to divide the clock by 32 */
  TCCR2B  = _BV(CS21) | _BV(CS20);
  /* On TIMSK2, setting the TOIE2 bits to 1 enables the Timer/Counter2 Overflow interrupt. */
  TIMSK2 = _BV(TOIE2);

  /* Set up digital input interrupts for the slotted disk input
    uses digital pin 5, this is ATmega PD5 PCINT21 OC0B/T1
  */

  pinMode(SensorDiskPin, INPUT);       // equivalent to DDRD  &= ~(1 << DDD5)clear the PD5 pin, so PD5 is now an input
  //  digitalWrite(SensorDiskPin, HIGH);   //turn on pullup register
  EICRA  |= (1 << ISC00);              //set to trigger on any logic change
  EIMSK  |= (1 << INT0);               //enable interrupts on INT0

  sei();                                      //enable interrupts  */
  /* set up PID */
  myPID. SetSampleTime(JoyStickScanRate * JoystickToPidSampleTime);                              //set how often run PID as a multiple of JoystickScanRate, so PID run less often than rate scan joystick
  myPID.SetMode(AUTOMATIC);                 //Turn on the PID loop

  return;
}  //  end of setup()

/** Main Loop

*/
void loop(void)
{
  if (interrupt_Counter >= OneSec )          //check if a second has expired
  {
    toggleLed();                              //yes, flash the led
    reset_Counter();                          //reset counter
  }
  /* check if a valid slot under the sensor */
  unsigned int tmp = 0;
  cli();                                      //disable interrupts as about to read variables accessed by ISR
  if (validSlotUnderSensor)                   //get state of sensor that is set by the ISR
  {
    tmp = timeBetweenSlots;                   //just had a valid slout under the sensor get the time between slots calculated by the ISR
    validSlotUnderSensor = false;             //clear flag, this is set by the ISR
  }
  sei();

  if (tmp)                                    //check if a new slot was just under the sensor
  {
    goKartSpeed = mySlottedDisk.calculateSpeed(tmp);        //yes, calculate wheel speed
  }

  /* check if time to scan joystick for changes to x and Y axis */
  if ((millis() - joys_Time_Of_Last_Scan) > JoyStickScanRate)
  {
    joys_Time_Of_Last_Scan = millis();          //yes, reset timer, check x and Y axis for changes
    if (js.check_X_Axis() || js.check_Y_Axis()) //check if x axis of joystick has changed or y axis of joystick has changed
    {
      /* yes one has changed, so as y changes speed forwards or backwards, always process y before x
        use the y speed to firstly set the speed and direction for either forwards or backwards,
        then use x to increase or decrease the speed of the left or right wheels to enable turning;
        to turn slow down the wheel of the direction you want to turn and leave the other wheel unchanged */

      unsigned int xTurningDegrees,  yReqSpeed;       //local variables for storing values
      uint8_t leftPower, rightPower;                  //local variables for storing values
      bool  xDir = LEFT,  yDir = FORWARD;             //local variables for storing values

      js.process_Y(&yReqSpeed, &yDir);                //get requested speed and direction for y axis
      if (yDir == BACK)                               //check if direction is BACK or reverse
        yReqSpeed = yReqSpeed / ReverseSpeedSlower;   //yes, slow down reverse speed as a safety measure

      js.process_X(&xTurningDegrees, &xDir);          //get position of x axis?

      MAIN_LOOP_DEBUG_PRINT(__FUNCTION__, " xTurningDegrees:", xTurningDegrees, " xDir:", xDir, " yReqSpeed:", yReqSpeed, " yDir:", yDir);   //if MAIN_LOOP_DEBUG defined print out results to the serial monitor

      input     = map((double) goKartSpeed, 0, MaxSpeedmmPerSec, 0, MaxPower);     //Get current goKart speed and change the scale to the PWM power output scale
      setpoint  = map((double) yReqSpeed,   0, MaxSpeedmmPerSec, 0, MaxPower);     //get setpoint requested by the joystick and change the scale to the PWM power output scale

      if (myPID.Compute())                                              //Run the PID loop, and check something has changed, PID function has the addresses of Input, Setput, and Output variables so updated from the compute function
      {
        leftPower = (uint8_t) output; rightPower = (uint8_t) output;    //yes, now have the output from the PID, update power to each wheel taking account of turn request from the x axis
        if (xTurningDegrees != 0 && xDir == LEFT)                       //is it a request to turn and the request is to the left
          leftPower = map( xTurningDegrees, 90, 0, 0, output);          //yes, slow left wheel, by reducing the power by the amount of turning degrees, & leave right wheel unchanged
        else if (xTurningDegrees != 0 && xDir == RIGHT)                 //no, must be request to move right
          rightPower = map( xTurningDegrees, 90, 0, 0, output);         //slow right wheel speed, by the amount of turning degrees, & leave left wheel unchanged

        MAIN_LOOP_DEBUG_PRINT("     input:", input, " setpoint:", setpoint, " leftPower:", leftPower, " rightPower:", rightPower, " ");   //if MAIN_LOOP_DEBUG defined print out results to the serial monitor

        left_Motor.updatePower(leftPower);            //update motor power from the PID output after taking into account turning
        left_Motor.updateDir(yDir);                   //update motor direction from the y axis as it determines forwards or backwards
        right_Motor.updatePower(rightPower);          //update motor speed from the requested speed after taking into account turning
        right_Motor.updateDir(yDir);                  //update  motor direction from the y axis as it determines forwards or backwards
      }
    }
  }
}
//end of loop()
//-------------------------------------

void toggleLed(void)
{
  led ? led = LOW : led = HIGH;              //swap led state from high to low or low to high
  digitalWrite(LedPin, led);                 //update the led
}

void reset_Counter(void)
{
  cli();                                     //interrupts off
  interrupt_Counter = 0;                     //reset counter
  sei();                                     //interrupts on
}

/* end GoKartMkIII */


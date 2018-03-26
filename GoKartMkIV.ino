/** Drives a small electric goKart
   Author:    Kirk Kratzmann
   Version:   1.0
   Date:      13/12/2017
*/

/** System Parameters
    Maximum speed is a nomimanted speed,
      maximum physical speed is function of motor, gearing and wheel diameter
      Motor loaded rpm range: 2600 3000 rpm, Average 2800 rpm
      Motor sprocket 9 teeth
      Drive wheel sprocket 68 teeth
      Drive wheel diameter 250mm
      Maximum physical speed = 2800 * 9 / 68 * (3.141 * .250) / 1000 * 60 = 17.5 km/hr
    Maximum reverse speed is a nominated fraction of the nominated maximum forward speed
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
#include "hardwaretests.h"
#include <PID_v1.h>

unsigned int  interrupt_Counter = 0;          //used in main loop to show the ISR is running
unsigned long joys_Time_Of_Last_Scan = 0;     //track when we last scanned for joystick changes
unsigned int  goKartSpeed = 0;                //stores speed of GoKart

unsigned int  xTurningDegrees,  yReqSpeed;    //used in main loop to return values from joystick processing
uint8_t       leftPower, rightPower;          //used in main loop
bool          xDir = LEFT,  yDir = FORWARD;   //used in main loop

uint8_t led = LOW;                            //state of led, initially off
bool lastSensorState;                         //used by diagnostics

/* define objects */

#ifdef HardwareTest
/* define Hardware Tests */
HardwareTests myHardwareTests;
#endif

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
   does a quick check for noise by ensuring pulses are above a specified size, if not ignore that pulse
   when a slot is under the sensor, the signal is high
   on rising edge records the time in microseconds
   on a falling edge, check how long it has been in the high state and if the sensor has been stable in the high state ie for the debounce period
   says that a valid slot has been under the sensor, calculates the time between slots, and sets the flag for the main loop to read and process
   Uses function micros() rather than millis() as with some disks with 100 slots the time between slots is only a few milliseconds.
   At the moment ignore the fact that the microseconds will overflow in 70 minutes
*/
unsigned long          timeSinceStartSlot;
unsigned long          timeSinceStartPreviousSlot;
volatile unsigned long timeBetweenSlots;
volatile bool          validSlotUnderSensor;
unsigned long          lastPinChangeTime ;

ISR(INT0_vect)
{
  unsigned long tmp = micros();                  //get the current time in microseconds
  if (tmp - lastPinChangeTime <= NoisePeriod)    //check not a noisy pulse
  {
    lastPinChangeTime = tmp;              //yes just noise, ignore, save the time so we can compare against the next pulse edge
    return;
  }
  lastPinChangeTime = tmp;                //save the time, so can check the next pulse
  /* Note line below has to be changed if the SensorDsikPin value is changed */
  if (PIND &= B00000100)                  //read sensor directly rather than use DigitalRead(), its high, so it is it a LOW to HIGH pin change
  {
    timeSinceStartSlot = tmp;             //yes, save time of low to high change, ie the start of a slot
    validSlotUnderSensor = false;         //clear flag to say we have just had a complete valid slot under sensor
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
}                                         //end of ISR(INT0_vect)
// PID Tuning parameters
float Kp = 1;   //Proportional Gain
float Ki = 2;   //Integral Gain
float Kd = 5;   //Differential Gain
double setpoint = 0, input = 0, output = 0;     //These are variables accessed by the PID Compute() function and code in the main loop, assume all stopped at startup

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

  pinMode(SensorDiskPin, INPUT);          // equivalent to DDRD  &= ~(1 << DDD5)clear the PD5 pin, so PD5 is now an input
  //  digitalWrite(SensorDiskPin, HIGH);  //turn on pullup register
  EICRA  |= (1 << ISC00);                 //set to trigger on any logic change
  EIMSK  |= (1 << INT0);                  //enable interrupts on INT0

  sei();                                  //enable interrupts
  /* set up PID */
  myPID. SetSampleTime(JoyStickScanRate * JoystickToPidSampleTime);   //set how often run PID as a multiple of JoystickScanRate, so PID run less often than rate scan joystick
  myPID.SetMode(AUTOMATIC);               //Turn on the PID loop

#ifdef HardwareTest
  myHardwareTests.displayHelpMsg();                                 //on startup print command help message
#endif
  return;
}  //  end of setup()

/*** Main Loop ***/

void loop(void) {
  if (interrupt_Counter >= OneSec ) {
    toggleLed();                              //yes, flash the led, to show something is happening
    reset_Counter();                          //reset counter
  }                                           //run hardware tests or normal main loop code, if HardwareTest defined, run the HardwareTest code, otherwise run normal main loop code
#ifdef HardwareTest
  while (Serial.available () > 0)                                         //run hardware test, this code is normally only used for setup of H/W, and normally not compiled
    myHardwareTests.processIncomingByte (Serial.read ());                 //read typed input from serial monitor
  if (myHardwareTests.motorCommandEntered()) {                            //check if a new command has been entered
    left_Motor.updatePower((uint8_t)myHardwareTests.getLeftTestSpeed());  //yes, update motor power from the entered speed
    left_Motor.updateDir(myHardwareTests.getLeftTestDir());               //update motor dir from the entered direction
    right_Motor.updatePower((uint8_t)myHardwareTests.getRightTestSpeed());//update motor power from the entered speed
    right_Motor.updateDir(myHardwareTests.getRightTestDir());             //update motor dir from the entered direction
    myHardwareTests.clearMotorCommandEnteredFlag();                       //clear flag ready for the next command

    Serial.print("Leftspeed:"); Serial.print(myHardwareTests.getLeftTestSpeed()); Serial.print(" Rightspeed:"); Serial.print(myHardwareTests.getRightTestSpeed());
    Serial.print(" Leftdir:");  Serial.print(myHardwareTests.getLeftTestDir());   Serial.print(" Rightdir:");   Serial.print(myHardwareTests.getRightTestDir());
    Serial.print(" Leftpower:"); Serial.print(left_Motor.getPower()); Serial.print(" Rightpower:"); Serial.println(right_Motor.getPower());
  }
  if (myHardwareTests.getJoystickDisplayTime())                                             //if non zero time, display both joystick axis
    if ((millis() - joys_Time_Of_Last_Scan) > myHardwareTests.getJoystickDisplayTime()) {   //check if enough time elaspsed since last time to display joystick x and y axis
      joys_Time_Of_Last_Scan = millis();                                                    //yes, save time joystick last displayed
      Serial.print("Joystick X:"); Serial.print(js.get_X_Axis()); Serial.print(" Y:"); Serial.println(js.get_Y_Axis());
    }
  if (myHardwareTests.getSensorDisplayFlag()) {                           //check if to display sensor
    bool state = mySlottedDisk.getSensorState();
    if (lastSensorState != state) {                                       //has sensor state changed?
      Serial.print("Sensor state:"); Serial.println(state);               //yes, print out the state
      lastSensorState = state;                                            //save new state
    }
  }                                                                       //end hardware tests

#else
  if ((millis() - joys_Time_Of_Last_Scan) > JoyStickScanRate) { //run normal program, check if time to scan joystick for changes to x and y axis
    joys_Time_Of_Last_Scan = millis();          //yes, save time joystick last scanned, calculate the speed and then check x and Y axis for changes
    /* check if a valid slot under the sensor */
    unsigned long tmp = 0;                      //used to store the timing data from the ISR
    cli();                                      //disable interrupts as about to read variables accessed by ISR
    if (validSlotUnderSensor) {                 //get state of speed sensor that is set by the ISR
      tmp = timeBetweenSlots;                   //just had a valid slot under the sensor, get the time between slots calculated by the ISR
      validSlotUnderSensor = false;             //clear flag, this is set by the ISR
    }
    sei();                                      //interrupts back on
    if (tmp)                                    //check if a slot has been under the sensor
      goKartSpeed = mySlottedDisk.calculateSpeed(tmp);        //yes, calculate wheel speed

    if (js.check_X_Axis() || js.check_Y_Axis()) { //check if x axis or y axis of joystick has changed
      /* yes one has changed, so as y determines speed forwards or backwards, always process y before x
        use the y speed to firstly set the speed and direction for either forwards or backwards,
        then use x to increase or decrease the speed of the left or right wheels to enable turning;
        to turn, slow down the wheel of the direction you want to turn and leave the other wheel unchanged */

      js.process_Y(&yReqSpeed, &yDir);                //yes, get requested speed and direction for y axis
      if (yDir == BACK)                               //check if direction is BACK or reverse
        yReqSpeed = yReqSpeed / ReverseSpeedSlower;   //yes, slow down reverse speed as a safety measure
      js.process_X(&xTurningDegrees, &xDir);          //get position of x axis?
      MAIN_LOOP_DEBUG_PRINT(__FUNCTION__, " xTurningDegrees:", xTurningDegrees, " xDir:", xDir, " yReqSpeed:", yReqSpeed, " yDir:", yDir);   //if MAIN_LOOP_DEBUG defined print out results to the serial monitor
    }
    input     = map((double) goKartSpeed, 0, MaxSpeedmmPerSec, 0, MaxPower);     //Get current goKart speed and change the scale to the PWM power output scale
    setpoint  = map((double) yReqSpeed,   0, MaxSpeedmmPerSec, 0, MaxPower);     //get setpoint requested by the joystick and change the scale to the PWM power output scale

    if (myPID.Compute()) {                                            //Run the PID loop, and check something has changed, PID function has the addresses of Input, Setput, and Output variables so these updated from within the Compute()
      leftPower = (uint8_t) output; rightPower = (uint8_t) output;    //yes, now have the output power from the PID, update power to each wheel taking account of turn request from the x axis
      if (xTurningDegrees != 0 && xDir == LEFT)                       //is it a request to turn and the request is to the left
        leftPower = map( xTurningDegrees, 90, 0, 0, output);          //yes, slow left wheel, by reducing the power by the amount of turning degrees, & leave right wheel unchanged
      else if (xTurningDegrees != 0 && xDir == RIGHT)                 //no, check if a request to move right
        rightPower = map( xTurningDegrees, 90, 0, 0, output);         //yes, slow right wheel speed, by the amount of turning degrees, & leave left wheel unchanged

      MAIN_LOOP_DEBUG_PRINT("     input:", input, " setpoint:", setpoint, " leftPower:", leftPower, " rightPower:", rightPower, " ");   //if MAIN_LOOP_DEBUG defined print out results to the serial monitor
      left_Motor.updatePower(leftPower);            //update motor power from the PID output after taking into account turning
      left_Motor.updateDir(yDir);                   //update motor direction from the y axis as it determines forwards or backwards
      right_Motor.updatePower(rightPower);          //update motor speed from the requested speed after taking into account turning
      right_Motor.updateDir(yDir);                  //update  motor direction from the y axis as it determines forwards or backwards
    }
  }                                                 // end of normal code
#endif
}
void toggleLed(void)
{
  led ? led = LOW : led = HIGH;              //swap led state from high to low or low to high
  digitalWrite(LedPin, led);                 //update the led
}                                           //end of toggleLed(void)

void reset_Counter(void)
{
  cli();                                     //interrupts off
  interrupt_Counter = 0;                     //reset counter
  sei();                                     //interrupts on
}                                           //end of reset_Counter(void)



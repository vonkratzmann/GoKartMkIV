/** Drives a small electric goKart
   Author:    Kirk Kratzmann
   Version:   1.0
   Date:      13/12/2017
*/

/** System Parameters **
    Maximum speed is a nomimanted speed,
      maximum physical speed is function of motor, gearing and wheel diameter
      Motor loaded rpm range: 2600 3000 rpm, Average 2800 rpm
      Motor sprocket 9 teeth
      Drive wheel sprocket 68 teeth
      Drive wheel diameter 250mm
      Maximum physical speed = 2800 * 9 / 68 * (3.141 * .250) / 1000 * 60 = 17.5 km/hr
    Maximum reverse speed is a nominated fraction of the nominated maximum forward speed, slowed for safety reasons

** System Harwdare Summary **
   System consists of:
   Two 12v DC motors
   A joystick connected to two 10 bit ADCS,
   On/Off switch
   12V DC battery
   Microprocessor - Arduino Uno running a ATmega328
   Driver Printed Circuit Boards for the two DC motors
   small DC to DC power supply to limit voltage level input for the microprocessor
   Each motor via a sprocket drives a chain to the two rear wheels
   Rear castor wheel with a slotted disk passing under a sensor used to calculate the speed

   The main program checks if a slot has been detected by the sensor and calculates the gokart speed
   reads the joystick at a preddefined rate
   converts these readings into a setpoint for a PID algorithm along with the current speed from the sensor.
   The PID then calculates the power output power for the motors
   This power ouput is converted to a Pulse Width Modulated output and along with direction is used to drive the motors

   Normal operation the diagnostic led fashes on and off every second

   The main loop has into two mutually exclusive parts, :
   1. is used to test the hardware by a simple command interface via the serial monitor. A define statement "#define HardwareTest" in GoKartMkIV.h enables
   this code and the normal main loop code is not run
   2. is the normal main program. This has a number of simple debug print statements which can be enable by the #DEFINEs in GoKartMkIV.h
   these can be used to debug the code in the normal main loop.

   With regard to testing and debugging there is some overlap between the two parts of the main program

** Software Structure Overview **
   The system consists of the following files:
    GOKartMKIV.ino:   Variable declarations, oject definitions, ISR, Setup and main loop
    GOKartMKIV.h:     Diagnostic definitions, I/O and constants
    joystick.cpp:     Joystick class member functions
    joystick.h:       Joystick class decelerations
    motor.cpp:        Motor class member functions
    motor.h:          Motor class decelerations
    slotteddisk.cpp:  Slotted disk(wheel speed sensor) class member functions
    slotteddisk.h:    Slotted disk class decelerations
    hardwaretests.cpp:Hardwaretests  class member functions
    hardwaretests.h:  Hardwaretests Motor class decelerations
    pid.cpp:          Modified version od Standard Arduino PID library
    pid.h:            PID class member functions


    The main loop logic for the joystick is:
    check if time to read the joystick, then
    check if x axis of joystick has changed and check if y axis of joystick has changed by calling "check_X_Axis()" and "check_Y_Axis()"

    if either one has changed process the change,
    as y changes speed forwards or backwards, always process y before x
    use the y to firstly set the requested speed and direction for either forwards or backwards by calling process_Y()
    as a safety measure, the reverse requested speed is a fraction of the forward requested speed
    This requested speed is the setpoit for the PID alogorithm, which calculates the output power
    Then use x to increase or decrease the requested speed of the left or right wheels to enable turning by calling process_X()
    to turn slow down the wheel of the direction you want to turn and leave the other wheel unchanged
    Then calls:
      "left_Motor.updatePower() to update motor speed from the requested speed
      "left_Motor.updateDir()" to update motor direction from the requested direction
      "right_Motor.updatePOwer()" to update motor speed from the requested speed
      "right_Motor.updateDir()" to update motor direction from the requested direction

      *** Note ****
      as motor sprockets face each other, ie orientated by 180 degrees,
      normally the direction of the left motor would be complemented compared to the right, so both wheels turn in the same direction.
      but although the motors are meant to be identical, the direction the motor rotates for a specified polarity is different
      so each motor has to be tested indvidually and the software set up accordingly or they have to be wired differently.
*/

#include "GoKartMkIV.h"
#include "joystick.h"
#include "motor.h"
#include "slotteddisk.h"
#include "hardwaretests.h"
#include "pid.h"


unsigned long  mainLoopCounter = 0ul;         //count number of times around the main loop, used to give indication of CPU utilisation 
unsigned int  ledFlashCounter = 0;            //used in main loop to show the program is running by flashing the led
unsigned long joys_Time_Of_Last_Scan = 0ul;   //track when we last scanned for joystick changes
unsigned int  goKartSpeed = 0;                //stores speed of GoKart

unsigned int  xTurningDegrees = 0;            //used in main loop to return values from joystick processing
unsigned int  yReqSpeed = 0;                  //used in main loop to return values from joystick processing
uint8_t       leftPower, rightPower;          //used in main loop
bool          xDir = LEFT,  yDir = FORWARD;   //used in main loop
bool          currentDir = FORWARD;           //used in main loop, to check before change direction

uint8_t led = LOW;                            //state of led, initially off

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

/*
   Slot detector ISR
   interrupts on change of state of input from slotted wheel sensor
   does a quick check for noise by ensuring pulses are above a specified size, if not ignore that pulse
   when a slot is under the sensor, the signal is high
   on rising edge records the time in microseconds
   on a falling edge, check how long it has been in the high state and if the sensor has been stable in the high state ie for the debounce period
   says that a valid slot has been under the sensor, calculates the time between slots, stores the time for this valid slot for when we get the next slot
   and sets the flag for the main loop to read and process
   Uses function micros() rather than millis() as with some disks with 100 slots the time between slots is only a few milliseconds.
   At the moment ignore the fact that the microseconds will overflow in 70 minutes
*/
unsigned long          timeSinceStartSlot;
unsigned long          timeSinceStartPreviousSlot;
volatile unsigned long timeBetweenSlots;
volatile bool          validSlotUnderSensor;
unsigned long          lastPinChangeTime ;

ISR(PCINT1_vect)
{
  unsigned long tmp = micros();                  //get the current time in microseconds
  if (tmp - lastPinChangeTime <= NoisePeriod) {  //check not a noisy pulse
    lastPinChangeTime = tmp;              //yes just noise, ignore, save the time so we can compare against the next pulse edge
    return;
  }
  lastPinChangeTime = tmp;                //save the time, so can check the next pulse
  /* Note line below has to be changed if the SensorDiskPin value is changed */
  if (PINC &= B00000100) {                //read sensor directly rather than use DigitalRead(A2), its high, so it is it a LOW to HIGH pin change
    timeSinceStartSlot = tmp;             //yes, save time of low to high change, ie the start of a slot
    validSlotUnderSensor = false;         //clear flag to say we have just had a complete valid slot under sensor
  }
  else {                                  //no, its low, so it is a HIGH to LOW pin change, ie the end of a slot
    if (tmp - timeSinceStartSlot > slotDebounceTime) {        //check if time pulse was high was greater than debounce period, if not ignore
      validSlotUnderSensor = true;                            //yes, we've just had a valid slot pulse
      timeBetweenSlots = tmp - timeSinceStartPreviousSlot;    //work out time bewteen slots
      timeSinceStartPreviousSlot = timeSinceStartSlot;        //save time for next calculation
    }
  }
}                                         //end of ISR(INT0_vect)

double setpoint = 0, input = 0, output = 0;                 //These are variables accessed by the PID Compute() function and code in the main loop, assume all stopped at startup

/* define PID Loops, Input is our PV, Output is our u(t), Setpoint is our SP */
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd);          //constructor also call set tunnings

/** Setup */
void setup(void)
{
  /* set up inputs using internal pull up resistors and set up outputs */
  pinMode(LedPin, OUTPUT);
  digitalWrite(LedPin, HIGH);     // sets the LED on

  Serial.begin(9600);             //set up serial port for any debug prints

  /* Set up PWM
    Timer 0, is 8 bits used by fuction millis();
    Timer 2, is 8 bits and is used to process pwm pulses to motors
    Timer is 16x10^6 (clock speed) / [prescaler x 255];  for prescaler of 32
    Use fast PWM mode, where the timer repeatedly counts from 0 to 255. The output turns on when the timer is at 0, and turns off when the timer matches the output compare register OCR2A and OCR2B.
    The higher the value in the output compare register, the higher the duty cycle.

    // ----- TIMER 2 -----
    // TCCR2A - Timer/Counter control register A
    // Bit  |    7     |    6     |    5     |    4     |  3  |  2  |    1    |    0    |
    //      |  COM2A1  |  COM2A0  |  COM2B1  |  COM2B0  |  -  |  -  |  WGM21  |  WGM20  |

    // TCCR2B - Timer/Counter control register B
    // Bit  |    7     |    6     |    5     |    4     |    3    |    2    |    1   |    0   |
    //      |  FOC2A   |  FOC2B   |    -     |    -     |  WGM22  |  CS22   |  CS21  |  CS20  |

    On TCCR2A, setting the COM2A bits and COM2B bits to 10 provides non-inverted PWM for outputs A and B,
    Clears OC2A & OC2B on Compare Match, set OC2A & OC2B at BOTTOM,
    setting the waveform generation mode bits WGM to 011 selects fast PWM
  */
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20);
  /* On TCCR2B, setting the CS bits to 011 sets the prescaler to divide the clock by 32 */
  TCCR2B  = _BV(CS21) | _BV(CS20);

  /* Set up digital input interrupts for the slotted disk input
    uses digital pin 2, this is ATmega PD2 PCINT18/INT0
    use as INT0, used this rather than one of PCINT0..23 interrupts as easier to set up
  */

  pinMode(SensorDiskPin, INPUT);          //set pin as input, did not enable pullup resistor as not sure how much current sensor can sink
  PCICR  |= (1 << PCIE1);                 //"Pin Change Interrupt Control Register", pin change interrupts enabled on PCINT[14:8]
  PCMSK1 |= (1 << PCINT10);               //"Pin Change Mask Register", enable pin change interrupt on PCINT10

  //  pinMode(SensorDiskPin, INPUT);      //set pin as input, did not enable pullup resistor as not sure how much current sensor can sink
  //  EICRA  |= (1 << ISC00);             //External Interrupt Control Register A, set to trigger on any logic change
  //  EIMSK  |= (1 << INT0);              //External Interrupt Mask Register, enable interrupts on INT0
  sei();                                  //enable interrupts

  /* set up PID */
  myPID.SetSampleTime(PIDSampleTime);         //set how often run PID
  myPID.SetOutputLimits(0, (double)MaxPower); //set range of PID output

#ifdef HardwareTest
  myHardwareTests.displayHelpMsg();       //on startup print command help message if hardware test enabled
#endif

  ledFlashCounter = millis();             //set up counter to flash led every second
  return;
}  //  end of setup()

/*** Main Loop ***/

void loop(void) {
  mainLoopCounter++;                          //count number of times around the main loop, used to give indication of CPU utilisation 
  if (millis() - ledFlashCounter >= 1000 ) {
    toggleLed();                              //yes, flash the led, to show something is happening
    ledFlashCounter = millis();               //reset counter
  }

  /* check if a valid slot has been under the sensor, if so get time between slots in microseconds
    the slot time is used by hardware diagnostics or normal code
  */
  unsigned long slotTime = 0ul;                 //used to store the timing data from the ISR
  cli();                                      //disable interrupts as about to read variables accessed by ISR
  if (validSlotUnderSensor) {                 //get state of speed sensor that is set by the ISR
    slotTime = timeBetweenSlots;              //just had a valid slot under the sensor, get the time between slots calculated by the ISR
    validSlotUnderSensor = false;             //clear flag, this is set by the ISR
  }
  sei();                                      //interrupts back on

  //run hardware tests or normal main loop code, if HardwareTest defined, run the HardwareTest code, otherwise run normal main loop code
#ifdef HardwareTest
  while (Serial.available () > 0)                                         //run hardware test, this code is normally only used for setup of H/W, and normally not compiled
    myHardwareTests.processIncomingByte (Serial.read());                  //read typed input from serial monitor

  //check if valid command has been entered
  if (myHardwareTests.motorCommandEntered()) {                            //check if motor command
    left_Motor.updatePower(myHardwareTests.getLeftTestSpeed());           //yes, update motor power from the entered speed
    left_Motor.updateDir(!myHardwareTests.getLeftTestDir());              //update motor dir from the entered direction, complemented as motors orinetated by 180 degrees
    right_Motor.updatePower(myHardwareTests.getRightTestSpeed());         //update motor power from the entered speed
    right_Motor.updateDir(myHardwareTests.getRightTestDir());             //update motor dir from the entered direction
    myHardwareTests.clearMotorCommandEnteredFlag();                       //clear flag ready for the next command
    Serial.print("Leftspeed:"); Serial.print(myHardwareTests.getLeftTestSpeed()); Serial.print(" Rightspeed:"); Serial.print(myHardwareTests.getRightTestSpeed());
    Serial.print(" Leftdir:");  Serial.print(myHardwareTests.getLeftTestDir());   Serial.print(" Rightdir:");   Serial.println(myHardwareTests.getRightTestDir());
  }

  if (myHardwareTests.getJoystickDisplayTime())                                             //if non zero time, display both joystick axis
    if ((millis() - joys_Time_Of_Last_Scan) > myHardwareTests.getJoystickDisplayTime()) {   //check if enough time elaspsed since last time to display joystick x and y axis
      joys_Time_Of_Last_Scan = millis();                                                    //yes, save time joystick last displayed
      Serial.print("Joystick X:"); Serial.print(js.get_X_Axis()); Serial.print(" Y:"); Serial.println(js.get_Y_Axis());
    }

  if (myHardwareTests.getSensorDisplayFlag()) {                           //check if to display sensor
    if (slotTime)  {                                                      //yes, check if ISR has detected a valid slot
      Serial.print("Time:");  Serial.println(slotTime / 1000);            //yes, display time calculated by the ISR, convert from micro to millseconds
      Serial.print("Speed:"); Serial.println(mySlottedDisk.calculateSpeed(slotTime));   //display speed
    }
  }//end hardware tests

#else
  //run normal program, calculates speed, scans joystick and runs pid then updates motors
  if (slotTime)                                                           //check if ISR has detected a valid slot
    goKartSpeed = mySlottedDisk.calculateSpeed(slotTime);                 //yes, calculate wheel speed in mm/sec
  else if (millis() - mySlottedDisk.getTimeSinceLastSpeedCalculation() > noSlotForTime) //no, check time we last had a slot detected, by checking time between speed calculations
    goKartSpeed = mySlottedDisk.calculateSpeed(0);                        //too long between slots detected assume GoKart stopped, use calculateSpeed() so times stored in slotted disk are updated

  if ((millis() - joys_Time_Of_Last_Scan) >= JoyStickScanRate) {            //check if time to scan joystick for changes to x and y axis
    joys_Time_Of_Last_Scan = millis();                                      //yes, save time joystick last scanned, calculate the speed and then check x and Y axis for changes

    /*check if x axis or y axis of joystick has changed
      if one has changed, as y determines speed forwards or backwards, always process y before x
      use the y speed to firstly set the speed and direction for either forwards or backwards,
      then use x to increase or decrease the speed of the left or right wheels to enable turning;
      to turn, slow down the wheel of the direction you want to turn and leave the other wheel unchanged
    */
    if (js.check_X_Axis() || js.check_Y_Axis()) {     //check if x axis or y axis of joystick has changed
      js.process_Y(&yReqSpeed, &yDir);                //yes, get requested speed and direction for y axis
      if (yDir == BACK)                               //check if direction is BACK or reverse
        yReqSpeed = yReqSpeed / ReverseSpeedSlower;   //yes, slow down reverse speed as a safety measure
      js.process_X(&xTurningDegrees, &xDir);          //get position of x axis?
      MAIN_LOOP_DEBUG_PRINT(__FUNCTION__, " xTurningDegrees:", xTurningDegrees, " xDir:", xDir, " yReqSpeed:", yReqSpeed, " yDir:", yDir); //if MAIN_LOOP_DEBUG defined print to serial monitor
    }
  }

  /* run PID
     PID function has the addresses of Input, Setput, & Output variables and these updated in Compute()
     if PID did a computation, get the output and calculate left and right motor power, and update PWM outputs to the motor
     to prevent possible damage to motors, before update direction check if there is a change of direction and only allow if motors at a low power level
  */
  input     = (double)goKartSpeed;                                    //get input ready for pid
  setpoint  = (double)yReqSpeed;                                      //get setpoint ready for PID

  if (myPID.Compute()) {                                              //check if PID loop did a computation, ie in automatic and internal timer has expired
    leftPower = rightPower = (uint8_t) output;                        //yes, update power from PID to each wheel taking account of turn request from the x axis
    if (xTurningDegrees != 0 && xDir == LEFT)                         //is it a request to turn and the request is to the left
      leftPower = (uint8_t)map(xTurningDegrees, 90, 0, 0, output);    //yes, slow left wheel, by reducing the power by the amount of turning degrees, & leave right wheel unchanged
    else if (xTurningDegrees != 0 && xDir == RIGHT)                   //no, check if a request to move right
      rightPower = (uint8_t)map(xTurningDegrees, 90, 0, 0, output);   //yes, slow right wheel speed, by the amount of turning degrees, & leave left wheel unchanged
    PID_DEBUG_PRINT("     input:", input, " setpoint:", setpoint, " leftPower:", leftPower, " rightPower:", rightPower, " "); //if  PID_DEBUG_PRINT defined print to serial monitor

    left_Motor.updatePower(leftPower);                  //update motor power from the PID output after taking into account turning
    right_Motor.updatePower(rightPower);                //update motor speed from the requested speed after taking into account turning
    if (checkOkToChangeDir(yDir, (uint8_t)output)) {    //check ok to change direction, so only cchange direction at low power
      right_Motor.updateDir(yDir);                      //update motor direction from y axis as determines forwards or back
      left_Motor.updateDir(yDir);                       //update motor direction from y axis as determines forwards or back, complimented as motors orinentated by 180 degrees
      currentDir = yDir;                                //save new direction
    }
  }                                                     //end of normal code in main loop
#endif


}                                                       //end of main loop


void toggleLed(void) {
  led ? led = LOW : led = HIGH;             //swap led state from high to low or low to high
  digitalWrite(LedPin, led);                //update the led
}                                           //end of toggleLed(void)


//checkOkToChangeDir - return false if changing direction and we are above specified power level to the motors
//to prevent damage to the motors

bool checkOkToChangeDir(bool newDir, uint8_t myPower) {
  if (currentDir != newDir && myPower > LowPower )
    return false;                            //not changing direction
  else
    return true;
}


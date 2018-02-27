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
      "left_Motor.update_Dir()" to update motor direction from the requested direction
      "right_Motor.update_Speed()" to update motor speed from the requested speed
      "right_Motor.update_Dir()" to update motor direction from the requested directio
*/

#include "GoKartMkIV.h"

#include "joystick.h"
#include "motor.h"
#include "slotteddisk.h"
#include <PID_v1.h>

unsigned int interrupt_Counter = 0;           //used in main loop to show the ISR is running

unsigned long  joys_Time_Of_Last_Scan = 0;    //track when we last scanned for joystick changes

uint8_t led = LOW;                            //state of led, initially off

//define output comparision registers for PWM, Atmega register used to set the duty cycle of the PWM, write 0 to 255
//if 2 lines below included in goKartMkIV.h generates a compile error
const uint8_t* left_Pwm_Reg    = 0xB3;       // this is OCR2A, for PWM output PD3 OC2A, UNO pin 11
const uint8_t* right_Pwm_Reg   = 0xB4;       // this is OCR2B, for PWM output PD3 OC2B, UNO pin 3

/* define objects */

/* define joystick */
JoyStick js;  //define joystick

/* define motors */
Motor left_Motor(left_Pwm_Reg, left_Dir_Pin, left_Pwm_Pin);
Motor right_Motor(right_Pwm_Reg, right_Dir_Pin, right_Pwm_Pin);

/* Interrupt Service Routine for when counter overflows in timer 2 */

ISR(TIMER2_OVF_vect)
{
  ISR_DEBUG_ENTRY                                   //Diagnostics
  interrupt_Counter = interrupt_Counter + 1;        //used to show we are alive and ISR running
  ISR_DEBUG_EXIT                                    //Diagnostics
  return;
}

/** Setup */
void setup(void)
{
  /* set up inputs using internal pull up resistors and set up outputs */
  pinMode(LedPin,          OUTPUT);
  digitalWrite(LedPin,       HIGH);     // sets the LED on

  Serial.begin(9600);                   //set up serial port for any debug prints

  DEBUG_PRINTLN("Started");

  /* Set up timer interrupt */

  /* Timer 0, is 8 bits used by fuction millis();
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

  sei();                                      //enable interrupts  */

  return;
}  //  end of setup()

/** Main Loop

*/
void loop(void)
{
  if (interrupt_Counter >= One_Sec )          //check if a second has expired
  {
    toggle_Led();                             //yes, flash the led
    reset_Counter();                          //reset counter
  }
  /* check for any joystick movement and update motor speeds */

  if ((millis() - joys_Time_Of_Last_Scan) > JoyStick_Scan_Rate) //check if time to scan joystick for changes to x and Y axis
  {
    joys_Time_Of_Last_Scan = millis();          //yes, reset timer, check x and Y axis for changes
    if (js.check_X_Axis() || js.check_Y_Axis()) //check if x axis of joystick has changed and check if y axis of joystick has changed
    {
      /* yes one has changed, so as y changes speed forwards or backwards, always process y before x
        use the y speed to firstly set the speed and direction for either forwards or backwards,
        then use x to increase or decrease the speed of the left or right wheels to enable turning;
        to turn slow down the wheel of the direction you want to turn and leave the other wheel unchanged */

      int x_Speed;            //local variable to store new speed for x axis
      int x_Dir;              //local variable to store new direction
      int y_Speed;            //local variable to store new speed for y axis
      int y_Dir;              //local variable to store direction
      int left_Speed;         //local variable to store speed of left wheel
      int right_Speed;        //local varialbe to store speed of right wheel

      js.process_Y(&y_Speed, &y_Dir);           //get speed and direction for y axis
      if (y_Dir == REVERSE)                     //chek if direction is reverse
        y_Speed = y_Speed / ReverseSpeedSlower; //yes, slow down reverse speed as a safety measure
        
      left_Speed     = y_Speed;                 //initialise speeds, set left and righ to same, ie moving straight, not turning
      right_Speed    = y_Speed;

      js.process_X(&x_Speed, &x_Dir);            //get position of x axis?
      if (x_Speed != 0 && x_Dir == LEFT)         //is it a request to turn and the request is to the left?
      {
        int x_Speed_Reduction = map(x_Speed, JoystickMin, JoystickMax, 0, y_Speed);     //yes, scale the x reduction in speed as a percentage of the current y speed 

        left_Speed -= x_Speed_Reduction;        //slow left wheel speed, by subtracting the scaled y speed, & leave right wheel unchanged
      }
      else if (x_Speed != 0 && x_Dir == RIGHT) //no, must be request to move right, slow right wheel & leave left wheel unchanged
      {
        int x_Speed_Reduction = map(x_Speed, JoystickMin, JoystickMax,  0, y_Speed);  //scale the x reduction in speed as a percentage of the current y speed
        right_Speed -= x_Speed_Reduction;                                      //slow right wheel speed, by subtracting the scaled y speed & leave left wheel unchanged
      }
      left_Motor.set_Requested_Speed(left_Speed);   //set new speed
      left_Motor.set_Requested_Dir(y_Dir);          //set new direction, have to use Y direction as it determines forwards or backwards
      right_Motor.set_Requested_Speed(right_Speed); //set new speed
      right_Motor.set_Requested_Dir(y_Dir);         //set new direction, have to use Y direction as it determines forwards or backwards

      MAIN_LOOP_DEBUG_PRINT(__FUNCTION__);
      MAIN_LOOP_DEBUG_PRINT(" x_Speed_Reduction: ");
      MAIN_LOOP_DEBUG_PRINT(x_Speed_Reduction);
      MAIN_LOOP_DEBUG_PRINT(" left_Speed: ");
      MAIN_LOOP_DEBUG_PRINT(left_Speed);
      MAIN_LOOP_DEBUG_PRINT(" right_Speed: ");
      MAIN_LOOP_DEBUG_PRINTLN(right_Speed);

      /* now have speed and direction update the motors */
      left_Motor.update_Speed();              //update motor speed from the requested speed
      left_Motor.update_Dir();                //update motor direction from the requested direction
      right_Motor.update_Speed();             //update motor speed from the requested speed
      right_Motor.update_Dir();               //update  motor direction from the requested direction
    }
  }
}
//end of loop()
//-------------------------------------

void toggle_Led(void)
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


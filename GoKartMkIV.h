/* Yacht parameters */

#ifndef GoKart_h
#define GoKart_h

#include "arduino.h"

/* define to run hardware tests, alows manually to enter speeds ,monitor joystick and sensor
 * does not run normal code
 * comment out to run normal code
 */

//if HardwareTest defined, compile the HardwareTest code, otherwise ignore and compile normal code
//#define HardwareTest

/* define to run joystick code debugging which which read the joystick and print to the serial monitor
  comment out before code is released
*/
//#define  JOYSTICK_DEBUG

#ifdef   JOYSTICK_DEBUG
#define  JOYSTICK_DEBUG_PRINT(x, y, z, a, b)    Serial.print(x); Serial.print(y); Serial.print(z); Serial.print(a); Serial.println(b)
#else
#define  JOYSTICK_DEBUG_PRINT(x, y, z, a, b)
#endif

/* define to run joystick code debugging for process joystick Y axis and print to the serial monitor
  comment out before code is released
*/
//#define  JOYSTICK_PROCX_DEBUG

#ifdef   JOYSTICK_PROCX_DEBUG
#define  JOYSTICK_PROCX_DEBUG_PRINT(x, y, z, a, b, c)    Serial.print(x); Serial.print(y); Serial.print(z); Serial.print(a); Serial.print(b); Serial.println(c)
#else
#define  JOYSTICK_PROCX_DEBUG_PRINT(x, y, z, a, b, c)
#endif

/* define to run joystick code debugging which process joystick Y axis and print to the serial monitor
  comment out before code is released
*/
//#define  JOYSTICK_PROCY_DEBUG

#ifdef   JOYSTICK_PROCY_DEBUG
#define  JOYSTICK_PROCY_DEBUG_PRINT(x, y, z, a, b, c)    Serial.print(x); Serial.print(y); Serial.print(z); Serial.print(a); Serial.print(b); Serial.println(c)
#else
#define  JOYSTICK_PROCY_DEBUG_PRINT(x, y, z, a, b, c)
#endif

/* define to run joystick code debugging with a slow scan rate
 * this reduces amout of output to the serial monitor 
  comment out before code is released
*/
//#define JOYSTICK_DEBUG_SCAN

/* define to run joystick code debugging which force a value from the joystick
  comment out before code is released
*/
//#define  JOYSTICK_FORCEDEBUG

#ifdef   JOYSTICK_FORCEDEBUG
#define  JOYSTICK_DEBUG_Y_EQUALS_256     y_New = 256;
#define  JOYSTICK_DEBUG_X_EQUALS_256     x_New = 256;
#else
#define  JOYSTICK_DEBUG_Y_EQUALS_256
#define  JOYSTICK_DEBUG_X_EQUALS_256
#endif

/* define to run motor code debugging which print to the serial monitor
  comment out before code is released
*/
//#define  MOTOR_DEBUG
#ifdef   MOTOR_DEBUG
#define  MOTOR_DEBUG_PRINT(x, y, z)    Serial.print(x);  Serial.print(y);  Serial.println(z)
#else
#define  MOTOR_DEBUG_PRINT(x, y, z)
#endif

/* define to run sensor code debugging which print to the serial monitor
  comment out before code is released
*/
//#define  SENSOR_DEBUG
#ifdef   SENSOR_DEBUG
#define  SENSOR_DEBUG_PRINT(x, y, z, a, b)    Serial.print(x); Serial.print(y); Serial.print(z); Serial.print(a); Serial.println(b)
#else
#define  SENSOR_DEBUG_PRINT(x, y, z, a, b)
#endif

/* define to run PID code debugging which print to the serial monitor
  comment out before code is released
*/
//#define PID_DEBUG
#ifdef   PID_DEBUG
#define  PID_DEBUG_PRINT(x, y, z, a, b, c, d, e, f) Serial.print(x); Serial.print(y); Serial.print(z); Serial.print(a); Serial.print(b); Serial.print(c); Serial.print(d); Serial.print(e); Serial.println(f)
#else
#define  PID_DEBUG_PRINT(x, y, z, a, b, c, d, e, f)
#endif

//#define PID_DEBUG1
#ifdef   PID_DEBUG1
#define  PID_DEBUG_PRINT1(x, y, z, a, b, c, d, e) Serial.print(x); Serial.print(y); Serial.print(z); Serial.print(a); Serial.print(b); Serial.print(c); Serial.print(d); Serial.print(e)
#else
#define  PID_DEBUG_PRINT1(x, y, z, a, b, c, d, e)
#endif

/* define to run main loop code debugging which print to the serial monitor
  comment out before code is released
*/
//#define  MAIN_LOOP_DEBUG

#ifdef   MAIN_LOOP_DEBUG
#define  MAIN_LOOP_DEBUG_PRINT(x, y, z, a, b, c, d, e, f)    Serial.print(x); Serial.print(y); Serial.print(z); Serial.print(a); Serial.print(b); Serial.print(c); Serial.print(d); Serial.print(e); Serial.println(f)
#else
#define  MAIN_LOOP_DEBUG_PRINT(x, y, z, a, b, c, d, e, f)
#endif

/* system parameters
 * use millimeters per second for speed rather km/hr as more resolution for type int eg 
 * km/hr    mm/sec
 *  2.5       694
 *  5        1389
 *  7.5      2083
 * 10.0      2778
 * 12.5      3472
 * 15        4167
 * 17.5      4861
 */
const unsigned int MaxSpeedKmh      =  5;                                       //maximum speed in km/hr
const unsigned int MaxSpeedmmPerSec = MaxSpeedKmh * 1000000 / 3600;             //maximum speed in mm/sec
const uint8_t      MaxPower         = 255;                                      //255 gives 100% duty cycle to the PWM, ie max power
const uint8_t      LowPower         = MaxPower * .1;                            //Power limit below which we can change irection
const int          ReverseSpeedSlower = 5;                                      //Slow reverse speed compared to forward speed as a safety measure

/* set up directions for motors */
#define FORWARD     true
#define BACK        false
#define RIGHT       true
#define LEFT        false

/* joystick Parameters */

/* Regard joystick in stopped position if equal to or between Joys_Stopped_High & Joys_Stopped_Low
     Note difference between Stopped_High and Stopped_low has to be greater than JoyStick_Max_ROC
     as desirable that max change on each joystick scan less than stopped range on joystick,
     so as the joystick values change there will always be one position of the joystick where the motor is stopped
     so the change of direction will occur when the motor is stopped
*/

const int JoystickMin = 0;
const int JoystickMax = 511;                      //range for a perfect joystick,
/* set up stopped range for the joystick */
const int Stopped_High = 538;                     //setjoystick high range for stopped
const int Stopped_Low  = 485;                     //setjoystick low range for stopped

// As joystick ADC reads 0 to 1023, the joystick range is:
//    | 0-484 | 485 - 538 | 539 - 1023 |
//    |  Low  |  Stopped  |    High    |
//range  485      53           485
//
//                  Joystick Operation
//         (orientation with cables at the bottom)
//
//                     Forward (dir 1)
//                        0
//
//   (Dir 0) Left <-     JoyStick   -> Right(dir 1)
//      1023                               0
//
//                     Back   (dir 0)
//                        1023
/*
   Combination of scan rate and maximum Rate of Change (ROC) limit speed of response of system
   as scan rate is 50 millseconds, that is 20 scans in 1 second
   therefore max change in one second is 48 X 20 = 960
   therefore time to maximum speed is (0 to Stopped Low)/960 = 480/960 = .5 second
   NOTE if you change stopped range of joystick, these numbers need to be adjusted
*/

#ifndef   JOYSTICK_DEBUG_SCAN
/* note this is the normal version, change this if you want to change the speed of response of the system */
const unsigned long JoyStickScanRate   = 50;   //scan every 50 ms or 1/20 of a second, (see comments above), normal scan rate
const int           JoyStick_Max_ROC   = 48;   //limit rate of change allowable by the joystick (see comments above)
#else
/* note this is the debug version, uses a slower scan rate to limit the amount of debug data displyed on the serial monitor window*/
const unsigned long JoyStickScanRate   = 500;   //scan every 500 ms or 1/2 of a second, (see comments above), slower for debugging
const int           JoyStick_Max_ROC   = 48;    //limit rate of change allowable by the joystick (see comments above)
#endif

const int  Noise_Mask                   = 0xFFF0; //clear bottom bits to mask any noise on signal

/* ADC I/O for Joystick*/
const uint8_t Xaxis_JoystickAnalogPin   = 0;      //x axis of joystick
const uint8_t Yaxis_JoystickAnalogPin   = 1;      //y xis of joystick

/** motors
   define i/o for each motor driver board, each board has 2 inputs: direction & pwm
*/

const uint8_t  leftDirPin	  = 2;                //wired to the motor driver board to set the direction the motor turns
const uint8_t  leftPwmPin	  = 3;                //PWM pulse to set the speed of the motor, this is ATmega PB3 OC2A, UNO pin 11
uint8_t* const leftPwmReg   = (uint8_t *)0xB4;  // this is OCR2B, for PWM output PD3 OC2B, UNO pin 3

const uint8_t  rightDirPin  = 10;               //wired to the motor driver board to set the direction the motor turns
const uint8_t  rightPwmPin	= 11;               //PWM pulse to set the speed of the motor, this is ATmega PD3 OC2B, UNO pin 3
uint8_t* const rightPwmReg  = (uint8_t *)0xB3;  // this is OCR2A, for PWM output PD3 OC2A, UNO pin 11

/* define i/O for led */
const uint8_t LedPin        =  13;              //LED connected to digital pin 13

/* define i/O for slotted wheel
  used to determine speed of goKart
  Slotted disk sensor connected to analogue input pin 2, this is ATmega PC2 ADC2/PCINT10
  *** NOTE *** 
  ISR has to be changed if the SensorDsikPin is changed
*/
#define SensorDiskPin  A2
const unsigned long noSlotForTime = 2000;       //if no valid slots for this period, assume GoKart is stopped         

#endif

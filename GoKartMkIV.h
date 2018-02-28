/* Yacht parameters */

#ifndef GoKart_h
#define GoKart_h

#include "arduino.h"

/* define to run joystick diagnostics which which read the joystick and print to the serial monitor
  comment out before code is released
*/
//#define  JOYSTICK_DEBUG

#ifdef   JOYSTICK_DEBUG
#define  JOYSTICK_DEBUG_PRINT(x, y, z, a, b)    Serial.print(x); Serial.print(y); Serial.print(z); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d)
#else
#define  JOYSTICK_DEBUG_PRINT(x, y, z, a, b, c, d)
#endif

/* define to run joystick diagnostics for process joystick Y axis and print to the serial monitor
  comment out before code is released
*/
//#define  JOYSTICK_PROCX_DEBUG

#ifdef   JOYSTICK_PROCX_DEBUG
#define  JOYSTICK_PROCX_DEBUG_PRINT(x, y, z, a, b, c)    Serial.print(x); Serial.print(y); Serial.print(z); Serial.print(a); Serial.print(b); Serial.println(c)

#else
#define  JOYSTICK_PROCX_DEBUG_PRINT(x, y, z, a, b, c)
#endif

/* define to run joystick diagnostics which process joystick Y axis and print to the serial monitor
  comment out before code is released
*/
//#define  JOYSTICK_PROCY_DEBUG

#ifdef   JOYSTICK_PROCY_DEBUG
#define  JOYSTICK_PROCY_DEBUG_PRINT(x, y, z, a, b, c)    Serial.print(x); Serial.print(y); Serial.print(z); Serial.print(a); Serial.print(b); Serial.println(c)
#else
#define  JOYSTICK_PROCY_DEBUG_PRINT(x, y, z, a, b, c)
#endif

/* define to run joystick diagnostics with a slow scan rate
  comment out before code is released
*/
//#define JOYSTICK_DEBUG_SCAN

/* define to run joystick diagnostics which force a value from the joystick
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

/* define to run motor diagnostics which print to the serial monitor
  comment out before code is released
*/
//#define  MOTOR_DEBUG
#ifdef   MOTOR_DEBUG
#define  MOTOR_DEBUG_PRINT(x, y, z,)    Serial.print(x);  Serial.print(y);  Serial.println(z)
#else
#define  MOTOR_DEBUG_PRINT(x, y, z)
#endif

/* define to run sensor diagnostics which print to the serial monitor
  comment out before code is released
*/
//#define  SENSOR_DEBUG

#ifdef   SENSOR_DEBUG
#define  SENSOR_DEBUG_PRINT(x, y, z, a, b)    Serial.print(x); Serial.print(y); Serial.print(z); Serial.print(a); Serial.print(b); Serial.print(c); Serial.println(d)
#else
#define  SENSOR_DEBUG_PRINT(x, y, z, a, b)
#endif

/* define to run main loop diagnostics which print to the serial monitor
  comment out before code is released
*/
//#define  MAIN_LOOP_DEBUG

#ifdef   MAIN_LOOP_DEBUG
#define  MAIN_LOOP_DEBUG_PRINT(x, y, z, a, b, c, d, e, f)    Serial.print(x); Serial.print(y); Serial.print(z); Serial.print(a); Serial.print(b); Serial.print(c); Serial.print(d); Serial.print(e); Serial.println(f)
#else
#define  MAIN_LOOP_DEBUG_PRINT(x, y, z, a, b, c, d, e, f)
#endif

//system parameters
/* Note the value of ValidTimeBetweenSlots is only valid for wheels 200mm diameter and larger, and speeds 20km/hr or slower */
const unsigned int MaxSpeedKmh      =  15;                                      //maximum speed in km/hr. If this is changed check value of ValidTimeBetweenSlots!!!
const unsigned int MaxSpeedmmPerSec = MaxSpeedKmh * 1000 * 1000 / 3600;         //maximum speed in mm/sec for 15km/hour 4,167mm/sec
const uint8_t      MaxPower         = 255;                                      //255 gives 100% duty cycle to the PWM, ie max power
const int          ReverseSpeedSlower = 2;                                      //Slow reverse speed compared to forward speed

/* set up directions for motors */
#define FORWARD     true
#define BACK        false
#define RIGHT       true
#define LEFT        false

const unsigned int One_Sec = 1960;            //used in main loop to show the ISR is running, flip flops led off and on each second
const unsigned int Qtr_Sec =  490;            //used in main loop to flash led show for a quarter of a second

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
const int     Stopped_High = 543;      //setjoystick high range for stopped
const int     Stopped_Low  = 480;      //setjoystick low range for stopped

// As joystick ADC reads 0 to 1023, the joystick range is:
//    | 0-479 | 480 - 543 | 544 - 1023 |
//    |  Low  |  Stopped  |    High    |
//range  480      64           480

//
//                  Joystick Operation
//         (orientation with cables at the bottom)
//
//                     Forward (dir 1)
//                        0
//
//   (Dir 0) Left <-     JoyStick   -> Right(dir 1)
//      0                               1023
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
const unsigned long JoyStickScanRate   = 200;   //scan every 200 ms or 1/5 of a second, (see comments above), slower for debugging
const int  JoyStick_Max_ROC            = 48;    //limit rate of change allowable by the joystick (see comments above)
#endif

const int JoystickToPidSampleTime       = 1;      //set ratio of when joystick is sampled to when PID is computed, should not be less than 1

const int  Noise_Mask                   = 0xFFF0; //clear bottom bits to mask any noise on signal

/* ADC I/O for Joystick*/
const uint8_t Xaxis_JoystickAnalogPin     = 1;    //x axis of joystick
const uint8_t Yaxis_JoystickAnalogPin     = 0;    //y xis of joystick

/** motors
   define i/o for each motor driver board, each board has 2 inputs: direction & pwm
*/
uint8_t* const leftPwmReg    = (uint8_t *)0xB3;       // this is OCR2A, for PWM output PD3 OC2A, UNO pin 11
uint8_t* const rightPwmReg   = (uint8_t *)0xB4;       // this is OCR2B, for PWM output PD3 OC2B, UNO pin 3

const uint8_t  leftDirPin	  = 10;         //wired to the motor driver board to set the direction the motor turns
const uint8_t  leftPwmPin	  = 11;         //PWM pulse to set the speed of the motor, this is ATmega PB3 OC2A, UNO pin 11

const uint8_t  rightDirPin  = 2;          //wired to the motor driver board to set the direction the motor turns
const uint8_t  rightPwmPin	= 3;          //PWM pulse to set the speed of the motor, this is ATmega PD3 OC2B, UNO pin 3

/* define i/O for led */
const uint8_t LedPin          =  13; //LED connected to digital pin 13

/* define i/O for slotted wheel
  used to determine speed of goKart
*/
const uint8_t SensorDiskPin =  5; //Slotted disk sensor connected to digital pin 5

#endif

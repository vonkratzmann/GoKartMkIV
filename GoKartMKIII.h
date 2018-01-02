/* Yacht parameters */

#ifndef GoKart_h
#define GoKart_h

#include "arduino.h"

/* define to run diagnostics which print to the serial monitor
  comment out before code is released
*/
//#define  DEBUG

#ifdef   DEBUG
#define  DEBUG_PRINT(x)    Serial.print(x)
#define  DEBUG_PRINTLN(x)  Serial.println(x)
#define  DEBUG_FILE(x)
#else
#define  DEBUG_PRINT(x)
#define  DEBUG_PRINTLN(x)
#define  DEBUG_FILE(x)
#endif

/* define to run ISR diagnostics which attempt to determine the overhead of the ISR
   and print out te results in the main loop
  comment out before code is released
*/
//#define  ISR_DEBUG

#ifdef   ISR_DEBUG
#define  ISR_DEBUG_ENTRY      entry_Time=micros();
#define  ISR_DEBUG_EXIT       exit_Time=micros();
#define  ISR_DEBUG_PRINT(x)   Serial.print(x)
#define  ISR_DEBUG_PRINTLN(x) Serial.println(x)
#else
#define  ISR_DEBUG_ENTRY
#define  ISR_DEBUG_EXIT
#define  ISR_DEBUG_PRINT(x)
#define  ISR_DEBUG_PRINTLN(x) Serial.println(x)
#endif

/* define to run joystick diagnostics which which read the joystick and print to the serial monitor
  comment out before code is released
*/
//#define  JOYSTICK_DEBUG

#ifdef   JOYSTICK_DEBUG
#define  JOYSTICK_DEBUG_PRINT(x)    Serial.print(x)
#define  JOYSTICK_DEBUG_PRINTLN(x)  Serial.println(x)
#define  JOYSTICK_DEBUG_FILE(x)
#else
#define  JOYSTICK_DEBUG_PRINT(x)
#define  JOYSTICK_DEBUG_PRINTLN(x)
#define  JOYSTICK_DEBUG_FILE(x)
#endif

/* define to run joystick diagnostics for process joystick Y axis and print to the serial monitor
  comment out before code is released
*/
//#define  JOYSTICK_PROCX_DEBUG

#ifdef   JOYSTICK_PROCX_DEBUG
#define  JOYSTICK_PROCX_DEBUG_PRINT(x)    Serial.print(x)
#define  JOYSTICK_PROCX_DEBUG_PRINTLN(x)  Serial.println(x)
#define  JOYSTICK_PROCX_DEBUG_FILE(x)
#else
#define  JOYSTICK_PROCX_DEBUG_PRINT(x)
#define  JOYSTICK_PROCX_DEBUG_PRINTLN(x)
#define  JOYSTICK_PROCX_DEBUG_FILE(x)
#endif

/* define to run joystick diagnostics which process joystick Y axis and print to the serial monitor
  comment out before code is released
*/
//#define  JOYSTICK_PROCY_DEBUG

#ifdef   JOYSTICK_PROCY_DEBUG
#define  JOYSTICK_PROCY_DEBUG_PRINT(x)    Serial.print(x)
#define  JOYSTICK_PROCY_DEBUG_PRINTLN(x)  Serial.println(x)
#define  JOYSTICK_PROCY_DEBUG_FILE(x)
#else
#define  JOYSTICK_PROCY_DEBUG_PRINT(x)
#define  JOYSTICK_PROCY_DEBUG_PRINTLN(x)
#define  JOYSTICK_PROCY_DEBUG_FILE(x)
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
//a#define  MOTOR_DEBUG

#ifdef   MOTOR_DEBUG
#define  MOTOR_DEBUG_PRINT(x)    Serial.print(x)
#define  MOTOR_DEBUG_PRINTLN(x)  Serial.println(x)
#define  MOTOR_DEBUG_FILE(x)
#else
#define  MOTOR_DEBUG_PRINT(x)
#define  MOTOR_DEBUG_PRINTLN(x)
#define  MOTOR_DEBUG_FILE(x)
#endif

/* define to run main loop diagnostics which print to the serial monitor
  comment out before code is released
*/
//#define  MAIN_LOOP_DEBUG

#ifdef   MAIN_LOOP_DEBUG
#define  MAIN_LOOP_DEBUG_PRINT(x)    Serial.print(x)
#define  MAIN_LOOP_DEBUG_PRINTLN(x)  Serial.println(x)
#define  MAIN_LOOP_DEBUG_FILE(x)
#else
#define  MAIN_LOOP_DEBUG_PRINT(x)
#define  MAIN_LOOP_DEBUG_PRINTLN(x)
#define  MAIN_LOOP_DEBUG_FILE(x)
#endif

/* set up directions for motors */
#define FORWARD     0
#define REVERSE     1
#define RIGHT       0
#define LEFT        1

const int One_Sec = 1960;            //used in main loop to show the ISR is running, flip flops led off and on each second
const int Qtr_Sec =  490;            //used in main loop to flash led show for a quarter of a second

/* joystick Parameters */

/* Regard joystick in stopped position if equal to or between Joys_Stopped_High & Joys_Stopped_Low
     Note difference between Stopped_High and Stopped_low has to be greater than JoyStick_Max_ROC
     as desirable that max change on each joystick scan less than stopped range on joystick,
     so as the joystick values change there will always be one position of the joystick where the motor is stopped
     so the change of direction will occur when the motor is stopped
*/

/* set up stopped range for the joystick */
const int     Stopped_High = 543;      //setjoystick high range for stopped
const int     Stopped_Low  = 480;      //setjoystick low range for stopped

// As joystick ADC reads 0 to 1023, the joystick range is:
//    | 0-479 | 480 - 543 | 544 - 1023 |
//    |  Low  |  Stopped  |    High    |
//range  480      64           480
/* Set up speed range for joystick */
const int JOYSTICK_MINSPEED = 0;
const int JOYSTICK_MAXSPEED = 511;                      //set to upper limit of joystick, to try to make the joystick look like it's working range is 0 to 511
const int REVERSE_MAXPSEED = JOYSTICK_MAXSPEED / 2;     //limit reverse speed

/* LIMIT_TURNING is used to limit rate of turning to stop violent turns while going fast,
  so the effective value of variable Speed_Reduction in main loop is made smaller because of increased range
  eg in this case the variable Speed_Reduction is reduced by a factor of 4
  eg const int LIMIT_TURNING = JOYSTICK_MAXSPEED * 4;
*/
const int LIMIT_TURNING = JOYSTICK_MAXSPEED * 1;      //at the moment no limiting of the turning rate
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
//                     Reverse   (dir 0)
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
const unsigned long JoyStick_Scan_Rate   = 50;   //scan every 50 ms or 1/20 of a second, (see comments above), normal scan rate
const int  JoyStick_Max_ROC              = 48;   //limit rate of change allowable by the joystick (see comments above)
const int  JoyStick_Fwd_Max_ROC          = 24;   //special limit rate of change for acceleration when travelling forward
#else
/* note this is the debug version, this is not normally changed */
const unsigned long JoyStick_Scan_Rate   = 200;   //scan every 200 ms or 1/5 of a second, (see comments above), slower for debugging
const int  JoyStick_Max_ROC              = 48;    //limit rate of change allowable by the joystick (see comments above)
const int  JoyStick_Fwd_Max_ROC          = 12;   //special limit rate of change for acceleration when travelling forward
#endif

const int  noise_Mask                    = 0xFFF0; //clear bottom bits to mask any noise on signal

/* ADC I/O for Joystick*/
const uint8_t Xaxis_JoystickAnalogPin     = 1;    //x axis of joystick
const uint8_t Yaxis_JoystickAnalogPin     = 0;    //y xis of joystick

/* Set up speed range for motor, PWM range is 0 t0 255, which is stopped to full speed for the motor. If the upper motor speed is to be restricted,
   then MOTOR_MAXSPEED is set to something below 255 */
const int   MOTOR_MINSPEED = 0;
const int   MOTOR_MAXSPEED = 190;

/* Motor Parameters
   Response time of the system is controlled by the joystick max rate of change value.
   Motor max rate of change is for when the chain hits the stops, if present
   this de-acceleration is not controlled by the joystick
*/

const int  Motor_Max_ROC = JoyStick_Max_ROC;               //limit rate of change allowable by the motors (see comments above) set the same here as no stops

const long Debounce = 100;                    //debounce time for switch in millisecs

/** motors
   define i/o for each motor driver board, each board has 2 inputs: direction & pwm
*/
const uint8_t  left_Dir_Pin	  = 10;         //wired to the motor driver board to set the direction the motor turns
const uint8_t  left_Pwm_Pin	  = 11;         //PWM pulse to set the speed of the motor, this is ATmega PB3 OC2A, UNO pin 11

const uint8_t  right_Dir_Pin     = 2;       //wired to the motor driver board to set the direction the motor turns
const uint8_t  right_Pwm_Pin	   = 3;       //PWM pulse to set the speed of the motor, this is ATmega PD3 OC2B, UNO pin 3

/* define i/O for led */
const uint8_t LedPin =  13; //LED connected to digital pin 13
#endif

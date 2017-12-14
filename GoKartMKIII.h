/* Yacht parameters */

#ifndef Yacht_h
#define Yacht_h

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

/* define to run joystick diagnostics which which print to the serial monitor
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
#define  MOTOR_DEBUG_PRINT(x)    Serial.print(x)
#define  MOTOR_DEBUG_PRINTLN(x)  Serial.println(x)
#define  MOTOR_DEBUG_FILE(x)
#else
#define  MOTOR_DEBUG_PRINT(x)
#define  MOTOR_DEBUG_PRINTLN(x)
#define  MOTOR_DEBUG_FILE(x)
#endif

/* define to run switch diagnostics which print to the serial monitor
  comment out before code is released
*/
//#define  SWITCH_DEBUG

#ifdef   SWITCH_DEBUG
#define  SWITCH_DEBUG_PRINT(x)    Serial.print(x)
#define  SWITCH_DEBUG_PRINTLN(x)  Serial.println(x)
#define  SWITCH_DEBUG_FILE(x)
#else
#define  SWITCH_DEBUG_PRINT(x)
#define  SWITCH_DEBUG_PRINTLN(x)
#define  SWITCH_DEBUG_FILE(x)
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
#define FORWARD     1
#define REVERSE     0
#define TOPORT      1
#define TOSTARBOARD 0
#define LOOSENING   1
#define TIGHTENING  0

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
const int     Stopped_High = 546;      //setjoystick high range for stopped
const int     Stopped_Low  = 475;      //setjoystick low range for stopped
//stopped range is 70

// As joystick ADC reads 0 to 1023, the joystick range is:
// | 0-475 | 476 - 546 | 547 - 1023 |
// |  Low  |  Stopped  |    High    |

/* Set up speed range for joystick */
const int MINSPEED = 0;
const int MAXSPEED = 481;                     

/*
   Combination of scan rate and maximum Rate of Change (ROC) limit speed of response of system
   Restrict 0 to max change to approximately 1 second 
   as scan rate is 50 millseconds, that is 20 scans in 1 second
   therefore max change on each joystick scan is 48, ie 48 X 20 = 480 ( ie 0 to Stopped Low)
   from max speed in one direction to maximum speed in other direction will take 3 seconds
   NOTE if you change stopped range of joystick, these numbers need to be adjusted 
*/

#ifdef   JOYSTICK_DEBUG_SCAN
const unsigned long JoyStick_Scan_Rate    = 200;   //scan every 200 ms or 1/5 of a second, (see comments above), slower for debugging
#else
const unsigned long JoyStick_Scan_Rate    = 50;   //scan every 50 ms or 1/20 of a second, (see comments above), normal scan rate
#endif
const int  JoyStick_Max_ROC              = 48;     //limit rate of change allowable by the joystick (see comments above)
const int  noise_Mask                    = 0xFFF0; //clear bottom bits to mask any noise on signal

/* ADC I/O for Joystick*/
const uint8_t rudder_JoystickAnalogPin     = 1;    //x axis of joystick
const uint8_t boom_JoystickAnalogPin      = 0;    //y xis of joystick

/* Set up speed range for motor, PWM range is 0 t0 255, which is stopped to full speed for the motor. If the upper motor speed is to be restricted,
   then MOTOR_MAXSPEED is set to something below 255 */
const int   MOTOR_MINSPEED = 0;
const int   RUDDER_MOTOR_MAXSPEED = 90;              
const int   BOOM_MOTOR_MAXSPEED = 127;
const long  MOTOR_MOVING_TIME = 1500;          //Time in milliseconds a motor has to be moving in one direction before can clear inhibit movement flag


/* Motor Parameters
   Response time of the system is controlled by the joystick max rate of change value.
   Motor max rate of change is for when the chain hits the stops,
   this de-acceleration is not controlled by the joystick
*/

const int  Motor_Max_ROC    = 25;             //limit rate of change allowable by the motors (see comments above)

const long Debounce = 100;                    //debounce time for switch in millisecs

/** motors
   define i/o for each motor driver board, each board has 2 inputs: direction & pwm
*/
const uint8_t  rudder_Dir_Pin	  = 10;          //sets direction rudder motor turns
const uint8_t  rudder_Pwm_Pin	  = 11;         //PWM pulse to set the speed of the rudder motor, this is ATmega PB3 OC2A, UNO pin 11

const uint8_t  boom_Dir_Pin     = 2;          //sets the direction the boom motor turns
const uint8_t  boom_Pwm_Pin	    = 3;          //PWM pulse to set the speed of the boom motor, this is ATmega PD3 OC2B, UNO pin 3

/** end of travel detectors
   define i/O for reed switches to detect end of travel for the chain on each motor
*/
const uint8_t  rudder_Port_EndofTravel_Pin      = 8;
const uint8_t  rudder_Starboard_EndofTravel_Pin	= 9;
const uint8_t  boom_Loose_EndofTravel_Pin       = 6;
const uint8_t  boom_Tight_EndofTravel_Pin       = 7;

/* define i/O for led */
const uint8_t LedPin =  13; //LED connected to digital pin 13
#endif

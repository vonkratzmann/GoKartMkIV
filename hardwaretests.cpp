#include "GoKartMkIV.h"
#include "HardwareTests.h"

//if HardwareTest defined in GoKartMkIV.h, compile the HardwareTest code below, otherwise ignore and compile normal code
#ifdef HardwareTest
/*  Commands  **** NOTE in Serial Monitor ensure "Newline" is selected in the bottom dropdown menu  ****
     Ls sets speed of the left  motor to the value of s which has to be between 0 and 255, if error speed set to zero
     Rs sets speed of the right motor to the value of s which has to be between 0 and 255, if error speed set to zero
     Ld sets direction of the left  motor to the value of d which has to be between f(forwards)or b(backwards), if error direction set to forwards
     Rd sets direction of the right motor to the value of d which has to be between f(forwards)or b(backwards), if error direction set to forwards
     Dz display joystick x and y axis every z milliseconds, if error stop the display
     Ds display the time between slots detected by the sensor 
*/

HardwareTests::HardwareTests() {
  leftWheelSpeed = 0;             //ensure stopped at start
  leftWheelDir = FORWARD;         //initialise direction to forward
  rightWheelSpeed = 0;
  rightWheelDir = FORWARD;
}                                 //end of constructor HardwareTests()

//displayHelpMsg -
void HardwareTests::displayHelpMsg(void) {
  Serial.println("*** Ensure 'Newline' is selected in the bottom dropdown menu ***");
  Serial.println("Ls set speed of left motor,  s = 0 to 255, if error s = 0");
  Serial.println("Rs set speed of right motor, s = 0 to 255, if error s = 0");
  Serial.println("Ld set direction of left motor,  d = f or b, forwards/backwards, if error d = forwards");
  Serial.println("Ld set direction of right motor, d = f or b, forwards/backwards, if error d = forwards");
  Serial.println("Dz display joystick x and y axis every z milliseconds, if error z = 0");
  Serial.println("Ds display the time between slots");
  Serial.println("H display this message");
}                                                                                                 //end of displayHelpMsg()


void HardwareTests::processIncomingByte (const byte inByte) {

  switch (inByte) {
    case '\n':                        //terminator reached, end of text
      inString += (char)0;            //add terminating null byte
      if (inString.length() != 0)     //check string not empty
        processData();                //not empty, so process input_line here
      break;

    case '\r':                        // discard carriage return
      break;

    default:
      inString += (char)inByte;       //add next character
      break;
  }                                   //end of switch
}                                     //end of processIncomingByte()


//processData - process incoming serial data after a terminator received
void HardwareTests::processData() {
  inString.setCharAt(0, inString[0] &= 0xDF); //convert first char to upper case
  switch (inString[0]) {              //process the first character command
    case 'L':                         //left wheel selected
      if (isDigit(inString[1]))       //check if a digit, then it is a speed command
        leftWheelSpeed = HardwareTests::processSpeedCommand();
      else                            //not a digit, assume it is a character and it is a direction command
        leftWheelDir = HardwareTests::processDirectionCommand();
      break;

    case 'R':                         //right wheel selected
      if (isDigit(inString[1]))       //check if a digit, then it is a speed command
        rightWheelSpeed = HardwareTests::processSpeedCommand();
      else                            //not a digit, assume it is a character and it is a direction command
        rightWheelDir = HardwareTests::processDirectionCommand();
      break;

    case 'D':                         //display selected
      if (isDigit(inString[1]))       //check if a digit, then it is a joystick command with time between displays of x and y axis
        HardwareTests::processDisplayJoystickCommand();
      else                            //not a digit, assume it is a character and it is a display sensor state command
        HardwareTests::processDisplaySensorCommand();
      break;

    case 'H':                         //check if to display help message
      HardwareTests::displayHelpMsg();
      break;

    default:                        //ignore
      break;
  }                                 //end of switch
  inString.remove(0);               //empty the string out for the next command
}                                   //end of processData()


//processSpeedCommand - get the speed typed and check the range, if error return zero
uint8_t HardwareTests::processSpeedCommand() {
  if (inString.substring(1).toInt() > (long)MaxPower)     //get the specified speed after the command character, check in the correct range
    return 0;                                             //no, then return zero which will stop the motor
  motorCommandEnteredFlag = true;                         //set flag to say we have a new motor command
  return (uint8_t)inString.substring(1).toInt();
}


//processDirectionCommand - check if forward or backward, if error return forward
bool HardwareTests::processDirectionCommand() {
  inString.setCharAt(1, inString[1] &= 0xDF); //convert direction char to upper case
  motorCommandEnteredFlag = true;    //set flag to say we have a new motor command
  if (inString[1] == 'B')
    return BACK;
  else
    return FORWARD;                 //if anything other 'B' return forward
}                                   //end of processSpeedCommand()


//getLeftTestSpeed -
uint8_t HardwareTests::getLeftTestSpeed() {
  return leftWheelSpeed;
}                                   //end of getLeftTestSpeed()


//getRightTestSpeed -
uint8_t HardwareTests::getRightTestSpeed() {
  return rightWheelSpeed;
}                                   //end of getRightTestSpeed()


//getLeftTestDir -
bool HardwareTests::getLeftTestDir() {
  return leftWheelDir;
}                                   //end of getLeftTestDir()


//getRightTestDir -
bool HardwareTests::getRightTestDir() {
  return rightWheelDir;
}                                   //end of getRightTestDir()


//motorCommandEntered -
bool HardwareTests::motorCommandEntered() {
  return motorCommandEnteredFlag;
}                                   //end of motorCommandEntered()


//clearMotorCommandEnteredFlag -
void HardwareTests::clearMotorCommandEnteredFlag() {
  motorCommandEnteredFlag = false;
}                                   //end of clearMotorCommandEnteredFlag()


//processDisplayJoystickCommand -
void HardwareTests::processDisplayJoystickCommand(void) {
  joystickTime = inString.substring(1).toInt(); //get the specified time in milliseconds after the command character, if error time will be set to zero
}                                               //end of HardwareTests::processDisplayJoystickCommand()


//getJoystickDisplayTime -
unsigned long HardwareTests::getJoystickDisplayTime(void) {
  return joystickTime;
}                                             //end of getJoytsickDisplayTimeg()


//processDisplaySensorCommand -
void HardwareTests::processDisplaySensorCommand(void) {
  inString.setCharAt(1, inString[1] &= 0xDF); //convert direction char to upper case
  if (inString[1] == 'S')
    sensorDisplayFlag = true;                 //if 's' set flag to display sensor value
  else
    sensorDisplayFlag = false;               //anything else do not display sensor value
}                                            //end of processDisplaySensorCommand()


//getSensorDisplayFlag -
bool HardwareTests::getSensorDisplayFlag(void) {
  return sensorDisplayFlag;
}                                           //end of getSensorDisplayFlag()
#endif

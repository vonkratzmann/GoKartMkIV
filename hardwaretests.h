/** Class Diagnostics

*/

//if HardwareTest defined in GoKartMkIV.h, compile the HardwareTest code below, otherwise ignore and compile normal code
#ifdef HardwareTest
#ifndef HardwareTests_h
#define HardwareTests_h

#include "Arduino.h"

class HardwareTests
{
  private:
    String        inString;           //string to hold typed input 
    uint8_t       leftWheelSpeed;     //store speed entered
    bool          leftWheelDir;       //store direction entered
    uint8_t       rightWheelSpeed;    //store speed entered
    bool          rightWheelDir;      //store direction entered
    bool          motorCommandEnteredFlag; //say if we have a new motorCommand
    bool          sensorDisplayFlag;  //say if we have a new motorCommand
    unsigned long joystickTime;       //time between displays of joystick x and y axis on the serial monitor
  public:
    HardwareTests ();
    void          processIncomingByte(const byte);
    void          processData (void);
    uint8_t       processSpeedCommand(void);
    bool          processDirectionCommand(void);
    uint8_t       getLeftTestSpeed(void);
    uint8_t       getRightTestSpeed(void);
    bool          getLeftTestDir(void);
    bool          getRightTestDir(void);
    bool          motorCommandEntered(void);
    void          clearMotorCommandEnteredFlag(void);
    void          processDisplayJoystickCommand(void);
    unsigned long getJoystickDisplayTime(void);
    void          processDisplaySensorCommand(void);
    bool          getSensorDisplayFlag(void);
    void          displayHelpMsg(void);
};
#endif
#endif

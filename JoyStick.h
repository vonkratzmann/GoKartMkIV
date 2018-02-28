/** Class joyStick

*/
#ifndef JoyStick_h
#define JoyStick_h

#include "Arduino.h"

class JoyStick
{
  private:
    unsigned int     x_Cur, y_Cur;            //current position of joystick
    unsigned int     x_New, y_New;            //new position of joystick
    int     diff;
    bool x_Chnged, y_Chnged;         //flag joy stick position has changed

  public:
    JoyStick ();
    bool    check_X_Axis (void);  //check if change in joystick x position
    bool    check_Y_Axis (void);  //check if change in joystick y position
    void    process_X(unsigned int *spd, bool *dir); // process change for x axis of joystick
    void    process_Y(unsigned int *spd, bool *dir); // process change for y axis of joystick
};
#endif

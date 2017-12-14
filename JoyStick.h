/** Class joyStick

*/
#ifndef JoyStick_h
#define JoyStick_h

#include "Arduino.h"

class JoyStick
{
  private:
    int     x_Cur, y_Cur;            //current position of joystick
    int     x_New, y_New;            //new position of joystick
    int     diff;
    bool x_Chnged, y_Chnged;         //flag joy stick position has changed

  public:
    JoyStick ();
    bool    check_X_Axis (void);  //check if change in joystick x position
    bool    check_Y_Axis (void);  //check if change in joystick y position
    void    process_X(int *spd, int *dir); // process change for x axis of joystick
    void    process_Y(int *spd, int *dir); // process change for y axis of joystick
};
#endif

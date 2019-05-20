/*======================================================================
Copyright 2019 Google LLC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    https://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
======================================================================*/

/*
DESCRIPTION:
============
The same PrJoystick data structure is used all the time. It is filled out
by the joyLib routines. A call to joyCalibrate will calibrate the PrJoystick.
It is essential that at that time the PrJoystick is in homeposition and plugged
in.

All subsequent calls to joyRead will return in the data structure (in buttons,
see defines, xPos, and yPos) the buttons pressed and the x and y position
relative to the calibrated position.
*/

#ifndef _PCV_Joystick_h_
#define _PCV_Joystick_h_

/* defines */

#define JOY_BUTTON_NONE 0x00
#define JOY_BUTTON_FIRE 0x01
#define JOY_BUTTON_TOP  0x02
#define JOY_BUTTON_BOTH 0x03


class PCV_Joystick
{
public:

    PCV_Joystick();
    //~PCV_Joystick(){}

    // calibrates the joystick (don't touch the joystick)
    int joystick_calibrate();

    // returns in joyData the current position of the joystick
    //  undefined if joystick has not been calibrated before
    int joystick_hwread ( int& x, int& y, int& buttons );

    int joystick_calibrated() { return ( calib_ ); }

private:
    int calib_;
    int xCalib_;
    int yCalib_;

};
#endif //  _PCV_Joystick_h_

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

#include <Eigen/Dense>

#include "Filter.h"
#include "RAHjoy.h"


RAHjoy::RAHjoy()
{ init( 0 );
}

RAHjoy::RAHjoy(const Float hz)
{ init( hz );
}

void
RAHjoy::init(const Float hz)
{ x_max_=JOY_X_MAX;
  x_min_=JOY_X_MAX;
  y_max_=JOY_Y_MAX;
  y_min_=JOY_Y_MAX;
  x_dead_=JOY_X_DED;
  y_dead_=JOY_Y_DED;
  hz_=hz;
}

RAHjoy::RAHjoy(const Float hz, int xmax, int xmin, int ymax, int ymin)
{ x_max_=xmax;
  x_min_=xmin;
  y_max_=ymax;
  y_min_=ymin;
  x_dead_=JOY_X_DED;
  y_dead_=JOY_Y_DED;
  hz_=hz;
}

void
RAHjoy::norm(float &x, float &y, int &b)
{
  int xi,yi;
  int x_sign=1;
  int y_sign=1;

  joystick_hwread(xi,yi,b);
  x =  xi;
  y = -yi; // invert y coord.

  if(x<0)
  { x=-x;
    x_sign=-1;
  }
  if(y<0)
  { y=-y;
    y_sign=-1;
  }

  x -= x_dead_;
  x /= x_max_-x_dead_;
  if( x>1.0 ) x=1.0;
  if( x<0.0 ) x=0.0;
  x *= x_sign;

  y -= y_dead_;
  y /= y_max_-y_dead_;
  if( y>1.0 ) y=1.0;
  if( y<0.0 ) y=0.0;
  y *= y_sign;

} // func joyNorm


void
RAHjoy::smooth(float &x, float &y, int &b)
{
 if( hz_ ) // ONLY USEABLE IF CONSTRUCTED WITH SERVO RATE
 {
  static int k=0;
  static Filter Fcx;
  static Eigen::Vector2d  cx = Eigen::Vector2d::Zero();
  static Eigen::Vector2d fcx = Eigen::Vector2d::Zero();

  if(k==0)
  { joystick_calibrate();
    Fcx.LowPass(cx, hz_,  5);
  }

  if((++k) > hz_/25 ) // READS AT ~25 Hz
  { k = 1;
    float jx,jy;
    norm(jx,jy,b);
    cx[0] = jx;
    cx[1] = jy;
  }

  fcx = Fcx.Filt( cx );  // DO NOT FILTER BUTTON
  x = fcx[0];
  y = fcx[1];

 } // if(hz_)
 else        // NO KNOWN SERVO RATE SO RET. ZEROs & ERR
 {
   x =  0;
   y =  0;
   b = -1;
 }

}


Vector3d
RAHjoy::joyFill_cx()
{
 if( hz_ ) // ONLY USEABLE IF CONSTRUCTED WITH SERVO RATE
 {
  static int k=-1;
  static Filter Fcx;
  static Eigen::Vector3d cx = Eigen::Vector3d::Zero(3);

  float joy_x,joy_y;
  int joy_b;

  if(k==-1)
  { joystick_calibrate();
    Fcx.LowPass(cx, hz_,  5);
  }

  if((++k)%((int)hz_/50)==0)
  {
    k=0;
    norm(joy_x,joy_y,joy_b);
    if(joy_b == 1)
    { cx[0] = MAX_JOY_V*joy_x;
      cx[1] = MAX_JOY_V*joy_y;
      cx[2] = MAX_JOY_W*0.0  ;
    }
    else if(joy_b == 2)
    { cx[0] = MAX_JOY_V*0.0  ;
      cx[1] = MAX_JOY_V*joy_y;
      cx[2] = MAX_JOY_W*(-joy_x);
    }
    else if(joy_b == 3)
    { cx[0] = MAX_JOY_V*joy_x;
      cx[1] = MAX_JOY_V*joy_y;
      cx[2] = MAX_JOY_W*(-joy_x);
    }
    else
    { cx[0] = 0.0;
      cx[1] = 0.0;
      cx[2] = 0.0;
    }
  }

  cx_f = Fcx.Filt(cx);
 } // if(hz_)
 else cx_f.setZero(3);  // NO KNOWN SERVO RATE SO RET. ZEROs

 return cx_f;
}

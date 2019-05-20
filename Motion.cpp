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


#include <iostream>
#include <cmath>
#include <cstring>

#include <Eigen/Dense>
#include "PCV_Types.h"

#include "RAHjoy.h"

#include "Traj3.h"
#include "traj_circle.h"

#include "Motion.h"
#include "PCV_State.h"

using namespace std;
using namespace Eigen;

static Vector3d destPos = Vector3d::Zero();
static Vector3d cxd;    = Vector3d::Zero();
static RAHjoy  *joy;

static int idx_sqar=0;


// SINGLETON
static MotionList  sList; //CREATE STATIC
static MotionIter  sIter; //CREATE STATIC
static MotionIter  sNext; //CREATE STATIC

MotionList &
Motion::ListGet()
{
  if( sList.IsEmpty() ) // ADD NULL MOTION IF EMPTY
  { sList.BackPush( new Motion(TRAJ_M_NULL) );
    sIter = sList.Begin();    // POINT TO BEGINNING
    joy = new RAHjoy((int)hw_freq); // INITIALIZE JOYSTICK
  }
  return sList;
}

// SINGLETON
MotionIter &
Motion::IterGet()
{
  if( !sIter.NotDone() ) // POINT TO BEGIN IF NOT SET
  { if( sList.IsEmpty() ) // Make sure list exists
      ListGet();
    else
      sIter = sList.Begin();
  }
  return sIter;
}

// SINGLETON
MotionIter &
Motion::NextGet()
{
  return sNext; // POINTS TO NOTHING BY DEFAULT
}

// DATA SINGLETON
static TrajType traj_type[] =
{
  { TRAJ_M_CIRC, "circ" },
  { TRAJ_M_CVEL, "cvel" },
  { TRAJ_M_HOME, "home" },
  { TRAJ_M_NULL, "null" },
  { TRAJ_M_PACE, "pace" },
  { TRAJ_M_RAND, "rand" },
  { TRAJ_M_SQAR, "sqar" },
  { TRAJ_M_SAVE, "save" },
  { TRAJ_M_JOYP, "joyp" },
  { TRAJ_M_JOYV, "joyv" },
  { TRAJ_M_ACCEL,"accel"},
  { TRAJ_M_STEP, "step" },
  { TRAJ_M_GOAL, "goal" },
  { TRAJ_M_ZERO, "zero" },
  { TRAJ_M_W   , "w"    },
};

int num_traj_type = sizeof(traj_type)/sizeof(TrajType);

TrajMode const
traj_name2mode( char *name )
{ for(int i=0; i<num_traj_type; i++ )
  { if( !strcmp(name, traj_type[i].name) )
      return traj_type[i].mode;
  }
  return (TrajMode) -1; // NAME NOT FOUND
}

char const *
traj_mode2name( TrajMode mode )
{ for(int i=0; i<num_traj_type; i++ )
  { if( mode == traj_type[i].mode )
      return traj_type[i].name;
  }
  return NULL; // MODE NOT FOUND
}


Motion::Motion( TrajMode _trajMode )
  : ax_(0), aw_(0), vx_(0), vw_(0), param(NULL), param_max(0)
{ // CREATE MOTION WITH DEFAULTS FOR GIVEN MOTION TYPE

  ctrlFrame_     = CTRL_F_GLOBAL; //LOCAL
  trajMode_      = _trajMode;
  controlMode_   = CTRL_LM;       //CALI
  accel_FF_      = true;          //false
  fricMode_      = FRIC_NONE;     //BOHR
  gathMode_      = false;         //false
  internalForce_ = true;          //false

  trajEnd_       = 60;


  KPx_ =    1200;
  KVx_ =      4;
  KPa_ =    1100;
  KVa_ =      2;

  KpE_ =    170;   //  70.0; //  75.0;


  switch( trajMode_ )
  {
  case TRAJ_M_RAND:
    ax_ = 1.00;
    aw_ = 0.30 * REV2RAD;
    vx_ = 0.75;
    vw_ = 0.20 * REV2RAD;
    break;

  case TRAJ_M_PACE:
    ax_ = 0.5;
    aw_ = 0.2 * REV2RAD;
    vx_ = 0.5;
    vw_ = 0.15 * REV2RAD;
    break;

  case TRAJ_M_CIRC:
    param_max = 5;
    param = new Param_struct[param_max];
    param[0].val = 0.100; strcpy(param[0].name,"r");   // r
    param[1].val = 2.000; strcpy(param[1].name,"wr");  // wr
    param[2].val = 0.000; strcpy(param[2].name,"a");   // a
    param[3].val = 1.600; strcpy(param[3].name,"wa");  // wa,
    param[4].val = 0.700; strcpy(param[4].name,"ramp"); // ramp
    break;

  case TRAJ_M_CVEL: // CONSTANT VELOCITY
    dxdd[0] = 0.0;
    dxdd[1] = 0.0;
    dxdd[2] = 0.0;
    break;

  case TRAJ_M_SQAR:
    ax_ = 0.40;
    aw_ = 0.15 * REV2RAD;
    vx_ = 0.70;
    vw_ = 0.15 * REV2RAD;
    break;

  case TRAJ_M_HOME:
    KPx_ = 100.0; //1000
    KVx_ =   2.0; //  20
    KPa_ = 100.0; //1000
    KVa_ =   2.0; //  20
    ax_ = 0.2;
    aw_ = 0.2 * REV2RAD;
    vx_ = 0.2;
    vw_ = 0.07 * REV2RAD;
    break;

  case TRAJ_M_JOYP:
    ax_ = 0.50;
    aw_ = 0.30 * REV2RAD;
    vx_ = 0.70;
    vw_ = 0.15 * REV2RAD;
    break;

  case TRAJ_M_JOYV:
    ctrlFrame_ = CTRL_F_LOCAL;
    ax_ = 0.50;
    aw_ = 0.30 * REV2RAD;
    vx_ = 0.70;
    vw_ = 0.15 * REV2RAD;
    break;

  case TRAJ_M_SAVE:
    ax_ = 0.3;
    aw_ = 0.3 * REV2RAD;
    vx_ = 0;
    vw_ = 0;
    break;

  case TRAJ_M_NULL:
    ax_ = 0.3;
    aw_ = 0.3 * REV2RAD;
    vx_ = 0;
    vw_ = 0;
    trajEnd_ = TRAJ_END_NONE;
    break;

  case TRAJ_M_ACCEL:
    param_max = 1;
    param = new Param_struct[param_max];
    param[0].val = 8.0;   strcpy(param[0].name,"a");
    break;

  case TRAJ_M_STEP:
    break;

  case TRAJ_M_GOAL:
    ax_ = 0.5;
    aw_ = 0.08 * REV2RAD;
    vx_ = 0.70;
    vw_ = 0.15 * REV2RAD;
    param_max = 3;
    param = new Param_struct[param_max];
    param[0].val = 0.0;   strcpy(param[0].name,"x");
    param[1].val = 0.0;   strcpy(param[1].name,"y");
    param[2].val = 0.0;   strcpy(param[2].name,"theta");
    trajEnd_ = TRAJ_END_NONE;
    break;

  case TRAJ_M_ZERO:
    ax_ = 0.3;
    aw_ = 0.3 * REV2RAD;
    vx_ = 0;
    vw_ = 0;
    trajEnd_ = TRAJ_END_NOW;
    break;

  case TRAJ_M_W:
    ax_ = 0.3;
    aw_ = 0.3 * REV2RAD;
    vx_ = 0;
    vw_ = 0;
    trajEnd_ = TRAJ_END_NOW;
    break;

  } // end switch TRAJ_MODE
}


Motion::~Motion()
{
  delete param;
}


void
Motion::Step()
{
  if( sNext.position ) // JUMP OVERRIDE SET ?
  {  sIter = sNext;    // UPDATE GLOBAL ITER
     sNext.position = NULL;
  }
  else                 // INCREMENT TO NEXT MOTION
  { if( sIter.position->next == NULL )//LAST? ADD NULL
      sList.BackPush( new Motion( TRAJ_M_NULL ) );

    ++sIter;           // UPDATE GLOBAL ITER
  }

  sIter->Init(); // INIT

  // JUST MOVE TO NEXT MOTION
  // DON'T BOTHER TO delete OLD MOTIONS [LAZY!]
  // (BUT WE HAVE A RECORD OF MOTIONS IF WE NEED IT)
}


void
Motion::Init()
{
  switch( trajMode_ )
  {
  case TRAJ_M_RAND:
    traj->curPos( gx  );
    traj->curVel( gxd );
    traj->accel(  ax_, aw_ );
    traj->maxVel( vx_, vw_ );

//     gObj.constant("ax",ax_);
//     gObj.constant("aw",aw_);
//     gObj.constant("vx",vx_);
//     gObj.constant("vw",vw_);
//     gObj.constant("kp",KP_);
//     gObj.constant("kv",KV_);
    break;

  case TRAJ_M_PACE:
    traj->curPos( gx  );
    traj->curVel( gxd );
    traj->accel(  ax_, aw_ );
    traj->maxVel( vx_, vw_ );

//     gObj.constant("ax",ax_);
//     gObj.constant("aw",aw_);
//     gObj.constant("vx",vx_);
//     gObj.constant("vw",vw_);
//     gObj.constant("kp",KP_);
//     gObj.constant("kv",KV_);
    break;

  case TRAJ_M_CIRC:
    init_traj_circle( param[0].val, // r
                      param[1].val, // wr
                      param[2].val, // a
                      param[3].val, // wa,
                      hw_freq,
                      param[4].val); // ramp
//    init_traj_circle( r_,wr_,a_,wa_,hw_freq,ramp_);

//     gObj.constant("r",r_);
//     gObj.constant("wr",wr_);
//     gObj.constant("a",a_);
//     gObj.constant("wa",wa_);
//     gObj.constant("ramp",ramp_);
//     gObj.constant("kp",KP_);
//     gObj.constant("kv",KV_);
    break;

  case TRAJ_M_CVEL:
    // CONSTANT VELOCITY
    break;

  case TRAJ_M_SQAR:
    traj->curPos( gx  );
    traj->curVel( gxd );
    traj->accel(  ax_, aw_ );
    traj->maxVel( vx_, vw_ );
    idx_sqar = 0;

//     gObj.constant("ax",ax_);
//     gObj.constant("aw",aw_);
//     gObj.constant("vx",vx_);
//     gObj.constant("vw",vw_);
//     gObj.constant("kp",KP_);
//     gObj.constant("kv",KV_);
    break;

  case TRAJ_M_GOAL:
    traj->curPos( gx  );
    traj->curVel( gxd );
    traj->accel(  ax_, aw_ );
    traj->maxVel( vx_, vw_ );
    destPos[0] = param[0].val;
    destPos[1] = param[1].val;
    destPos[2] = param[2].val;
    traj->dest2(destPos);
    break;

  case TRAJ_M_HOME:
    traj->curPos( gx  );
    traj->curVel( gxd );
    traj->accel(  ax_, aw_ );
    traj->maxVel( vx_, vw_ );

    destPos.setZero();
    traj->dest2(destPos);
    break;

  case TRAJ_M_JOYP:
    traj->curPos( gx  );
    traj->curVel( gxd );
    traj->accel(  ax_, aw_ );
    traj->maxVel( vx_, vw_ );
    break;

  case TRAJ_M_JOYV:
    x.setZero();  // ZERO THE LOCAL COORDS
    traj->curPos( x  );
    traj->curVel( xd );
    traj->accel(  ax_, aw_ );
    traj->maxVel( vx_, vw_ );
    break;

  case TRAJ_M_SAVE:
    if( gathMode_ == true )
    {
      puts("INIT: TRAJ_M_SAVE");
//       gObj.save();
    }
    else  // NOT SAVING SO SKIP THIS MODE
    { ctrlFrame_ = CTRL_F_GLOBAL;
      trajMode_ = TRAJ_M_NULL;
      trajEnd_ = TRAJ_END_NOW;
    }
    break;

  case TRAJ_M_NULL:
    traj->curPos( gx  );
    traj->curVel( gxd );
    traj->accel(  ax_, aw_ );
    traj->maxVel( vx_, vw_ );
    break;

  case TRAJ_M_ZERO:    // ALWAYS "TRAJ_END_NOW"
    gx.setZero();         // ZERO GLOBAL COORDS
    traj->curPos( gx  );
    traj->curVel( gxd );
    traj->accel(  ax_, aw_ );
    traj->maxVel( vx_, vw_ );
    break;

  case TRAJ_M_W:
    traj->curPos( gx  );
    traj->curVel( gxd );
    traj->accel(  ax_, aw_ );
    traj->maxVel( vx_, vw_ );
    break;

  case TRAJ_M_ACCEL:
    dxdd.setZero();
    dxdd[2] = param[0].val;

//    gObj.constant("a",a);
    break;

  case TRAJ_M_STEP:
    dx.setZero();
    dxd.setZero();
    dxdd.setZero();
    dx[0] = 0.015;
    break;
  } // end switch TRAJ_MODE

  // TRANSFORM DURATION INTO EXPIRATION TIME
  if( trajEnd_ > 0  &&  trajEnd_ < TRAJ_END_NONE )
    trajEnd_ += cur_time;

} // END Init()


void
Motion::Run()
{
// BEGIN TRAJECTORY RUN SECTION
      switch( trajMode_ )
      {
      case TRAJ_M_RAND:
        if( traj->get(dx, dxd, dxdd) == 0 )
        {
          destPos[0] = 1.0*(rand()/(Float)RAND_MAX);
          destPos[1] = 1.5*(rand()/(Float)RAND_MAX);
          destPos[2] = 4.0*(rand()/(Float)RAND_MAX);
          traj->dest2(destPos);
          cout << "t=  " << cur_time << endl;
          destPos.display();
        }
        break;

      case TRAJ_M_PACE:
        if( traj->get(dx, dxd, dxdd) == 0 )
        {
          if( !destPos[0] && !destPos[1] && !destPos[2] )
          { destPos[0] = 0;
            destPos[1] = 2.5;
            destPos[2] = 0.0;
          }
          else
          { destPos[0] = 0;
            destPos[1] = 0;
            destPos[2] = 0;
          }
          traj->dest2(destPos);
        }
        break;

      case TRAJ_M_CIRC:
        traj_circle(dx, dxd, dxdd);
        break;

      case TRAJ_M_CVEL:
        dx [0] += 0.0 / hw_freq;
        dxd[0]  = 0.0;
        dx [1] += 0.00 / hw_freq;
        dxd[1]  = 0.00;
        dx [2] += 0.500 / hw_freq;
        dxd[2]  = 0.500;
        break;

      case TRAJ_M_SQAR:
        if( traj->get(dx, dxd, dxdd) == 0 )
        {
          switch( idx_sqar % 4 )
          {
          case 0:
            destPos[0]=0.000; destPos[1]=1.200; destPos[2]= -3;
            break;
          case 1:
            destPos[0]=1.000; destPos[1]=1.200; destPos[2]=  0;
            break;
          case 2:
            destPos[0]=1.000; destPos[1]=0.000; destPos[2]=  5;
            break;
          case 3:
            destPos[0]=0.000; destPos[1]=0.000; destPos[2]=  0;
            break;
          default:
            destPos.setZero();
            break;
          } // end switch idx_sqar
          traj->dest2(destPos);
          cout << "t=  " << cur_time << endl;
          destPos.display();
          ++idx_sqar;
        }
        break; // end TRAJ_M_SQAR

      case TRAJ_M_GOAL:
        if( traj->get(dx, dxd, dxdd) == 0 )
          trajEnd_ = TRAJ_END_NOW;
        break;

      case TRAJ_M_HOME:
        if( traj->get(dx, dxd, dxdd) == 0 )
          trajEnd_ = TRAJ_END_NOW;
        break;

      case TRAJ_M_JOYP:
        joy->joyFill_cx( destPos );
        traj->dest2( destPos );
        traj->get(dx, dxd, dxdd);
        break;

      case TRAJ_M_JOYV:
        { static float jx=0,jy=0;
          static int jb=0,jb_old=-1;
          static Float funOff = 0;
          static Float funX = 0;

          joy->smooth(jx,jy,jb);
          if( jb != jb_old )
            switch( jb )
            { case 0:
              case 1:
              case 2:
                ctrlFrame_ = CTRL_F_LOCAL;
                traj->curPos( x  );
                traj->curVel( xd );
                break;
              case 3:
                ctrlFrame_ = CTRL_F_GLOBAL;
                traj->curPos( gx  );
                traj->curVel( gxd );
                funX = gx[2] + funOff;
                break;
            }
          switch(jb)
          { case 1:
              cxd[0] =  jx;
              cxd[1] =  jy;
              cxd[2] =   0;
              traj->get_uV( cxd, dx, dxd, dxdd );
              break;
          case 2:
            cxd[0] = -jy*sin(funOff);
            cxd[1] =  jy*cos(funOff);
            cxd[2] = -jx;
            traj->get_uV( cxd, dx, dxd, dxdd );
            break;
          case 3:
            funOff -= rxd[2]/hw_freq;
            cxd[0] = -jy*sin(funX);
            cxd[1] =  jy*cos(funX);
            cxd[2] = -jx;
//                 cxd[0] = -jy*sin(funOff) + 0.5*jy*rxd[2]*cos(funOff);
//                 cxd[1] =  jy*cos(funOff) + 0.5*jy*rxd[2]*sin(funOff);
            traj->get_uV( cxd, dx, dxd, dxdd );
            break;
          case 0:  // MANLIFT (EMULATE KINEMATICS AND STATE OF SCISSOR-LIFT)
            if( fabs(jx) < 0.02 )  // AVOID DIVISION BY ZERO
            { cxd[0] =  0;
              cxd[1] = jy * 0.999; // LIMIT TO BOUNDARY SPEED
              cxd[2] =  0;
            }
            else
            { double L         =  1.850;  // WHEELBASE
              double c         =  0.350;  // DIST OF XR BEHIND STEERED WHEELS
              double theta_max = 80.0 * DEG2RAD; // MAX STEER ANG.
              double roll_max  =  0.750;  // MAX LIN VEL OF DRIVEN WHEEELS
              double b         =   L-c;
              double r = L/tan(jx*theta_max); // DIST TO IC ALONG FIXED AXIS
              jy *= roll_max; // MAX DRIVEN-WHEEL SPEED OF MANLIFT
              double omega = -jy/hypot(r,b);  // ROT_VEL AROUND IC
              if( r<0 ) omega = -omega;
              cxd[0] =  -omega * b;
              cxd[1] =  -omega * r;
              cxd[2] =   omega;
            }
            traj->get_V_lin( cxd, dx, dxd, dxdd );
            break;
          default:
            cxd.setZero();
            traj->get_uV( cxd, dx, dxd, dxdd );
            break;
          }
          jb_old = jb; // AGE BUTTON VALUE
        }
        break;

      case TRAJ_M_SAVE:
        dx = gx;
        dxd = gxd;
        dxdd.setZero();

//             if( gObj.state() == GATHER_STATE_WRITTEN )
//             { trajEnd  = TRAJ_END_NOW;
//             }
        break;

      case TRAJ_M_NULL:
        dx = gx;
        dxd.setZero();
        dxdd.setZero();
        break;

      case TRAJ_M_ACCEL:
        if( fabs(gxd[0]) > 1.2 ||
            fabs(gxd[1]) > 1.2 ||
            fabs(gxd[2]) > 6.0 )
        { trajMode_ = TRAJ_M_NULL;
          trajEnd_  = TRAJ_END_NOW;
        }
        break;

      case TRAJ_M_STEP:
        break;

      case TRAJ_M_ZERO:  // SHOULD NEVER GET HERE
        break;

      case TRAJ_M_W:     // SHOULD NEVER GET HERE
        break;

      } // end switch TRAJ_MODE
} // END Run()


#define MAXzPOS  0.100 // m
#define MAXzANG  0.350 // 20 deg

bool
Motion::Goal_is_valid()
{
  static Vector3d zx; // STATIC FOR SPEED
  GET_mi;

  if( mi->ctrlFrame_ == CTRL_F_GLOBAL )
  { zx = dx - gx;
  }
  else
  { zx = dx -  x;
  }
  if( hypot(zx[0],zx[1]) > MAXzPOS ||
             fabs(zx[2]) > MAXzANG )
  { cout << "ERR: goal too far, skip to next Motion." << endl;
      return false;
  }

  return true;
}

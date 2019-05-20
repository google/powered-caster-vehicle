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


#ifndef _Motion_h_
#define _Motion_h_

#include "MyDeque.h"  // NEED TO REPLACE WITH A MODERN LIB TO DO THIS


typedef enum {
 TRAJ_M_CIRC,
 TRAJ_M_CVEL,
 TRAJ_M_HOME,
 TRAJ_M_NULL,
 TRAJ_M_PACE,
 TRAJ_M_RAND,
 TRAJ_M_SQAR,
 TRAJ_M_SAVE,
 TRAJ_M_JOYP,
 TRAJ_M_JOYV,
 TRAJ_M_ACCEL,
 TRAJ_M_STEP,
 TRAJ_M_GOAL,
 TRAJ_M_ZERO,
 TRAJ_M_W,
} TrajMode;

typedef struct
{ TrajMode mode;
  char name[12];
} TrajType;

typedef enum {
 CTRL_LM,
 CTRL_L,
 CTRL_L_CONST,
 CTRL_TEST,
 CTRL_CALI,
} CtrlMode;

typedef enum {
 CTRL_F_LOCAL,
 CTRL_F_GLOBAL,
} CtrlFrame;

typedef enum {
 FRIC_NONE,
 FRIC_ANTI,
 FRIC_BOHR,
//  FRIC_SIGD,
} FricMode;

typedef struct Param_struct
{ Float val;
  char  name[20];
} Param_struct;


#define TRAJ_END_EXIT  (-100)
#define TRAJ_END_NOW   (-1)
#define TRAJ_END_NONE  ( 1E7) // 115 DAYS


class Motion
{
 public:
  Motion( TrajMode );
 ~Motion();

  void Step();
  void Init();
  void Run();
  static bool Goal_is_valid();

  // STATIC FUNCTIONS FOR SINGLETONs
  static MyDeque<Motion *>         &ListGet();
  static MyDequeIterator<Motion *> &IterGet();
  static MyDequeIterator<Motion *> &NextGet();

  // VARIABLES
  CtrlFrame ctrlFrame_;
  TrajMode  trajMode_;
  CtrlMode  controlMode_;
  int       accel_FF_;
  FricMode  fricMode_;
  int       gathMode_;
  int       internalForce_;

  Float     trajEnd_;// HOLDS duration UNTIL ACTIVE MOTION

  Float   KPx_,KVx_,KPa_,KVa_,KpE_;
  Float   ax_,aw_,vx_,vw_;     // Traj3
  Param_struct  *param;
  int            param_max;

//  Float   r_,wr_,a_,wa_,ramp_; // traj_circle

};


// MOTION LIST TYPES
typedef MyDeque<Motion *>         MotionList;
typedef MyDequeIterator<Motion *> MotionIter;

// SHORTCUT TO GET LOCAL COPY OF SINGLETON
#define GET_mq   MotionList &mq = Motion::ListGet(); // get Motion Queue
#define GET_mi   MotionIter &mi = Motion::IterGet(); // get Motion Itererator
#define GET_mx   MotionIter &mx = Motion::NextGet(); // get Motion Next


#define REV2RAD  (6.28318530718)
#define DEG2RAD  (0.0174532925199)

// PROTOTYPES
TrajMode const  traj_name2mode( char *name );
char     const *traj_mode2name( TrajMode mode );

#endif // _Motion_h_

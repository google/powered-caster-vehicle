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


#ifndef _PCV_State_h_
#define _PCV_State_h_

#include <Eigen/Dense>
#include "PCV_Types.h"

#include "Traj3.h"

using namespace Eigen;

#ifndef PCV_STATE_GLOBAL_CREATE
 extern Float hw_freq;
 extern Float cur_time;

 // PREFIX: d=desired, g=global, r=raw ; SUFFIX d=dot, dd=dot.dot
 //              pos   vel   acc
 extern Vector8d   q,   qd;

 extern Vector3d       rxd;
 extern Vector3d      rgxd;
 extern Vector3d   x,   xd;
 extern Vector3d  gx,  gxd, gxdd;
 extern Vector3d  dx,  dxd, dxdd;

 extern Traj3    *traj;
#endif
#endif // _PCV_State_h_

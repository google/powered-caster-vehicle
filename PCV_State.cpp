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


#define PCV_STATE_GLOBAL_CREATE

#include <Eigen/Dense>
#include "PCV_Types.h"

#include "PCV_State.h"

using namespace Eigen;

// CREATE THESE GLOBAL VARIABLES HERE
Float hw_freq;
Float cur_time;

// PREFIX: d=desired, g=global, r=raw ; SUFFIX d=dot, dd=dot.dot
//       pos   vel   acc
Vector8d   q              = Vector8d::Zero()
Vector8d        qd        = Vector8d::Zero()

Vector3d       rxd        = Vector3d::Zero();
Vector3d      rgxd        = Vector3d::Zero();
Vector3d   x              = Vector3d::Zero();
Vector3d        xd        = Vector3d::Zero();
Vector3d  gx              = Vector3d::Zero();
Vector3d       gxd        = Vector3d::Zero();
Vector3d            gxdd  = Vector3d::Zero();
Vector3d  dx              = Vector3d::Zero();
Vector3d       dxd        = Vector3d::Zero();
Vector3d            dxdd  = Vector3d::Zero();

Traj3  *traj;

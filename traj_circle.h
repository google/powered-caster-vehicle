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


#ifndef _traj_circle_h_
#define _traj_circle_h_

#include <Eigen/Dense>
#include "PCV_Types.h"

using namespace Eigen;

void
init_traj_circle(double r,
                 double wr,
                 double a,
                 double wa,
                 double hz,
                 double ramp);

void
traj_circle(Vector3d &x, Vector3d &xd, Vector3d &xdd);

#endif // _traj_circle_h_

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


#ifndef _Traj3_h_
#define _Traj3_h_

#include <Eigen/Dense>

using namespace Eigen;

class
Traj3
{
public:
  Traj3(double freq);
 ~Traj3();

  void curPos(Vector3d &_curPos);
  void curVel(Vector3d &_curVel);
  void maxVel(double v_lim=0.5, double w_lim=1.57);
  void maxVel(Vector3d &_maxVel);
  void accel(double lin_acc=0.25, double rot_acc=1.0);
  void accel(Vector3d &acc);
  void dest2(Vector3d &destPos);

  // FOR VELOCITY CONTROL
  void get_uV(Vector3d &destUvel, Vector3d &trajPos,
              Vector3d &trajVel,  Vector3d &trajAcc);

  void get_V_lin(Vector3d &destvel, Vector3d &trajPos,
                 Vector3d &trajVel, Vector3d &trajAcc);

  int get(Vector3d &trajPos, Vector3d &trajVel, Vector3d &trajAcc);

private:
  double period_;
  double v_lim_;
  double w_lim_;
  double lin_acc_;
  double rot_acc_;

  Vector3d *curPos_;
  Vector3d *curVel_;

  Vector3d *destPos_;
  Vector3d *destVel_;
  Vector3d *acc_;
  Vector3d *maxVel_;

};

#endif // _Traj3_h_

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


#ifndef _RAHjoy_H_
#define _RAHjoy_H_

#include <Eigen/Dense>

#include "PCV_Joystick.h"

#define MAX_JOY_V 1.0
#define MAX_JOY_W 1.0


#define JOY_X_MAX   282
#define JOY_Y_MAX   295

#define JOY_X_DED    16
#define JOY_Y_DED    16



class
RAHjoy:public PCV_Joystick
{
public:
  RAHjoy(void);
  RAHjoy(const Float hz);
  RAHjoy(const Float hz, int xmax, int xmin, int ymax, int ymin );
 ~RAHjoy(){};

  void norm( float &x, float &y, int &b);
  void smooth( float &x, float &y, int &b);
  Eigen::Vector3d joyFill_cx(); // <-- NEEDS HZ FIRST!


private:
  void init(const Float hz);

  int x_max_,x_min_,y_max_,y_min_;
  Float x_dead_,y_dead_;
  Float hz_;
};

#endif // _RAHjoy_H_

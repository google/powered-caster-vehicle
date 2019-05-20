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


#include <stdio.h>
#include <math.h>

#include <Eigen/Dense>
#include "PCV_Types.h"

#include "traj_circle.h"

using namespace Eigen;

static double hz_;
static double r_,wr_;
static double a_,wa_;
static double ramp_,t_;


void
init_traj_circle(double r,
                 double wr,
                 double a,
                 double wa,
                 double hz,
                 double ramp)
{
  r_  = r;
  wr_ = wr;
  a_  = a;
  wa_ = wa;
  hz_ = hz;
  ramp_ = ramp;

  t_=0.0;
}

void
traj_circle(Vector3d &x, Vector3d &xd, Vector3d &xdd)
{
  double wr,wa;

   if( t_<ramp_ )
   { wr = t_/ramp_ * wr_;
     wa = t_/ramp_ * wa_;
   }
   else
   { wr = wr_;
     wa = wa_;
   }

    x[0] =  r_*      cos( wr*t_ ) - r_ ;
   xd[0] = -r_*wr*   sin( wr*t_ ) ;
  xdd[0] = -r_*wr*wr*cos( wr*t_ ) ;

    x[1] =  r_*      sin( wr*t_ ) ;
   xd[1] =  r_*wr*   cos( wr*t_ ) ;
  xdd[1] = -r_*wr*wr*sin( wr*t_ ) ;

    x[2] =  a_*   (1-cos( wa*t_ ));
   xd[2] =  a_*wa*   sin( wa*t_ ) ;
  xdd[2] =  a_*wa*wa*cos( wa*t_ ) ;


  t_ += 1.0/hz_;

}

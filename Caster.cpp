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


#include <cmath>

#include <Eigen/Dense>
#include "PCV_Types.h"

#include "Caster.h"

using namespace Eigen;

Caster::Caster(
            double _Kx, double _Ky, double _ang,
            double  _b, double  _r,
            double  _f, double _Mf, double _If,
            double _Ih, double _Ii, double _Is, double _It, double _Ij,
            double _Ns, double _Nt, double _Nw,
            double _px, double _py, double _Mp, double _Ip)
    :Lambda(M3Z), Mu(V3Z),
     Ce(M3I), Jdot(M3Z), A(M3Z), CC(V3Z), L_Pk(M3Z), Mu_Pk(V3Z)
{
 Kx = _Kx;   Ky = _Ky;
  b = _b;    bi= 1.0/b;   r = _r;    ri= 1.0/r;
  f = _f;    e = f-b;    Mf = _Mf;  If = _If;
 Ih = _Ih;  Ii = _Ii;    Is = _Is;  It = _It;    Ij = _Ij;
 Ns = _Ns;  Nt = _Nt;    Nw = _Nw;
 px = _px;  py = _py;    Mp = _Mp;  Ip = _Ip;

 Px = Kx + px;
 Py = Ky + py;
 P2  = pow(Px,2) + pow(Py,2);

 enc_offset = (long)(((M_PI_2 - _ang) * Ns * ENC_RAD2CNT)+0.5);

 Fill_A();
 Fill_L_Pk();
}


void
Caster::Fill_A()
{
  double mfe2Ifih= Mf*e*e + If + Ii + Ih;
  double IsNs    = Is * Ns;
  double ItNt    = It * Nt;
  double ItNt2   = ItNt*Nt;

  A(0,0) = mfe2Ifih + IsNs*Ns + ItNt;
  A(0,1) = (Ii - Ih - ItNt2) * Nw;
  A(0,2) = mfe2Ifih - IsNs - ItNt;

  A(1,0) = A(0,1);
  A(1,1) = Mf*r*r + (Ii + Ih + ItNt2)*Nw*Nw + Ij;
  A(1,2) = (Ii - Ih + ItNt ) * Nw;

  A(2,0) = A(0,2);
  A(2,1) = A(1,2);
  A(2,2) = mfe2Ifih + Is + It;
}


inline void
Caster::Fill_CC()
{
  register double  u_w = u + w;
  register double Mfre = Mf*r*e;

  CC(0) =  Mfre * v * u_w;
  CC(1) = -Mfre * pow(u_w,2);
  CC(2) =  CC[0];
}


void
Caster::Fill_L_Pk()
{
  L_Pk(0,0) =  Mp;
  L_Pk(0,1) =  0.0;
  L_Pk(0,2) = -Mp * Py;

  L_Pk(1,0) =  0.0;
  L_Pk(1,1) =  Mp;
  L_Pk(1,2) =  Mp * Px;

  L_Pk(2,0) = L_Pk(0,2);
  L_Pk(2,1) = L_Pk(1,2);
  L_Pk(2,2) = Ip + Mp*P2;
}


inline void
Caster::Fill_Mu_Pk()
{
  register double w2  = pow(w,2);

  Mu_Pk(0) = -Mp * Px * w2;
  Mu_Pk(1) = -Mp * Py * w2;
  Mu_Pk(2) =  0.0;
}


inline void
Caster::Fill_Ce()  // Ce (C_theta) (a.k.a. Ji )
{
  Ce(0,0) =  bi*s;
  Ce(0,1) = -bi*c;
  Ce(0,2) = -bi*(Kx*c + Ky*s) - 1.0;

  Ce(1,0) =  ri*c;
  Ce(1,1) =  ri*s;
  Ce(1,2) =  ri*(Kx*s - Ky*c);

  Ce(2,0) =  0.0;
  Ce(2,1) =  0.0;
  Ce(2,2) =  1.0;
}


inline void
Caster::Fill_Jdot()
{
  register double u_w = u + w;

  Jdot(0,0) =  b*c*u_w;
  Jdot(0,1) = -r*s*u_w;
  Jdot(0,2) =  Kx*w + b*c*u_w;

  Jdot(1,0) =  b*s*u_w;
  Jdot(1,1) =  r*c*u_w;
  Jdot(1,2) =  Ky*w + b*s*u_w;

  Jdot(2,0) =  0.0;
  Jdot(2,1) =  0.0;
  Jdot(2,2) =  0.0;
}


void
Caster::Fill_LM( double const _u,   // sigma dot
                 double const _v,   //  rho  dot
                 double const _w)   // theta dot
{

  static Vector3d   Qdot(3);

  // SET STATE
  Qdot[0] = u = _u;
  Qdot[1] = v = _v;
  Qdot[2] = w = _w;

  // FILL JACOBIANS
  Fill_Ce();
  Fill_Jdot();

  // FILL JT-SPACE CC VECTOR (NOTE: 'A' IS CONSTANT)
  Fill_CC();
  // FILL OP-SPACE CC VECTOR (NOTE: 'L_Pk' IS CONSTANT)
  Fill_Mu_Pk();

  //Lambda = L_Pk + (Cet * A * Ce)
  Lambda = L_Pk + Ce.transpose() * A * Ce;

  //Mu = Mu_Pk + [Cet * ( CC - (A*(Ce*(Jdot*Qdot))) )]
  Mu = Mu_Pk + Ce.transpose()*(CC-(A*(Ce*(Jdot*Qdot))));
}

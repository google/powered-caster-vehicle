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


#ifndef _Caster_h_
#define _Caster_h_

#include <cmath>

#include <Eigen/Dense>
#include "PCV_Types.h"

#define XR_h  ( 0.2159)      // m
#define XR_b  (-0.020*0.995) // m ( Dimension * Empirical_Calibration_Factor )
#define XR_r  ( 0.055*1.000) // m ( Dimension * Empirical_Calibration_Factor )

#define XR_f  (-0.010)       // m
#define XR_Mf ( 2.720)       // kg
#define XR_If ( 3.32e-3)     // kg m2

#define XR_Ih ( 4.78e-4)     // kg m2
#define XR_Ii ( 1.74e-5)     // kg m2
#define XR_Is ( 7.11e-5)     // kg m2
#define XR_It ( 8.53e-5)     // kg m2
#define XR_Ij ( 4.00e-4)     // kg m2

#define XR_Ns ( 96.0/24.0)   //
#define XR_Nt ( 85.0/35.0)   //
#define XR_Nw (  2.0/ 1.0)   //

#define XR_px (  0.0)        // m
#define XR_py (  0.0)        // m
#define XR_Mp (  5.2)        // kg
#define XR_Ip (  0.022)      // kg m2

// Vehicle Constants (without Caters).  Change for CoM (x,y) Mass and Inertia
#define XR_Vx (  0.0 )       // m
#define XR_Vy (  0.0 )       // m
#define XR_Mv ( 30.0 )       // kg
#define XR_Iv (  2.1 )       // kg m2


#define ENC_REV2CNT (8192.0)
#define ENC_CNT2REV (1.0/ENC_REV2CNT)
#define ENC_RAD2CNT (ENC_REV2CNT/M_2_PI)
#define ENC_CNT2RAD (M_2_PI/ENC_REV2CNT)

#define Nm_PER_AMP   0.32


using namespace Eigen;

class Caster
{
 public:
  Caster(double _Kx,       double _Ky,       double _ang,
         double  _b=XR_b,  double  _r=XR_r,
         double  _f=XR_f,  double _Mf=XR_Mf, double _If=XR_If,
         double _Ih=XR_Ih, double _Ii=XR_Ii,
         double _Is=XR_Is, double _It=XR_It, double _Ij=XR_Ij,
         double _Ns=XR_Ns, double _Nt=XR_Nt, double _Nw=XR_Nw,
         double _px=XR_px, double _py=XR_py,
         double _Mp=XR_Mp, double _Ip=XR_Ip);
 ~Caster(){}

  Matrix3d Lambda;
  Vector3d Mu;

  void Fill_LM( double const _u,   // sigma dot
                double const _v,   //  rho  dot
                double const _w);  // theta dot

  double Kx,Ky;   // x,y COORDS FROM VEHICLE CENTER TO STEER AXIS
  double b,bi;    // CASTER OFFSET (negative), b INVERSE
  double r,ri;    // WHEEL RADIUS  (positive), r INVERSE
  double f, e, Mf, If;  // FORK CG FROM STEER AXIS (negative), e, MASS, INERTIA
  double Ih, Ii, Is, It, Ij; // INERTIAS
  double Ns, Nt, Nw; // GEAR RATIOS, STEER, TRANSLATE, WHEEL
  double px,py,Mp,Ip;// PUMPKIN CG x,y COORDS FROM STEER AXIS, MASS, INERTIA

  double Px,Py,P2; // PUMPKIN GC x,y COORDS FROM VEHICLE CENTER

  double a,c,s;  // sigma, cos(sigma), sin(sigma)
  double u,v,w;  // sigma dot, rho dot, theta dot

  long   enc_offset; // FOR CALIBRATION OF EXACT HOME SENSOR POSITION

 private:
         void Fill_A();
  inline void Fill_CC();

         void Fill_L_Pk();
  inline void Fill_Mu_Pk();

  inline void Fill_Ce();
  inline void Fill_Jdot();

  Matrix3d Ce;
  Matrix3d Jdot;
  Matrix3d A;
  Vector3d CC;

  Matrix3d L_Pk;
  Vector3d Mu_Pk;

};

#endif // _Caster_h_

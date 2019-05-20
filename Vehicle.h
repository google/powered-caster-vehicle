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


#ifndef _Vehicle_h_
#define _Vehicle_h_

#include <Eigen/Dense>
#include "PCV_Types.h"

#include "Caster.h"

using namespace Eigen;


#define NUM_TRUSS_LINKS  6


class
Vehicle
{
public:

  static Vehicle* HandleGet(); // SINGLETON
                               // CREATION DOES A Home()
  ~Vehicle();

  void Home( void );

  void JointRad(Vector8d &jRad);
  void MotorTq(Vector8d const &mTq);
  void JointTq(Vector8d const &jTq);

  void JtTq2MotAmp(Vector8d const &jtq, Vector8d &motAmp);

  // ADD PARTS THAT ARE FIXED TO VEHICLE
  // USE x,y LOCATION OF CENTER OF MASS OF EACH PIECE
  // USE INERTIA ABOUT PIECE'S OWN CENTER OF MASS
  void Add_Solid(double _x, double _y, double _M, double _I);

  // FILL Lambda AND Mu FOR VEHICLE (THIS OBJECT)
  void Dyn(Vector8d qd, double w);

  void Add_Caster(double Kx, double Ky, double ang,
         double  _b=XR_b,  double  _r=XR_r,
         double  _f=XR_f,  double _Mf=XR_Mf, double _If=XR_If,
         double _Ih=XR_Ih, double _Ii=XR_Ii,
         double _Is=XR_Is, double _It=XR_It, double _Ij=XR_Ij,
         double _Ns=XR_Ns, double _Nt=XR_Nt, double _Nw=XR_Nw,
         double _px=XR_px, double _py=XR_py,
         double _Mp=XR_Mp, double _Ip=XR_Ip);

  void Fill_C(Matrix8x3d &C);
  void Fill_Jcp(Matrix3x8d &J);
  void Fill_Jt_gamma(Matrix8x3d &Jt);
  void Fill_E_p(MatrixXd &E);
  void Fill_E_q(MatrixXd &E);

  Vector3d Fill_tqS(Vector8d const &qd, Vector8d const &tq,
                    Vector8d &tqS);

  Matrix3d Lambda;
  Vector3d Mu;

  double M_gross;    // Total Mass of vehicle + casters
  double I_gross;    // Approximate total Inertia of vehicle + casters
  double X_gross, Y_gross; // CoM of approx total mass

private:

  HWInterface* hwi_;

  Vehicle();

  bool  IsValid() const;

  void Add_Gross(double _x, double _y, double _M, double _I);
  void Fill_Mu_veh( double w );


  double X_veh,Y_veh,M_veh,I_veh;

  Caster *cstr[4];
  int num_casters;

  Matrix3d L_veh;
  Vector3d Mu_veh;
};

#endif // _Vehicle_h_


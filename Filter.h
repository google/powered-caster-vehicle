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


#ifndef _Filter_h_
#define _Filter_h_

#include <Eigen/Dense>
#include "PCV_Types.h"

using namespace Eigen;

class
Filter
{
public:
  Filter();
  Filter(Filter const &);
  Filter &operator=(Filter const &);
 ~Filter();

  // Use this after a filter has been initialized to filter the data
  VectorXd Filt(VectorXd const &in);


  // General digital filter: z_num/zden
  void Z_NumDen(VectorXd const &r0,
                VectorXd const &z_num, VectorXd const &z_den,
                VectorXd const &f0 );

  // Filters below are digital equivalents using Tustin's method
  void LowPass   // w/(s+w)
       (VectorXd const &r0, float sampFreq, float cutOffFreq);
  void D_LowPass  // (s*w)/(s+w)
       (VectorXd const &r0, float sampFreq, float cutOffFreq);
  void D         //  s
       (VectorXd const &r0, float sampFreq);
  void LeadLag   // (s+w1)/[(s+w2)(s+w3)]
       (VectorXd const &r0, float sampFreq, float f1, float f2, float f3 );
  void Unity();  //  1

private:
  int size_;
  int order_;

  VectorXd *a_;
  VectorXd *b_;

  VectorXd **f_;
  VectorXd **r_;

  void Allocate( int size, int order );
  void Purge();
};

#endif // _Filter_h_

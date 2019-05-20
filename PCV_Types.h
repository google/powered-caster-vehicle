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

#include <Eigen/Dense>

typedef Eigen::Matrix<double,8,1> Vector8d;
typedef Eigen::Matrix<double,8,3> Matrix8x3d;
typedef Eigen::Matrix<double,3,8> Matrix3x8d;

// For shorthand use in class constructor initialization list.
#define M3I (Eigen::Matrix3d::Identity())
#define M3Z (Eigen::Matrix3d::Zero())
#define V3Z (Eigen::Vector3d::Zero())

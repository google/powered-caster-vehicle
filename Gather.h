gg/*======================================================================
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


#ifndef _Gather_h_
#define _Gather_h_

#include <Eigen/Dense>

using namespace Eigen;

#define MAX_NUM_CONSTANTS  40 // MAX CONSTANTS OF ANY ONE TYPE
#define MAX_NUM_VECTORS    40 // MAX DATA VARIABLES OF ANY ONE TYPE

#define MAX_STR_LEN  80

#define PRIORITY_GATHER  28  // ECOS: 0=hi, 31=idle_thread

typedef enum {
  GATHER_STATE_EMPTY,  // READY TO GATHER DATA, BUFFERS ARE EMPTY
  GATHER_STATE_READY,  // READY TO GATHER DATA, BUFFERS CONTAIN SOME DATA
  GATHER_STATE_FULL,   // MAXIMUM NUMBER SAMPLES GATHERED, BUFFERS ARE FULL
  GATHER_STATE_SAVING, // DATA IS CURRENTLY BEING WRITTEN TO DISK
  GATHER_STATE_WRITTEN,// DATA HAS BEEN WRITTEN TO DISK AND FILE IS CLOSED
  GATHER_STATE_ERROR,  // THERE HAS BEEN AN ERROR
} GatherState;


typedef struct t_Data
{ Float *dataIn;
  Float *data;
  char   name[MAX_STR_LEN];
  int  row;
  int  col;
} T_Data;


class
Gather
{
public:

  // OPENS A FILE FOR GATHERING DATA
  // CREATE AS A GLOBAL VARIABLE OR
  // USE "static Gather" TO INITIALIZE    (DO NOT HOG THE STACK!)
  Gather( char const *filename, int const max_num_samples );
 ~Gather();

  // GATHER A CONSTANT...
  // CONSTANT VALUES GATHERED ONCE, AT THE TIME constant() IS CALLED
  // CONSTANT VALUES NOT GATHERED WHEN gather() METHOD IS CALLED
  void constant(char const *name, Float const     &dataIn);
  void constant(char const *name, VectorXd const  &dataIn);
  void constant(char const *name, MatrixXd const  &dataIn);

  // INDICATE WHAT DATA TO GATHER...
  // DATA VALUES NOT GATHERED WHEN data() IS CALLED
  // DATA VALUES GATHERED EVERY TIME gather() METHOD IS CALLED
  void data(char const *name, Float const     &dataIn);
  void data(char const *name, VectorXd const  &dataIn);
  void data(char const *name, MatrixXd const  &dataIn);

  // GATHERS data VALUES...  (TRY THIS: CALL gather() EACH SERVO TICK!)
  void gather( double const time );

  // WRITES THE GATHERED DATA VALUES TO A FILE
  // WILL ONLY SAVE AS MANY SAMPLES AS WERE GATHERED
  // BUILDS & SAVES FILE IN NEW thread at "PRIORITY_GATHER"
  void save();

  // RESET OBJECT, READY TO GATHER A NEW BATCH
  void reset();
  // RESET OBJECT, READY TO GATHER A NEW BATCH
  void reset( char const *filename, int const num_samples );

  // CHECK STATE OF OBJECT
  GatherState state(){ return state_; }

  // START TFTP SERVER AND FILE SYSTEM
  static void tftp_server_start(void);

  // CREATE/USE SINGLETON
  static Gather *Gather::HandleGet
    ( char const *filename, int const max_num_samples );

private:

 char         filename_[MAX_STR_LEN];
// float       *tData_;
 Float  t_;

 T_Data    C_[MAX_NUM_CONSTANTS];
 int    numC_;

 T_Data    D_[MAX_NUM_VECTORS];
 int    numD_;

 int    max_samples_;
 int    gatherCount_;

 GatherState  state_;

 void  *SaveThread_Stack_;
 friend void  saveThread_(unsigned int obj);

 void filename_set_(char const *filename);
 void zero_();
 void freeData_();

};

#endif //  _Gather_h_


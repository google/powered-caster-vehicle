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

// STANDARD HEADERS
#include <iostream>
#include <cmath>

#include <Eigen/Dense>
#include "PCV_Types.h"

// MODULE HEADERS
#include "Vehicle.h"
#include "Filter.h"
#include "Gather.h"
#include "RAHjoy.h"
#include "Servo_Timer.h"

#include "Traj3.h"
#include "traj_circle.h"

#include "PCV_State.h"
#include "Motion.h"

using namespace std
using namespace Eigen

#define HZ  500.0

#define GATHER_TIME  10


void
mymain( timer_addrword_t )
{
// START THE TIMER -- GET THE HW RESOLUTION
//    (START TIMER FIRST TO BE DOMINANT SERVO RATE)
    hw_freq = HZ;
    Servo_Timer *servo = Servo_Timer::HandleGet(hw_freq);
    Float hw_period = 1.0/hw_freq;
    cout << "Servo Rate = " << hw_freq << endl;

static Gather gObj("xr",GATHER_TIME*(int)HZ);

// INTIALIZE traj (GLOBAL)
    traj = new Traj3(hw_freq);

// MOTION ITERATOR
///    GET_mi;  // CREATE "mi" (current) MOTION ITERATOR.


// Zero the Base (ZERO HAPPENS UPON Vehicle OBJ CREATION)
    Vehicle *veh = Vehicle::HandleGet();

// FOR CENTER ROTATION OF XR4000 ROBOT
    veh->Add_Solid(  0.000,  0.000,  XR_Mv, XR_Iv);
    veh->Add_Caster(  0.000,  XR_h,  0*M_PI_2 - 20*ENC_CNT2RAD);
    veh->Add_Caster( -XR_h,   0.000, 1*M_PI_2 - 85*ENC_CNT2RAD);
    veh->Add_Caster(  0.000, -XR_h,  2*M_PI_2 - 75*ENC_CNT2RAD);
    veh->Add_Caster(  XR_h,   0.000, 3*M_PI_2 - 80*ENC_CNT2RAD);

    veh->Home(); // SET ENCODER PRESETS TO CASTER ANGLES

// LOCAL VARIABLES
    int k=0;
    long tick_miss=0;

    Float heading;

    Matrix3x8d J;
    Matrix8x3d Jt;
    Matrix3d   Lambda;
    Vector3d   Mu;
    Vector3d   cf;

    Vector3d   cxdd;
    Vector3d  fcxdd;
    Vector3d  cgxdd;

    Matrix8x3d C;
    Vector8d   qd_bar, qd_null;
    Vector8d   tq;

    Vector8d rtqS, tqS;
    Vector3d  rfS,  fS;
    Vector8d motAmp; // extra, not needed for control
    Vector8d tqI;    // extra, not needed for control

    Matrix<double, NUM_TRUSS_LINKS,8> E;
    VectorXd<NUM_TRUSS_LINKS> rtE;
    VectorXd<NUM_TRUSS_LINKS> tE;
    VectorXd<NUM_TRUSS_LINKS> ctE;

    Vector8d tqE;

    Matrix3d rot = Matrix3d::Identity();

// VECTORS TO GATHER
    gObj.data("tq",tq);
    gObj.data("gx",gx);
    gObj.data("gxd",gxd);
    gObj.data("dx",dx);
    gObj.data("dxd",dxd);
    gObj.data("q",q);
    gObj.data("qd",qd);
    gObj.data("tE",tE);
    gObj.data("tqE",tqE);

    gObj.data("amp",motAmp);   // extra
    gObj.data("tqI",tqI);      // extra
    gObj.data("tqS",tqS);      // extra
    gObj.data("qd_n",qd_null); // extra

// CHOOSE FILTERS
//  SERVO RATE          500  // 100 //  100 //  500
//  ----------          ---
    Float const FQD  =  150; // 42; //  45; // 145;
    Float const FGXD =   52; //  0; //  38; //  57;
    Float const FCXDD=    0; //
    Float const FTE  =   -1; //  0; //  30; //  45;
    Float const FXD  =   52; // 20; //  38; //  57;
    Float const FTQS =   42; // 42;

// INITIALIZE FILTERS
    Filter Fqd;
    veh->JointRad(q);
    if(FQD) Fqd.D_LowPass(q, hw_freq, FQD);
    else    Fqd.D(q, hw_freq);

    Filter Fgxd;
    rgxd.setZero();
    if(FGXD) Fgxd.LowPass(rgxd, hw_freq, FGXD);
    else     Fgxd.Unity();

    Filter Fcxdd;
    cxdd.setZero();
    if(FCXDD) Fcxdd.LowPass(cxdd, hw_freq, FCXDD);
    else      Fcxdd.Unity();

    Filter FtE;
    rtE.setZero();
    if(FTE==0)     FtE.Unity();
    else if(FTE>0) FtE.LowPass(rtE, hw_freq, FTE);
    else {
      // Butterworth 2nd-order notch, 45-65 Hz @500Hz
      VectorXd b(5),a(5);
      b[4] =  1.0;
      b[3] = -3.1065490382;
      b[2] =  4.4126617316;
      b[1] = -3.1065490382;
      b[0] =  1.0;
      a[4] =  0.7008967812;
      a[3] = -2.3681639156;
      a[2] =  3.6670730371;
      a[1] = -2.8327533240;
      a[0] =  1.0; // NOT USED
      FtE.Z_NumDen(rtE, b, a, rtE);  // NOTCH
    }

    Filter Fxd;
    rxd.setZero();
    if(FXD) Fxd.LowPass(rxd, hw_freq, FXD);
    else    Fxd.Unity();

    Filter FtqS;
    rtqS.setZero();
    if(FTQS) FtqS.LowPass(rtqS, hw_freq, FTQS);
    else     FtqS.Unity();
//     FtqS.Z_NumDen(rtqS, b, a, rtqS); // SAME AS FtE


// STABILIZE SERVO AMPS (n*HZ) seconds (KEEP WD ACTIVE)
    for(k=0; k<(1*HZ); ++k)
    {
      tq.setZero();
      veh->JointTq(tq);
      servo->tick();
    }

// INITIALIZE ANGLES & TIME
    veh->JointRad(q);
    servo->zero();

//////////////////////////////////////////
// THE LOOP
//////////////////////////////////////////
    while( mi->trajEnd_ > TRAJ_END_EXIT )
    {
    // WAIT FOR TICK
      tick_miss += servo->tick();
      if( tick_miss>9 )
      { cout << "tick miss = " >\<< servo->miss() << "!!!" << endl;
        tick_miss = 0;
      }
      cur_time = servo->time();

// BEGIN ODOMETRY SECTION

    // FIND AXIS ANGLES & SPEEDS
      veh->JointRad(q);   // FILLS ANGLES IN CASTER MODULES
      qd = Fqd.Filt(q);   // JOINT SPEEDS

    // FIND LOCAL OPERATIONAL SPEEDS
      veh->Fill_Jcp(J);       // NOTE: VIA CONTACT POINTS
      rxd = J * qd;           // RAW LOCAL OP SPEEDS
      x += rxd * hw_period;   // "LOCAL" COORDS
      xd = Fxd.Filt( rxd );   // LOCAL SPEED (ALSO FOR DYN)

    // DELTA: MAP LOCAL --> GLOBAL COORDS
      heading = gx[2]+ 0.5*rxd[2]*hw_period; // USE RAW_xd
      rot(0,0) =  cos(heading);
      rot(0,1) = -sin(heading);
      rot(1,0) =  sin(heading);
      rot(1,1) =  cos(heading);
      rgxd = rot * rxd;               // GET RAW_gxd

    // INTEGRATION TO GLOBAL COORDS
      gx += rgxd * hw_period;

      gxd = Fgxd.Filt(rgxd);     // FILTERED GLOBAL VELOCITY

// END ODOMETRY SECTION

// BEGIN CONTROL FORCE SECTION

        // GLOBAL CONTROL
      if( mi->ctrlFrame_ == CTRL_F_GLOBAL )
      {
//      cgxdd = (dx-gx)*mi->KP_ + (dxd-gxd)*mi->KV_; // SUBTRACTION IS SO +GAINs
        cgxdd[0] = (dx[0]-gx[0])*mi->KPx_ + (dxd[0]-gxd[0])*mi->KVx_;
        cgxdd[1] = (dx[1]-gx[1])*mi->KPx_ + (dxd[1]-gxd[1])*mi->KVx_;
        cgxdd[2] = (dx[2]-gx[2])*mi->KPa_ + (dxd[2]-gxd[2])*mi->KVa_;

        if( mi->trajMode_ == TRAJ_M_ACCEL )
          cgxdd.setZero();

        if( mi->accel_FF_ == true )
          cgxdd += dxdd;

        // GLOBAL OP SPACE ADJUSTMENTS (PUT HACKs HERE)
//         if( mi->trajMode_ == TRAJ_M_PACE )
//           cgxdd[2]=0;

        // MAP GLOBAL COMMAND TO LOCAL COORDS
        rot(0,1) *= -1.0;
        rot(1,0) *= -1.0;
        cxdd = rot * cgxdd;
      }
      else  // LOCAL CONTROL
      {
//      cxdd = (dx-x)*mi->KP_ + (dxd-xd)*mi->KV_; // SUBTRACTION IS SO +GAINs
        cxdd[0] = (dx[0]-x[0])*mi->KPx_ + (dxd[0]-xd[0])*mi->KVx_;
        cxdd[1] = (dx[1]-x[1])*mi->KPx_ + (dxd[1]-xd[1])*mi->KVx_;
        cxdd[2] = (dx[2]-x[2])*mi->KPa_ + (dxd[2]-xd[2])*mi->KVa_;

        if( mi->trajMode_ == TRAJ_M_ACCEL )
          cxdd.setZero();

        if( mi->accel_FF_ == true )
          cxdd += dxdd;

        // LOCAL OP SPACE ADJUSTMENTS (PUT HACKs HERE)
        //        cxdd[2]=0;
      }

    // FRICTION COMP
//      if( mi->fricMode_ == FRIC_BOHR )
//      { cxdd[2] += veh->FricBohr(qd,gxd[2],tq);
//      }
    // NULL MODE OVERRIDE (CLOBBER HERE, FOR SAFETY)
      if( mi->trajMode_ == TRAJ_M_NULL )
        cxdd.setZero();

      fcxdd = Fcxdd.Filt(cxdd);  // FILTER EXTERNAL TORQUES
      cxdd  = fcxdd;             // replace w/ filtered

// END CONTROL FORCE SECTION


// BEGIN OUTPUT TORQUE SECTION

    // DO DYNAMICS
      veh->Fill_C(C);         // USE CONSISTENT SPEEDS
      qd_bar = C * xd;        // IGNORE SLIP FOR DYNAMICS
      veh->Dyn(qd_bar, xd[2]);// FILL Lambda & Mu
      Lambda = veh->Lambda;
      Mu     = veh->Mu;

    // GET MAPPING OP FORCE -> JOINT TQ
      Jt = J.transpose(); // MOTOR TQ's w/ MIN CONTACT FORCES

      switch( mi->controlMode_ )
      {
      case CTRL_LM:
          cf = Lambda*cxdd;
          tq = Jt * (cf + Mu);
        break;

      case CTRL_L:
          cf = Lambda*cxdd;
          tq = Jt * cf;
        break;

      case CTRL_L_CONST: // USE AVERAGE MASS PROPERTIES
          cf[0] = veh->M_gross * cxdd[0];  // AVERAGE MASS
          cf[1] = veh->M_gross * cxdd[1];
          cf[2] = veh->I_gross * cxdd[2];  // AVERAGE INERTIA
          tq = Jt * cf ;
        break;

      case CTRL_TEST:    // BRUTE-FORCE CODE TO CLOBBER SOMETHING
          tq.setZero();
          tq[2] = 2.0;
        break;

      case CTRL_CALI:
        // HOLD WHEELS W/ CONTROLLER,
        // CALIBRATE W/ STRAIGHT-EDGE
        { Float jkp = 50;
          Float jkv =  1;
          Float cali_d = M_PI_2;
          tq.setZero();
          tq[0] = -jkp*cos(q[0]+ 2*cali_d) - jkv*qd[0];
          tq[2] = -jkp*cos(q[2]+ 3*cali_d) - jkv*qd[2];
          tq[4] = -jkp*cos(q[4]+ 0*cali_d) - jkv*qd[4];
          tq[6] = -jkp*cos(q[6]+ 1*cali_d) - jkv*qd[6];
        }
        break;

      default:
        cout << "PANIC: OUTPUT TORQUE" << endl;
        break;

      }  // end switch controlMode


    // ADD STEERING FRICTION COMPENSATION
//       fS = veh->Fill_tqS(qd,tq,tqS);
//       tq += Jt * fS;

    // ADD STEERING FRICTION COMPENSATION
      veh->Fill_tqS(qd,tq,rtqS);
      tqS = FtqS.Filt(rtqS);
      tq += tqS;

    // INTERNAL FORCE COMPUTATION
      if( mi->internalForce_ == true )
      {
        veh->Fill_E_q( E );
        // PROJECT TO 'E'-SPACE AND THEN BACK TO JT-SPACE
        qd_null = qd - qd_bar;  // NULL SPACE WHEEL SPEEDS
        rtE = E * qd_null  ;    // RAW SLIP (INTERNAL VELs)
        tE = FtE.Filt(rtE);     // FILTER INTERNAL VELs
        ctE = tE * (-mi->KpE_); // CONTROL FORCES to RESIST SLIP
        tqE * E.transpose() * ctE;   // MAP TO JOINT TORQUES
        tq += tqE;
      }

    // OUTPUT TORQUE TO JOINTS
      veh->JointTq(tq);

// extra for reporting more human-friendly units
veh->JtTq2MotAmp(tq,motAmp);
tqI = Jt *(Lambda*dxdd + Mu);


// END OUTPUT TORQUE SECTION

// BEGIN ACTION QUEUE
      if(  cur_time > mi->trajEnd_  ) //EXPIRED?
      { mi->Step();    // UPDATES mi VIA GLOBAL ITER
      }

      mi->Run();       // ALWAYS RUN TO UPDATE GOALS

        // SANITY CHECK
      while( !mi->Goal_is_valid() )
      { mi->Step();
        mi->Run();
      }

// END ACTION QUEUE
      if( mi->gathMode_ )
      { gObj.gather(cur_time);
        if( gObj.state() == GATHER_STATE_FULL )
          gObj.save();
      }

    } // end THE LOOP

} // end mymain


//---------------------------------------------------
//---------------------------------------------------
// NEEDS TO BE REPLACED.
// WAS IMPLEMENTED USING eCos OPERATING SYSTEM
// https://en.wikipedia.org/wiki/ECos
#define  STACKSIZE_MAIN  ( 1024 * 32 )
int stack_main[ STACKSIZE_MAIN/sizeof(int) ];

timer_handle_t  main_h;
timer_thread    main_obj;


// NEEDS TO BE REPLACED.
// WAS IMPLEMENTED USING eCos OPERATING SYSTEM
// https://en.wikipedia.org/wiki/ECos
void
timer_user_start( void )
{
  timer_thread_create( 3, mymain, 0, "mymain",
                     stack_main, STACKSIZE_MAIN,
                     &main_h, &main_obj);

  timer_thread_resume( main_h );
  Gather::tftp_server_start();
  HWInterface::HandleGet();
}


// NEEDS TO BE REPLACED.
// WAS IMPLEMENTED USING eCos OPERATING SYSTEM
// https://en.wikipedia.org/wiki/ECos
#if defined(redhat) || defined(timerwin)
int main(int argc, char **argv)
{
  timer_user_start();
  timer_thread_delay( 2000000 );
  return 0;
}
#endif

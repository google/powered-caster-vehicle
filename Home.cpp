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


#include <iostream>
#include <assert.h>

#include "Servo_Timer.h"
#include "Home.h"

#define IX_ACTIVE 1

//  HARDWARE OFFSETS FOR EACH CASTER MODULE
#define XRD_OS1 ( 1*6802 +  20)
#define XRD_OS2 ( 0*6802 +  85)
#define XRD_OS3 (-1*6802 +  75)
#define XRD_OS4 (-2*6802 +  80)

#define HZ_HOME  500.0

#define E_ERR   3000
#define V_DES   6000
#define ZERO_KP   2.0
#define ZERO_KV   0.04

using namespace std;

void
Home( void )
{
  Home( XRD_OS1, XRD_OS2, XRD_OS3, XRD_OS4 );
}


void
Home( long off1, long off2, long off3, long off4 )
{
  HWInterface* hwi = HWInterface::HandleGet();
  if( hwi->IsCalib() ) // IF CALIBRATED ALREADY, BAIL OUT
  { cout << "Home(): Already Calibrated" << endl;
    return;
  }
  else
  { cout << "Home(): Not Calibrated, Run Calibration..." << endl;
  }

  enum {INIT,STANDBY,READY,DONE} state;

  long e_des,e_now,e_old,e1;
  long tq;
  double dt;
  double v_now;

  ENC_LONGBYTE e[8];
  long e_offset[4];
  unsigned short axis;

  double hz = HZ_HOME;  // DEFAULT FREQ (IF FIRST CALLER)
  Servo_Timer* servo = Servo_Timer::HandleGet( hz );
  servo->freq( HZ_HOME );  // DON'T FORGER TO SET BACK!

  cout << "zeroing drive system...\n" << endl;

  for( axis=0; axis<8 ; axis++)
  { hwi->SetEncoderCounts( axis , 0x00000000 );
  }

  cout << "Starting zero(): calib=" << hwi->IsCalib() << endl;
  servo->zero();

  for( axis=0; axis<8 ; axis+=2)
  {
    hwi->SelectIndexAxis( axis, 0 );
    hwi->ResetIndexLatch();
    hwi->EncoderLatch();
    hwi->EncReadAll(e);
    e_des=e_old=e[axis];

    v_now=0;

    state = INIT;
    while( state!=DONE )
    {
      long tick_miss = servo->tick();
      assert( tick_miss == 0 );
      hwi->EncReadAll(e); e_now=e[axis];

      switch(state)
      {
      case INIT:
        if( ! hwi->IndexPulse() == IX_ACTIVE )
        { hwi->EncReadAll(e); e1=e[axis];
          state = STANDBY;
        }
        break;

      case STANDBY:
        if( ! hwi->IndexPulse() == IX_ACTIVE )
        { if( e_now - e1 > 800 )
          { hwi->ResetIndexLatch();
            state = READY;
          }
        }
        else
          state = INIT;
        break;

      case READY:
        if( hwi->IndexPulseLatch() == IX_ACTIVE )
        { hwi->EncReadAll(e); e1=e[axis];
          e_offset[axis/2] = (e_now+e1)/2L;
          state = DONE;
        }
        break;

      case DONE:
        break;    // DO NOTHING

      } //case

      dt = 1.0/HZ_HOME;
      if ( dt != 0.0 )
      { e_des += (long)(V_DES*dt);
        if(e_des-e_now>E_ERR) e_des = e_now+E_ERR;
        v_now = (e_now-e_old)/dt;
        tq = (long)(-ZERO_KP*(e_now-e_des))-(long)(ZERO_KV*v_now);
        hwi->RawDAC( axis, -tq );
        e_old = e_now;
      }

    } // while

    hwi->RawDAC(axis,0);
  } // for axis

  e_offset[0] += off1;
  e_offset[1] += off2;
  e_offset[2] += off3;
  e_offset[3] += off4;


  cout << "encoders       [0]      [2]      [4]      [6]" << endl;
  cout << "Offsets: " <<e_offset[0]<<" "<<e_offset[1]<<" "
                      <<e_offset[2]<<" "<<e_offset[3]<<endl;

  servo->delay_sec( 2 ); // IT's OK, but WD ZERO's DACs HERE

  // WRITE OFFSETS TO STEER REGISTERS SO GET zero at theta=0;
  cout << "Presets: ";
  int c;
  for( axis=0; axis<8 ; axis+=2)
  {
    state = STANDBY;
    c = 0;

    while( state!=DONE )
    {
      hwi->EncReadAll(e);
      e_now=e[axis];

      if( e_now == e_old ) //MAKE SURE ITS NOT MOVING!
        c++;
      else
        c=0;

      e_old = e_now;

      if( c > 1000 )
      { state = DONE;
      }
    } // while state

    hwi->SetEncoderCounts( axis , e_now-e_offset[axis/2] );
    cout << " " << e_now-e_offset[axis/2];
  } // for axis
  cout << endl;

  hwi->IsCalib(true);  // SET CALIB SIGNATURE
  servo->freq( hz );   // SET BACK TO ORIGINAL FREQ

  hwi->EncoderLatch();
  hwi->EncReadAll(e);
  cout << "Current: " <<e[0]<<" "<< e[2]<<" "<<e[4]<<" "<<e[6]<< endl;

  cout << "Finished zero(): calib=" <<  hwi->IsCalib() << endl;
  return;
}

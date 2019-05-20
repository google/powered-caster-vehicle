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
#include <cstring>

#include <Eigen/Dense>

#include "Gather.h"

using namespace std;
using namespace Eigen;


Gather::Gather( char const *filename, int const num_samples )
  :numC_(0),numD_(0),gatherCount_(0)
{
  zero_();
  reset( filename, num_samples );

  /* NEED TO IMPLEMENT A MODERN THREAD MANAGEMENT...

  // CREATE STACK FOR save() THREAD
  int   SAVETHREAD_STACKSIZE = ( 1024 * 32 );
  SaveThread_Stack_ = calloc(SAVETHREAD_STACKSIZE, sizeof(char));
  */
}

void
Gather::filename_set_(char const *filename)
{
  strcpy(filename_,filename);
  if( strlen(filename_)>FILENAME_MAX-4 ) // SAVE ROOM FOR FILE EXTENTION
  { filename_[FILENAME_MAX-5] = 0;
  }
  strcat(filename_,".csv");
}

void
Gather::zero_()
{
  numC_ = 0;
  numD_ = 0;

  filename_[0] = 0;
  max_samples_ = 0;
  gatherCount_ = 0;

  state_=GATHER_STATE_EMPTY;
}


void
Gather::reset()
{
  reset( filename_, max_samples_ );
}

void
Gather::reset( char const *filename, int const num_samples )
{
  freeData_();
  zero_();
  filename_set_(filename);
  max_samples_ = num_samples;
  data("t",t_);
}


void
Gather::constant(char const *name, Float const &dataIn)
{
  if( &dataIn == NULL )
  { cout << "Gather: '" << name << "' is NULL pointer" << endl;
    return;
  }
  strcpy(C_[numC_].name,name);
  C_[numC_].dataIn = NULL; // CONSTANT, so NOW not LATER
  C_[numC_].row  = 1;
  C_[numC_].col  = 1;
  C_[numC_].data = new Float[C_[numC_].row];

  int i;
  for( i=0 ; i<C_[numC_].row ; i++ )
    C_[numC_].data[i] =  dataIn;

  ++numC_;
}


void
Gather::constant(char const *name, VectorXd const &dataIn)
{
  if( &dataIn == NULL )
  { cout << "Gather: '"<< name <<"' is NULL pointer" << endl;
    return;
  }
  strcpy(C_[numC_].name,name);
  C_[numC_].dataIn = NULL; // CONSTANT, so NOW not LATER
  C_[numC_].row  = dataIn.size();
  C_[numC_].col  = 1;
  C_[numC_].data = new Float[C_[numC_].row];

  int i;
  for( i=0 ; i<C_[numC_].row ; i++ )
    C_[numC_].data[i] =  dataIn[i];

  ++numC_;
}

void
Gather::constant(char const *name, MatrixXd const &dataIn)
{
  if( &dataIn == NULL )
  { cout << "Gather: '"<< name <<"' is NULL pointer" << endl;
    return;
  }
  strcpy(C_[numC_].name,name);
  C_[numC_].dataIn = NULL; // CONSTANT, so NOW not LATER
  C_[numC_].row  = dataIn.row();
  C_[numC_].col  = dataIn.column();
  C_[numC_].data = new Float[C_[numC_].row * C_[numC_].col];

  for(int i=0 ; i<(C_[numC_].row*C_[numC_].col) ; ++i)
  { C_[numC_].data[i] = dataIn.data()[i];
  }

  ++numC_;
}



void
Gather::data(char const *name, Float const &dataIn)
{
  if( &dataIn == NULL )
  { cout << "Gather: '"<< name <<"' is NULL pointer" << endl;
    return;
  }
  strcpy(D_[numD_].name,name);
  D_[numD_].dataIn = (Float *)&dataIn;
  D_[numD_].row = 1;
  D_[numD_].col = 1;
  D_[numD_].data = new Float[max_samples_];
  ++numD_;
}


void
Gather::data(char const *name, VectorXd const &dataIn)
{
  if( &dataIn == NULL )
  { cout << "Gather: '"<< name <<"' is NULL pointer" << endl;
    return;
  }
  strcpy(D_[numD_].name,name);
  D_[numD_].dataIn = dataIn.data();
  D_[numD_].row = dataIn.size();
  D_[numD_].col = 1;
  D_[numD_].data = new Float[max_samples_ * D_[numD_].row];
  ++numD_;
}


void
Gather::data(char const *name, MatrixXd const &dataIn)
{
  if( &dataIn == NULL )
  { cout << "Gather: '"<< name <<"' is NULL pointer" << endl;
    return;
  }
  strcpy(D_[numD_].name,name);
  D_[numD_].dataIn = dataIn.data();
  D_[numD_].row = dataIn.row();
  D_[numD_].col = dataIn.column();
  D_[numD_].data = new Float[max_samples_ *
                            D_[numD_].row * D_[numD_].col];
  ++numD_;
}


Gather::~Gather()
{
  freeData_();
  /* NEED TO IMPLEMENT A MODERN WAY TO SAVE VIA A THREAD
 free(SaveThread_Stack_);
  */
}


void
Gather::freeData_()
{
  int i;

  for(i=0 ; i<numC_ ; ++i)
  { delete[] C_[i].data;
    C_[i].data = NULL;
  }
  numC_ = 0;

  for(i=0 ; i<numD_ ; ++i)
  { delete[] D_[i].data;
    D_[i].data = NULL;
  }
  numD_ = 0;
}


void
Gather::gather(double const time)
{
  int i,k;

  switch(state_)
  {
    case GATHER_STATE_FULL:   // DO NOTHING WHEN gather() IS CALLED
    case GATHER_STATE_SAVING: // DO NOTHING WHEN gather() IS CALLED
    case GATHER_STATE_WRITTEN:// DO NOTHING WHEN gather() IS CALLED
    case GATHER_STATE_ERROR:  // DO NOTHING WHEN gather() IS CALLED
    break;

    case GATHER_STATE_EMPTY:
      state_ = GATHER_STATE_READY; // NOTE: NO break; CONTINUE AND GATHER...
    case GATHER_STATE_READY:
      // GATHER TIME  (TIME IS FIRST DATA)
      t_ = time;

      // GATHER DATA
      for(k=0 ; k<numD_ ; ++k)
      { int vsize = D_[k].row * D_[k].col;
        for(i=0 ; i<vsize ; ++i)
        { D_[k].data[gatherCount_*vsize+i] = D_[k].dataIn[i];
        }
      }
      // CHECK IF DONE GATHERING
      if ( ++gatherCount_ >= max_samples_)
      { state_ = GATHER_STATE_FULL;
      }
      break;

    default:
      cout << "error: unknown Gather() state =  " << state_ << endl;
      state_ = GATHER_STATE_ERROR;
      break;
  }
}


void
Gather::save()
{
  // DON'T SAVE WHEN EMPTY, DON'T SAVE MORE THAN ONCE
  if( state_ == GATHER_STATE_READY || state_ == GATHER_STATE_FULL )
  {
    // MAKE SURE ONLY ONE THREAD STARTS
    state_ = GATHER_STATE_SAVING;

    /*  NEED TO IMPLEMENT A MODERN WAY TO SAVE USING A TREAD...
    static timer_handle_t  saveThread_h;
    static timer_thread    saveThread_obj;

    timer_thread_create(PRIORITY_GATHER,
                      saveThread_,
                      (unsigned int) this,
                      "saveThread",
                      SaveThread_Stack_,
                      SAVETHREAD_STACKSIZE,
                      &saveThread_h,
                      &saveThread_obj );

    timer_thread_resume( saveThread_h );
    */
  }
}



#define MAX_FILE_LINE_LEN  ( (10+2)*(40*6 + 25) ) // DIGITS*(VECTORS*SIZE+CONSTANTS)
#define FMT  ",%g"

#ifdef ecos
#include <tftp_support.h>    //USE SIMPLE ECOS FILESYSTEM
#endif

void saveThread_(unsigned int obj)
{
  Gather* gObj = (Gather*)obj;
  static char s[MAX_FILE_LINE_LEN+1];
  int slen;
  char tok[30];  // LARGE ENOUGH FOR A COMMA AND ONE DIGIT
  T_Data  *v;    // SHORTHAND to SAVE ME FROM TYPOs

  // OPEN FILE
#ifdef ecos
  extern struct tftpd_fileops dummy_fileops;
  int fd;
  fd = dummy_fileops.open( gObj->filename_, O_WRONLY );
#else
  FILE *file;
  file = fopen( gObj->filename_, "w" );
#endif

    s[0]=0;
    slen=0;

    // WRITE DATA LABELS
  for(int k=0 ; k<gObj->numD_ ; ++k)
  { v = &gObj->D_[k];
    char *r_fmt = "_%1d";char *c_fmt = "%1d";
    if( v->row>10 || v->col>10)
    { r_fmt = "_%02d"; c_fmt = "_%02d"; }
    for(int kr=0 ; kr<v->row ; ++kr)
      for(int kc=0 ; kc<v->col ; ++kc)
      { slen += sprintf( tok, ",%s", v->name );
        strcat( s, tok );       // NAME
        if( v->row > 1 )
        { slen += sprintf( tok, r_fmt, kr+1 );
          strcat( s, tok );   // ROW if VECTOR or MATRIX
        }
        if( v->col > 1 )
        { slen += sprintf( tok, c_fmt, kc+1 );
          strcat( s, tok );   // COL if MATRIX
        }
      }
  }

    // WRITE CONSTANT LABELS
  for(int k=0 ; k<gObj->numC_ ; ++k)
  { v = &gObj->C_[k];
    char *r_fmt = "_%1d";char *c_fmt = "%1d";
    if( v->row>10 || v->col>10)
    { r_fmt = "_%02d"; c_fmt = "_%02d"; }
    for(int kr=0 ; kr<v->row ; ++kr)
      for(int kc=0 ; kc<v->col ; ++kc)
      { slen += sprintf( tok, ",%s", v->name );
        strcat( s, tok );       // NAME
        if( v->row > 1 )
        { slen += sprintf( tok, r_fmt, kr+1 );
          strcat( s, tok );   // ROW if VECTOR or MATRIX
        }
          if( v->col > 1 )
        { slen += sprintf( tok, c_fmt, kc+1 );
          strcat( s, tok );   // COL if MATRIX
        }
      }
  }

  slen++;
  strcat( s, "\n" );     // END OF LINE

  // SAVE LINE TO FILE (skip first comma)
#ifdef ecos
  dummy_fileops.write( fd, s+1, slen-1 );
#else
  fputs ( s+1, file );
#endif

  // WRITE VALUES
  for(int idx=0; idx<gObj->gatherCount_ ; ++idx)
  {
    s[0]=0;
    slen=0;

      // WRITE DATA
    for(int k=0 ; k<gObj->numD_ ; ++k)
    { v = &gObj->D_[k];
      for(int kr=0 ; kr<v->row ; ++kr)
        for(int kc=0 ; kc<v->col ; ++kc)
        { slen += sprintf( tok, FMT ,
            v->data[idx*v->row*v->col + kr*v->col + kc ] );
          strcat( s, tok );
        }
    }
      // WRITE CONSTANTS
  if( idx == 0 )     // (ON THE FIRST PASS THRU ONLY)
  { for(int k=0 ; k<gObj->numC_ ; ++k)
    { v = &gObj->C_[k];
      for(int kr=0 ; kr<v->row ; ++kr)
        for(int kc=0 ; kc<v->col ; ++kc)
        { slen += sprintf( tok, FMT,
            v->data[idx*v->row*v->col + kr*v->col + kc ] );
          strcat( s, tok );
        }
    }
  } // CONSTANTS ONLY FOR idx==0

    slen++;
    strcat( s, "\n" );     // END OF LINE
    assert( slen < MAX_FILE_LINE_LEN ); // BAIL IF NOT LONG ENOUGH

    // SAVE LINE TO FILE (skip first comma)
#ifdef ecos
    dummy_fileops.write( fd, s+1, slen-1 );
#else
    fputs ( s+1, file );
#endif

  } // THE LOOP idx

  // CLOSE FILE
#ifdef ecos
  dummy_fileops.close( fd );
#else
  fclose( file );
#endif

  cout << "Save end:  " << gObj->gatherCount_ <<
          "samples written to '" <<  gObj->filename_  << "'." << endl;

  gObj->state_ = GATHER_STATE_WRITTEN;

#ifdef ecos
  timer_thread_exit();
#endif

  return;
} // END: SaveThread


// SINGLETON
static Gather *sp_gObj = NULL;

Gather *
Gather::HandleGet( char const *filename, int const max_num_samples )
{
  if( !sp_gObj ) // ONLY USE ARGS ON CREATION
  {
    sp_gObj = new Gather( filename, max_num_samples );
  }
                // IGNORE ARGS IF SINGLETON EXISTS
  return sp_gObj;
}


#ifdef ecos //*********  tftp _server_start ********//

#include <network.h>
#include <tftp_support.h>

void
Gather::tftp_server_start(void)
{
  extern struct tftpd_fileops dummy_fileops;
  int tftp_id;

  init_all_network_interfaces();
#ifdef  CYGHWR_NET_DRIVER_ETH0
  if( eth0_up )
  { tftp_id = tftpd_start(69, &dummy_fileops);
    cout << "start tftp server 0x" << tftp_id << endl; //TODO: output hex
    if( tftp_id <= 0 )
    { cout << "Failed to start tftp server!" << endl;
      return;
    }
  }
  else
  { cout << "Failed to bring up interface eth0!" << endl;
    return;
  }
#else
  cout << "Failed to define interface eth0!" << endl;
#endif //CYGHWR_NET_DRIVER_ETH0

  return;
}

#else        //***** DUMMY tftp _server_start  *****//

void
Gather::tftp_server_start(void)
{
  cout << "\nGather::tftp_server_start:  NOT IMPLEMENTED !!!" << endl;
  cout << "Gathered files will be saved in native filesystem instead.\n" << endl;
}

#endif // ecos

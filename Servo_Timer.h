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

#ifndef _Servo_Timer_h_
#define _Servo_Timer_h_


class HWInterface;

class Servo_Timer
{
public:
   // SETS FREQ IF FIRST CALL
   // DOES NOT SET FREQ IF ALREADY SET
   // EITHER WAY, SETS hz TO EXACT FREQ OF TIMER
  static Servo_Timer* HandleGet( double &hz );
   // USE THIS TO START USING CONSTANT (NO FREQ RETURNED)
  static Servo_Timer* HandleGet( const int hz = 100 );

   ~Servo_Timer();          // DESTRUCTOR

  double freq( double hz ); // SETS FREQ & RETURNS EXACT SERVO FREQUENCY
  double freq();            // RETURNS EXACT SERVO FREQUENCY

  void   zero();  // ZERO TICK COUNTER VALUE

  long  count();  // RETURN TICK COUNTER VALUE
  double time();  // RETURN TIME VALUE

  long   tick();  // WAIT FOR NEXT TICK
                  // RETURN NUMBER OF MISSED TICKS

  long   miss();  // RETURN TOTAL MISSED TICKS SINCE zero()

  void   delay_sec( double sec ); // PAUSE FOR SECONDS
  void   delay_min( double min ); // PAUSE FOR MINUTES

private:

  double hz_;

  long   tick_last_;  // LAST TIME tick() WAS CALLED
  long   tick_intr_;  // NUMBER OF INTERRUPTS SINCE tick()
  long   tick_miss_;  // TOTAL MISSED TICKS SINCE zero()

  HWInterface*      hwi_;    // POINTER TO TIMER HARDWARE

  timer_handle_t    int_h_;    // HANDLE TO INTERRUPT
  timer_interrupt   int_obj_;  // STORAGE FOR INTERRUPT OBJECT

  timer_sem_t       sem_;   // STORAGE FOR INTERRUPT SEMAPHORE

  Servo_Timer( double hz );  // CONSTRUCTOR
  bool  IsValid() const;

};

#endif // _Servo_Timer_h_

/*
  spindle_control.h - DSI spindle control methods
  Part of Grbl - Modified for use with RS-485 VFD

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud
  Copyright (c) 2018-2019 Stephen Culpepper

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef spindle_control_h
#define spindle_control_h

#define SPINDLE_NO_SYNC false
#define SPINDLE_FORCE_SYNC true

#define SPINDLE_STATE_DISABLE  0  // Must be zero.
#define SPINDLE_STATE_CW       1
#define SPINDLE_STATE_CCW      2




// Initializes spindle pins and hardware PWM, if enabled.
void spindle_init();

void spindle_initDSI();

void spindle_run_init();
void spindle_reset_init();
void spindle_rts_init();

void spindle_counter_tick();

// Returns current spindle output state. Overrides may alter it from programmed states.
int spindle_get_state();

// Called by g-code parser when setting spindle state and requires a buffer sync.
// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by spindle_sync() after sync and parking motion/spindle stop override during restore.
#ifdef VARIABLE_SPINDLE

  // Called by g-code parser when setting spindle state and requires a buffer sync.
  void spindle_sync(int state, float rpm);

  // Sets spindle running state with direction, enable, and spindle PWM.
  void spindle_set_state(int state, float rpm);
  
  // Sets spindle hz quickly for stepper ISR. Also called by spindle_set_state().
  // NOTE: PF525 setpoint is 16bit, Hz x 100.
  void spindle_set_speed(int hz_100_value);
  
  // Computes Hz register value for the given RPM for quick updating.
  int spindle_compute_hz_value(float rpm);
  
  void spindle_update();

  int ModWriteSingleBlocking(unsigned char node, uint16_t address, uint16_t value);
  // Read function not yet implemented
  int ModReadSingleBlocking(unsigned char node, uint16_t address);

  //async method not yet implemented
  int ModWriteSingle(unsigned char node, uint16_t address, uint16_t value);
  //async method not yet implemented
  int ModReadSingle(unsigned char node, uint16_t address);

  int ModReadbackSingle(unsigned char node);

#else
  
  // Called by g-code parser when setting spindle state and requires a buffer sync.
  #define spindle_sync(state, rpm) _spindle_sync(state)
  void _spindle_sync(int state);

  // Sets spindle running state with direction and enable.
  #define spindle_set_state(state, rpm) _spindle_set_state(state)
  void _spindle_set_state(int state);

#endif

// Stop and start spindle routines. Called by all spindle routines and stepper ISR.
void spindle_stop();


#endif

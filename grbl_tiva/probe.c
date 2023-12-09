/*
  probe.c - code pertaining to probing methods
  Part of Grbl

  Copyright (c) 2014-2016 Sungeun K. Jeon for Gnea Research LLC

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

#include "grbl.h"


// Inverts the probe pin state depending on user settings and probing cycle mode.
bool probe_invert;


// Probe pin initialization routine.
void probe_init()
{
  SysCtlPeripheralEnable(PROBE_PORT);
  SysCtlPeripheralEnable(TOOL_SETTER_PORT);
	GPIODirModeSet(PROBE_BASE, PROBE_PIN, GPIO_DIR_MODE_IN); // Configure as input pin
	GPIODirModeSet(TOOL_SETTER_BASE, TOOL_SETTER_PIN, GPIO_DIR_MODE_IN); // Configure as input pin
#ifdef DISABLE_PROBE_PIN_PULL_UP
	// Weak pull-down for normal low operation. Pin is 0 state unless pulled high.
  // Weak pull-down should be 13k or greater, but may be as low as 10k ohm
	GPIOPadConfigSet(PROBE_BASE, PROBE_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
	GPIOPadConfigSet(TOOL_SETTER_BASE, TOOL_SETTER_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
#else
	// Enable internal pull-up resistors. Normal high operation.
	GPIOPadConfigSet(PROBE_BASE, PROBE_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
	GPIOPadConfigSet(TOOL_SETTER_BASE, TOOL_SETTER_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
#endif
	probe_configure_invert_mask(false); // Initialize invert mask.
}


// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
void probe_configure_invert_mask(uint8_t is_probe_away)
{
  probe_invert = false; // Initialize as zero.
  // Default probe to : probe_invert is true
  if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert = !probe_invert; }
  if (is_probe_away) { probe_invert = !probe_invert; }
}


// TODO: Add error checking for toolsetter when probing, probe when toolsetting 
// The functionality should Be
// G38.2 / .3 / .4 / .5 / Should use the probe input and ignore the tool setter
// Ideally, probing should record the trigger point, accel to stop, and move back to tirgger point
// This would be the ideal behavior for limits and homing too
// The tool setter shoud be used with G100 
// G100 should move to Z+ Limit, Move to the given X and Y position, and then move Z- at the given feed rate
// The behavior should just be a probing cycle to z-
// If the probe triggers before the tool setter, it should generate an error
//
// Returns the probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
//uint8_t probe_get_state() { return((PROBE_DATA & PROBE_PIN) ^ probe_invert_mask); }
bool probe_get_state() 
{ 
  // Default, probe to: probe_invert is true
  // Default, probe away: probe_invert is false
  // Inverted, probe to: probe_invert is false
  // Inverted, probe away: probe_invert is true
  // AND = both sensors clear 
  bool probe_state = (GPIOPinRead(PROBE_BASE, PROBE_PIN) > 0) && (GPIOPinRead(TOOL_SETTER_BASE, TOOL_SETTER_PIN) > 0);

  // Temporary hack to allow only the tool setter pin to be used
  // bool probe_state = (GPIOPinRead(TOOL_SETTER_BASE, TOOL_SETTER_PIN) > 0);
  // End of hack

  if(probe_invert) return !probe_state;
  else return probe_state;
}


// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
// NOTE: This function must be extremely efficient as to not bog down the stepper ISR.
void probe_state_monitor()
{
  if (probe_get_state()) {
    sys_probe_state = PROBE_OFF;
    memcpy(sys_probe_position, sys_position, sizeof(sys_position));
    bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
  }
}

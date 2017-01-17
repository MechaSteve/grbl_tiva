/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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


#ifdef VARIABLE_SPINDLE
  static float pwm_gradient; // Precalulated value to speed up rpm to PWM conversions.
#endif


void spindle_init()
{
	#ifdef VARIABLE_SPINDLE
	// Configure variable spindle
	// PWM Scaling Factor
	pwm_gradient = SPINDLE_PWM_RANGE/(settings.rpm_max-settings.rpm_min);

	#else
	// Configure no variable spindle and only enable and direction pin.
	OutputSpindleEnableInit();
	OutputSpindleDirectionInit();
	#endif
	spindle_stop();
}


uint8_t spindle_get_state()
{
#ifdef VARIABLE_SPINDLE
	// Do something different?
#else //VARIABLE_SPINDLE

	if(OutputSpindleEnableRead())
	{
		if(OutputSpindleDirectionIsCW) return (SPINDLE_STATE_CW);
		else return (SPINDLE_STATE_CCW);
	}


	return(SPINDLE_STATE_DISABLE);
#endif // VARIABLE_SPINDLE
}


// Disables the spindle and sets PWM output to zero when PWM variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
void spindle_stop()
{
#ifdef VARIABLE_SPINDLE
	// no reason to set pwm to zero if we are using enable
	OutputSpindleEnableOff();
#else
	OutputSpindleEnableOff();
#endif
}


//TODO: Update these methods for PWM output or FM output
#ifdef VARIABLE_SPINDLE
  // Sets spindle speed PWM output and enable pin, if configured. Called by spindle_set_state()
  // and stepper ISR. Keep routine small and efficient.
  void spindle_set_speed(uint8_t pwm_value)
  {
    if (pwm_value == SPINDLE_PWM_OFF_VALUE) {
      spindle_stop();
    } else {
      SPINDLE_OCR_REGISTER = pwm_value; // Set PWM output level.
      SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.

      #if defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
        #ifdef INVERT_SPINDLE_ENABLE_PIN
          SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
        #else
          SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
        #endif
      #endif
    }
  }


  // Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
  uint8_t spindle_compute_pwm_value(float rpm) // 328p PWM register is 8-bit.
  {
    uint8_t pwm_value;
    rpm *= (0.010*sys.spindle_speed_ovr); // Scale by spindle speed override value.
    // Calculate PWM register value based on rpm max/min settings and programmed rpm.
    if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
      // No PWM range possible. Set simple on/off spindle control pin state.
      sys.spindle_speed = settings.rpm_max;
      pwm_value = SPINDLE_PWM_MAX_VALUE;
    } else if (rpm <= settings.rpm_min) {
      if (rpm == 0.0) { // S0 disables spindle
        sys.spindle_speed = 0.0;
        pwm_value = SPINDLE_PWM_OFF_VALUE;
      } else { // Set minimum PWM output
        sys.spindle_speed = settings.rpm_min;
        pwm_value = SPINDLE_PWM_MIN_VALUE;
      }
    } else { 
      // Compute intermediate PWM value with linear spindle speed model.
      // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
      sys.spindle_speed = rpm;
      pwm_value = floor((rpm-settings.rpm_min)*pwm_gradient) + SPINDLE_PWM_MIN_VALUE;
    }
    return(pwm_value);
  }
#endif


// Immediately sets spindle running state with direction and spindle rpm via PWM, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
#ifdef VARIABLE_SPINDLE
  void spindle_set_state(uint8_t state, float rpm)
#else
  void _spindle_set_state(uint8_t state)
#endif
{
	if (sys.abort) { return; } // Block during abort.
	if (state == SPINDLE_DISABLE)
	{
		// Halt or set spindle direction and rpm.
#ifdef VARIABLE_SPINDLE
		sys.spindle_speed = 0.0;
#endif //VARIABLE_SPINDLE
		spindle_stop();
	}
	else
	{
#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
		if (state == SPINDLE_ENABLE_CW) OutputSpindleDirectionCW();
		else  OutputSpindleDirectionCCW();
#endif // USE_SPINDLE_DIR_AS_ENABLE_PIN

#ifdef VARIABLE_SPINDLE
		// NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
		if (settings.flags & BITFLAG_LASER_MODE) {
		if (state == SPINDLE_ENABLE_CCW) { rpm = 0.0; } // TODO: May need to be rpm_min*(100/MAX_SPINDLE_SPEED_OVERRIDE);
		}
		//spindle_set_speed(spindle_compute_pwm_value(rpm));
		//spindle enable should still be used with variable speed (particularly with direction changes)
		OutputSpindleEnableOn();
#else // VARIABLE_SPINDLE
		// NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
		// if the spindle speed value is zero, as its ignored anyhow.
		OutputSpindleEnableOn();
#endif // VARIABLE_SPINDLE

	}

	sys.report_ovr_counter = 0; // Set to report change immediately
}


// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails 
// if an abort or check-mode is active.
#ifdef VARIABLE_SPINDLE
  void spindle_sync(uint8_t state, float rpm)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    spindle_set_state(state,rpm);
  }
#else
  void _spindle_sync(uint8_t state)
  {
    if (sys.state == STATE_CHECK_MODE) { return; }
    protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
    _spindle_set_state(state);
  }
#endif

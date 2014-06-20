/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file state_machine_helper.cpp
 * State machine helper functions implementations
 */

#include "state_machine_helper.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

bool isThrustCmdZero(struct offboard_control_setpoint_s offboard_sp)
{
  if (offboard_sp.p4 <= 0.1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool isOffboardCmdTimeValid(hrt_abstime time_last_offboard_cmd_received)
{
  if ((hrt_absolute_time() - time_last_offboard_cmd_received) < 50000)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void updateBatteryStateMachine(battery_states_t &battery_state, float battery_voltage)
{
  if (battery_voltage < 0.0f && battery_state != INVALID)
  {
    battery_state = INVALID;
    set_tune(TONE_STOP_TUNE); // No alarm needed if there is no battery connected
    return;
  }

  switch (battery_state)
  {
    case GOOD:
      // If battery voltage is below critical threshold switch to LOW
      if (battery_voltage <= BATTERY_LOW_THRES)
      {
        battery_state = LOW;
        set_tune(TONE_BATTERY_WARNING_SLOW_TUNE);
      }
      break;
    case LOW:
      // If battery voltage is above good threshold switch to GOOD and stop buzzer
      if (battery_voltage > BATTERY_GOOD_THRES)
      {
        battery_state = GOOD;
        set_tune(TONE_STOP_TUNE);
      }
      // If battery voltage is below critical threshold switch to CRITICAL
      if (battery_voltage <= BATTERY_CRITICAL_THRES)
      {
        battery_state = CRITICAL;
        set_tune(TONE_BATTERY_WARNING_FAST_TUNE);
      }
      break;
    case CRITICAL:
      // If battery voltage is above critical threshold switch to LOW
      if (battery_voltage > BATTERY_CRITICAL_THRES)
      {
        battery_state = LOW;
        set_tune(TONE_BATTERY_WARNING_SLOW_TUNE);
      }
      break;
    case INVALID:
      if (battery_voltage > BATTERY_GOOD_THRES)
      {
        battery_state = GOOD;
        set_tune(TONE_STOP_TUNE);
      }
      else if (battery_voltage > BATTERY_LOW_THRES)
      {
        battery_state = LOW;
        set_tune(TONE_BATTERY_WARNING_SLOW_TUNE);
      }
      else if (battery_voltage > 0.0f)
      {
        battery_state = CRITICAL;
        set_tune(TONE_BATTERY_WARNING_FAST_TUNE);
      }
      break;
  }
}


//        } else if (status.battery_warning == VEHICLE_BATTERY_WARNING_CRITICAL) {
//                /* play tune on battery critical */
//                set_tune(TONE_BATTERY_WARNING_FAST_TUNE);
//
//        } else if (status.battery_warning == VEHICLE_BATTERY_WARNING_LOW || status.failsafe_state != FAILSAFE_STATE_NORMAL) {
//                /* play tune on battery warning or failsafe */
//                set_tune(TONE_BATTERY_WARNING_SLOW_TUNE);
//
//        } else {
//                set_tune(TONE_STOP_TUNE);
//        }

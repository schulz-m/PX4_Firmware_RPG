/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: Thomas Gubler <thomasgubler@student.ethz.ch>
 *           Julian Oes <joes@student.ethz.ch>
 *           Anton Babushkin <anton.babushkin@me.com>
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
 * @file commander_helper.cpp
 * Commander helper functions implementations
 */

#include "buzzer_helper.h"

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

#define BLINK_MSG_TIME	700000	// 3 fast blinks

static int buzzer = -1;
static hrt_abstime tune_end = 0; // end time of currently played tune, 0 for repeating tunes or silence
static int tune_current = TONE_STOP_TUNE; // currently playing tune, can be interrupted after tune_end
static unsigned int tune_durations[TONE_NUMBER_OF_TUNES];

int buzzer_init()
{
  tune_end = 0;
  tune_current = 0;
  memset(tune_durations, 0, sizeof(tune_durations));
  tune_durations[TONE_NOTIFY_POSITIVE_TUNE] = 800000;
  tune_durations[TONE_NOTIFY_NEGATIVE_TUNE] = 900000;
  tune_durations[TONE_NOTIFY_NEUTRAL_TUNE] = 500000;
  tune_durations[TONE_ARMING_WARNING_TUNE] = 3000000;

  buzzer = open(TONEALARM_DEVICE_PATH, O_WRONLY);

  if (buzzer < 0)
  {
    warnx("Buzzer: open fail\n");
    return ERROR;
  }

  return OK;
}

void buzzer_deinit()
{
  close(buzzer);
}

void set_tune(int tune)
{
  unsigned int new_tune_duration = tune_durations[tune];

  /* don't interrupt currently playing non-repeating tune by repeating */
  if (tune_end == 0 || new_tune_duration != 0 || hrt_absolute_time() > tune_end)
  {
    /* allow interrupting current non-repeating tune by the same tune */
    if (tune != tune_current || new_tune_duration != 0)
    {
      ioctl(buzzer, TONE_SET_ALARM, tune);
    }

    tune_current = tune;

    if (new_tune_duration != 0)
    {
      tune_end = hrt_absolute_time() + new_tune_duration;

    }
    else
    {
      tune_end = 0;
    }
  }
}

/**
 * Blink green LED and play positive tune (if use_buzzer == true).
 */
void tune_positive(bool use_buzzer)
{
  if (use_buzzer)
  {
    set_tune(TONE_NOTIFY_POSITIVE_TUNE);
  }
}

/**
 * Blink white LED and play neutral tune (if use_buzzer == true).
 */
void tune_neutral(bool use_buzzer)
{
  if (use_buzzer)
  {
    set_tune(TONE_NOTIFY_NEUTRAL_TUNE);
  }
}

/**
 * Blink red LED and play negative tune (if use_buzzer == true).
 */
void tune_negative(bool use_buzzer)
{
  if (use_buzzer)
  {
    set_tune(TONE_NOTIFY_NEGATIVE_TUNE);
  }
}

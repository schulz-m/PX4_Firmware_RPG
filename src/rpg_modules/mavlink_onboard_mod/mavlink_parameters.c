/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file mavlink_parameters.c
 * MAVLink parameter protocol implementation (BSD-relicensed).
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include "mavlink_bridge_header.h"
#include "mavlink_parameters.h"
#include <uORB/uORB.h>
#include "math.h" /* isinf / isnan checks */
#include <assert.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <sys/stat.h>

extern mavlink_system_t mavlink_system;

extern int mavlink_missionlib_send_message(mavlink_message_t *msg);
extern int mavlink_missionlib_send_gcs_string(const char *string);

/**
 * If the queue index is not at 0, the queue sending
 * logic will send parameters from the current index
 * to len - 1, the end of the param list.
 */
static unsigned int mavlink_param_queue_index = 0;

/**
 * Callback for param interface.
 */
void mavlink_pm_callback(void *arg, param_t param);

void mavlink_pm_callback(void *arg, param_t param)
{
	mavlink_pm_send_param(param);
	usleep(*(unsigned int *)arg);
}

int mavlink_pm_queued_send()
{
	if (mavlink_param_queue_index < param_count()) {
		mavlink_pm_send_param(param_for_index(mavlink_param_queue_index));
		mavlink_param_queue_index++;
		return 0;

	} else {
		return 1;
	}
}

int mavlink_pm_send_param(param_t param)
{
    float value;
    param_get(param, &value);
    char* param_id=param_name(param);
    int type= param_type(param);
    int count= param_count();
    int index= param_get_index(param);
    mavlink_msg_param_value_send(MAVLINK_COMM_0, param_id, value, type, count, index);
}

void mavlink_pm_message_handler(const mavlink_channel_t chan, const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			/* Start sending parameters */
		    //Restart queue line
			mavlink_param_queue_index = 0;
		} break;

	case MAVLINK_MSG_ID_PARAM_SET: {

			/* Handle parameter setting */

			if (msg->msgid == MAVLINK_MSG_ID_PARAM_SET) {
				mavlink_param_set_t mavlink_param_set;
				mavlink_msg_param_set_decode(msg, &mavlink_param_set);

				if (mavlink_param_set.target_system == mavlink_system.sysid && ((mavlink_param_set.target_component == mavlink_system.compid) || (mavlink_param_set.target_component == MAV_COMP_ID_ALL))) {
					/* local name buffer to enforce null-terminated string */
					char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
					strncpy(name, mavlink_param_set.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
					/* enforce null termination */
					name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
					/* attempt to find parameter, set and send it */
					param_t param = param_find(name);
					/* set and send parameter */
					param_set(param, &(mavlink_param_set.param_value));
					mavlink_pm_send_param(param);
				}
			}
		} break;

	case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			mavlink_param_request_read_t mavlink_param_request_read;
			mavlink_msg_param_request_read_decode(msg, &mavlink_param_request_read);

			if (mavlink_param_request_read.target_system == mavlink_system.sysid && ((mavlink_param_request_read.target_component == mavlink_system.compid) || (mavlink_param_request_read.target_component == MAV_COMP_ID_ALL))) {
				/* when no index is given, loop through string ids and compare them */
				if (mavlink_param_request_read.param_index == -1) {
					/* local name buffer to enforce null-terminated string */
					char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
					strncpy(name, mavlink_param_request_read.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
					/* enforce null termination */
					name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
					/* attempt to find parameter and send it */
					mavlink_pm_send_param(param_find(name));

				} else {
					/* when index is >= 0, send this parameter again */
					mavlink_pm_send_param(param_for_index(mavlink_param_request_read.param_index));
				}
			}

		} break;
	}
}

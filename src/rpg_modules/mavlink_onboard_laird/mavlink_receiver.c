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
 * @file mavlink_receiver.c
 * MAVLink protocol message receive and dispatch
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

/* XXX trim includes */
#include <nuttx/config.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include "mavlink_bridge_header.h"
#include <drivers/drv_hrt.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>

#include "util.h"
#include "orb_topics.h"

/* XXX should be in a header somewhere */
pthread_t receive_start(int uart);

static void handle_message(mavlink_message_t *msg);
static void *receive_thread(void *arg);

static mavlink_status_t status;
static struct laird_control_setpoint_s laird_control_sp;
static orb_advert_t laird_control_sp_pub;

static void
handle_message(mavlink_message_t *msg)
{
	// Attitude MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT
	// TODO update mavlink message to attitude yaw_rate
        if (msg->msgid == MAVLINK_MSG_ID_ROLL_PITCH_YAW_THRUST_SETPOINT)
        {
          mavlink_roll_pitch_yaw_thrust_setpoint_t attitude_setpoint_mavlink_msg;

          mavlink_msg_roll_pitch_yaw_thrust_setpoint_decode(msg, &attitude_setpoint_mavlink_msg);

          laird_control_sp.mode = OFFBOARD_CONTROL_MODE_ATT_YAW_RATE;

          laird_control_sp.p1=attitude_setpoint_mavlink_msg.roll;
          laird_control_sp.p2=attitude_setpoint_mavlink_msg.pitch;
          laird_control_sp.p3=attitude_setpoint_mavlink_msg.yaw;
          laird_control_sp.p4=attitude_setpoint_mavlink_msg.thrust;

          laird_control_sp.timestamp = hrt_absolute_time();

          if( attitude_setpoint_mavlink_msg.thrust>0 ){
            laird_control_sp.armed = true;
          }
          else {
            laird_control_sp.armed = false;
          }

          /* check if topic has to be advertised */
          if (laird_control_sp_pub <= 0) {
            laird_control_sp_pub = orb_advertise(ORB_ID(laird_control_setpoint), &laird_control_sp);
          } else {
                  /* Publish */
                  orb_publish(ORB_ID(laird_control_setpoint), laird_control_sp_pub, &laird_control_sp);
          }
        } // end attitude

        // Rate
        if (msg->msgid == MAVLINK_MSG_ID_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT) {
          mavlink_roll_pitch_yaw_speed_thrust_setpoint_t rate_setpoint_mavlink_msg;
          mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint_decode(msg, &rate_setpoint_mavlink_msg);

          laird_control_sp.mode = OFFBOARD_CONTROL_MODE_DIRECT_RATES;

          laird_control_sp.p1=rate_setpoint_mavlink_msg.roll_speed;
          laird_control_sp.p2=rate_setpoint_mavlink_msg.pitch_speed;
          laird_control_sp.p3=rate_setpoint_mavlink_msg.yaw_speed;
          laird_control_sp.p4=rate_setpoint_mavlink_msg.thrust;

          laird_control_sp.timestamp = hrt_absolute_time();

          if( rate_setpoint_mavlink_msg.thrust>0 ){
            laird_control_sp.armed = true;
          }
          else {
            laird_control_sp.armed = false;
          }

          /* check if topic has to be advertised */
          if (laird_control_sp_pub <= 0) {
                  laird_control_sp_pub = orb_advertise(ORB_ID(laird_control_setpoint), &laird_control_sp);
          } else {
                  /* Publish */
                  orb_publish(ORB_ID(laird_control_setpoint), laird_control_sp_pub, &laird_control_sp);
          }
        } // end rate

        // parameter
        if (msg->msgid == MAVLINK_MSG_ID_PARAM_SET)
        {
          mavlink_param_set_t param_mavlink_msg;
          mavlink_msg_param_set_decode(msg, &param_mavlink_msg);

          printf("%s %3.5f \n",param_mavlink_msg.param_id, param_mavlink_msg.param_value);
          param_t param_ptr = param_find(param_mavlink_msg.param_id);
          param_set(param_ptr, &param_mavlink_msg.param_value);
        }
}


/**
 * Receive data from UART.
 */
static void *
receive_thread(void *arg)
{
	int uart_fd = *((int*)arg);

	const int timeout = 1000;
	uint8_t ch;

	mavlink_message_t msg;

	prctl(PR_SET_NAME, "mavlink offb mod rcv", getpid());

	while (!thread_should_exit) {

		struct pollfd fds[] = { { .fd = uart_fd, .events = POLLIN } };

		if (poll(fds, 1, timeout) > 0) {
			/* non-blocking read until buffer is empty */
			int nread = 0;

			do {
				nread = read(uart_fd, &ch, 1);

				if (mavlink_parse_char(chan, ch, &msg, &status)) { //parse the char
					/* handle generic messages and commands */
					handle_message(&msg);
				}
			} while (nread > 0);
		}
	}

	return NULL;
}

pthread_t
receive_start(int uart)
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	struct sched_param param;
  	param.sched_priority = SCHED_PRIORITY_MAX - 40;
  	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr, 2048);

	pthread_t thread;
	pthread_create(&thread, &receiveloop_attr, receive_thread, &uart);
	return thread;
}

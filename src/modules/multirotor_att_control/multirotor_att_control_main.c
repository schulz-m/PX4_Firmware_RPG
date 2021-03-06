/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file multirotor_att_control_main.c
 *
 * Implementation of multirotor attitude control main loop.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <getopt.h>
#include <time.h>
#include <math.h>
#include <poll.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <drivers/drv_gyro.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rpg/laird_control_setpoint.h>


#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

#include "multirotor_attitude_control.h"
#include "multirotor_rate_control.h"

__EXPORT int multirotor_att_control_main(int argc, char *argv[]);

static bool thread_should_exit;
static int mc_task;
static bool motor_test_mode = false;

PARAM_DEFINE_FLOAT(MC_RCLOSS_THR, 0.65f);

static int
mc_thread_main(int argc, char *argv[])
{
	/* declare and safely initialize all structs */
	struct vehicle_control_mode_s control_mode;
	memset(&control_mode, 0, sizeof(control_mode));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_attitude_setpoint_s att_sp;
	memset(&att_sp, 0, sizeof(att_sp));
	struct manual_control_setpoint_s manual;
	memset(&manual, 0, sizeof(manual));
	struct sensor_combined_s raw;
	memset(&raw, 0, sizeof(raw));
	struct offboard_control_setpoint_s offboard_sp;
	memset(&offboard_sp, 0, sizeof(offboard_sp));
        struct offboard_control_setpoint_s laird_sp;
        memset(&laird_sp, 0, sizeof(laird_sp));
	struct vehicle_rates_setpoint_s rates_sp;
	memset(&rates_sp, 0, sizeof(rates_sp));
	struct vehicle_status_s status;
	memset(&status, 0, sizeof(status));
	struct actuator_controls_s actuators;
	memset(&actuators, 0, sizeof(actuators));
        struct offboard_control_setpoint_s dominant_sp;
        memset(&dominant_sp, 0, sizeof(dominant_sp));

	/* subscribe to attitude, motor setpoints and system state */
	int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int param_sub = orb_subscribe(ORB_ID(parameter_update));
	int att_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	int setpoint_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));
        int laird_sub = orb_subscribe(ORB_ID(laird_control_setpoint));
	int control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	int rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	int status_sub = orb_subscribe(ORB_ID(vehicle_status));

	/*
	 * Do not rate-limit the loop to prevent aliasing
	 * if rate-limiting would be desired later, the line below would
	 * enable it.
	 *
	 * rate-limit the attitude subscription to 200Hz to pace our loop
	 * orb_set_interval(att_sub, 5);
	 */
	struct pollfd fds[2] = {
		{ .fd = att_sub, .events = POLLIN },
		{ .fd = param_sub, .events = POLLIN }
	};

	/* publish actuator controls */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++) {
		actuators.control[i] = 0.0f;
	}

	orb_advert_t actuator_pub = orb_advertise(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, &actuators);
	orb_advert_t att_sp_pub = orb_advertise(ORB_ID(vehicle_attitude_setpoint), &att_sp);
	orb_advert_t rates_sp_pub = orb_advertise(ORB_ID(vehicle_rates_setpoint), &rates_sp);

	/* register the perf counter */
	perf_counter_t mc_loop_perf = perf_alloc(PC_ELAPSED, "multirotor_att_control_runtime");
	perf_counter_t mc_interval_perf = perf_alloc(PC_INTERVAL, "multirotor_att_control_interval");
	perf_counter_t mc_err_perf = perf_alloc(PC_COUNT, "multirotor_att_control_err");

	/* welcome user */
	warnx("starting");

	bool control_yaw_position = true;
	bool control_rates = true;
	bool control_attitude = true;

	param_t mc_rc_loss_h = param_find("MC_RCLOSS_THR");
	float emergency_thurst = 0;
	param_get(mc_rc_loss_h, &emergency_thurst);

	while (!thread_should_exit) {

		/* wait for a sensor update, check for exit condition every 500 ms */
		int ret = poll(fds, 2, 500);

		if (ret < 0) {
			/* poll error, count it in perf */
			perf_count(mc_err_perf);

		} else if (ret == 0) {
			/* no return value, ignore */
		} else {

			/* only update parameters if they changed */
			if (fds[1].revents & POLLIN) {
				/* read from param to clear updated flag */
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), param_sub, &update);

				/* update parameters */
			}

			/* only run controller if attitude changed */
			if (fds[0].revents & POLLIN) {

				perf_begin(mc_loop_perf);

				/* get a local copy of system state */
				bool updated;
				orb_check(control_mode_sub, &updated);

				if (updated) {
					orb_copy(ORB_ID(vehicle_control_mode), control_mode_sub, &control_mode);
				}

				/* get a local copy of manual setpoint */
				orb_copy(ORB_ID(manual_control_setpoint), manual_sub, &manual);
				/* get a local copy of attitude */
				orb_copy(ORB_ID(vehicle_attitude), att_sub, &att);
				/* get a local copy of attitude setpoint */
				orb_copy(ORB_ID(vehicle_attitude_setpoint), att_setpoint_sub, &att_sp);
				/* get a local copy of rates setpoint */
				orb_check(setpoint_sub, &updated);
				if (updated) {
					orb_copy(ORB_ID(offboard_control_setpoint), setpoint_sub, &offboard_sp);
				}

                                orb_check(laird_sub, &updated);
                                if (updated) {
                                        orb_copy(ORB_ID(laird_control_setpoint), laird_sub, &laird_sp);
                                }

				/* get a local copy of status */
				orb_check(status_sub, &updated);

				if (updated) {
					orb_copy(ORB_ID(vehicle_status), status_sub, &status);
				}

				/* get a local copy of the current sensor values */
				orb_copy(ORB_ID(sensor_combined), sensor_sub, &raw);

				/* set flag to safe value */
                                control_rates = true;
                                control_attitude = true;
                                control_yaw_position = true;

				if ( status.arming_state == ARMING_STATE_ARMED ){
				  /* define which input is the dominating control input */
				  if (control_mode.flag_control_manual_enabled){
				    dominant_sp = laird_sp;
				  } else {
				    dominant_sp = offboard_sp;
				  }

                                  if (dominant_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_RATES) {
                                    control_rates = true;
                                    control_attitude = false;
                                    control_yaw_position = false;

                                    rates_sp.roll = dominant_sp.p1;
                                    rates_sp.pitch = dominant_sp.p2;
                                    rates_sp.yaw = dominant_sp.p3;
                                    rates_sp.thrust = dominant_sp.p4;
                                    rates_sp.timestamp = hrt_absolute_time();
                                    orb_publish(ORB_ID(vehicle_rates_setpoint), rates_sp_pub, &rates_sp);

                                  } else if (dominant_sp.mode == OFFBOARD_CONTROL_MODE_DIRECT_ATTITUDE) {
                                    control_rates = true;
                                    control_attitude = true;
                                    control_yaw_position = true;

                                    att_sp.roll_body = dominant_sp.p1;
                                    att_sp.pitch_body = dominant_sp.p2;
                                    att_sp.yaw_body = dominant_sp.p3;
                                    att_sp.thrust = dominant_sp.p4;
                                    att_sp.timestamp = hrt_absolute_time();
                                    /* publish the result to the vehicle actuators */
                                    orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

                                  } else if ( dominant_sp.mode == OFFBOARD_CONTROL_MODE_ATT_YAW_RATE ) {
                                    control_rates = true;
                                    control_attitude = true;
                                    control_yaw_position = false;

                                    att_sp.roll_body = dominant_sp.p1;
                                    att_sp.pitch_body = dominant_sp.p2;
                                    att_sp.yaw_body = 0;
                                    att_sp.thrust = dominant_sp.p4;
                                    att_sp.timestamp = hrt_absolute_time();

                                    rates_sp.yaw = dominant_sp.p3;
                                    orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
                                  } else {
                                    // should never happen
                                    control_rates = true;
                                    control_attitude = true;
                                    control_yaw_position = false;

                                    att_sp.roll_body = 0;
                                    att_sp.pitch_body = 0;
                                    att_sp.yaw_body = 0;
                                    att_sp.thrust = emergency_thurst;
                                    att_sp.timestamp = hrt_absolute_time();

                                    rates_sp.yaw = 0;
                                    orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);

                                  }
				} else { // we are in an emergency mode
				  control_rates = true;
                                  control_attitude = true;
                                  control_yaw_position = false;

                                  att_sp.roll_body = 0;
                                  att_sp.pitch_body = 0;
                                  att_sp.yaw_body = 0;
                                  att_sp.thrust = emergency_thurst;
                                  att_sp.timestamp = hrt_absolute_time();

                                  rates_sp.yaw = 0;
                                  orb_publish(ORB_ID(vehicle_attitude_setpoint), att_sp_pub, &att_sp);
				}

				/* check if we should we reset integrals */
				bool reset_integral = !control_mode.flag_armed || att_sp.thrust < 0.1f;	// TODO use landed status instead of throttle

				/* run attitude controller if needed */
				if ( control_attitude ) {
					multirotor_control_attitude(&att_sp, &att, &rates_sp, control_yaw_position, reset_integral);
					orb_publish(ORB_ID(vehicle_rates_setpoint), rates_sp_pub, &rates_sp);
				}

				/* measure in what intervals the controller runs */
				perf_count(mc_interval_perf);

				/* run rates controller if needed */
				if (  control_rates ) {
					/* get current rate setpoint */
					bool rates_sp_updated = false;
					orb_check(rates_sp_sub, &rates_sp_updated);

					if (rates_sp_updated) {
						orb_copy(ORB_ID(vehicle_rates_setpoint), rates_sp_sub, &rates_sp);
					}

					/* apply controller */
					float rates[3];
					rates[0] = att.rollspeed;
					rates[1] = att.pitchspeed;
					rates[2] = att.yawspeed;
					multirotor_control_rates(&rates_sp, rates, &actuators, reset_integral);

				} else {
					/* rates controller disabled, set actuators to zero for safety */
					actuators.control[0] = 0.0f;
					actuators.control[1] = 0.0f;
					actuators.control[2] = 0.0f;
					actuators.control[3] = 0.0f;
				}

				actuators.timestamp = hrt_absolute_time();
				orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);

				perf_end(mc_loop_perf);
			} /* end of poll call for attitude updates */
		} /* end of poll return value check */
	}

	warnx("stopping, disarming motors");

	/* kill all outputs */
	for (unsigned i = 0; i < NUM_ACTUATOR_CONTROLS; i++)
		actuators.control[i] = 0.0f;

	orb_publish(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_pub, &actuators);


	close(att_sub);
	close(control_mode_sub);
	close(manual_sub);
	close(actuator_pub);
	close(att_sp_pub);

	perf_print_counter(mc_loop_perf);
	perf_free(mc_loop_perf);

	fflush(stdout);
	exit(0);
}

static void
usage(const char *reason)
{
	if (reason)
		fprintf(stderr, "%s\n", reason);

	fprintf(stderr, "usage: multirotor_att_control [-m <mode>] [-t] {start|status|stop}\n");
	fprintf(stderr, "    <mode> is 'rates' or 'attitude'\n");
	fprintf(stderr, "    -t enables motor test mode with 10%% thrust\n");
	exit(1);
}

int multirotor_att_control_main(int argc, char *argv[])
{
	int	ch;
	unsigned int optioncount = 0;

	while ((ch = getopt(argc, argv, "tm:")) != EOF) {
		switch (ch) {
		case 't':
			motor_test_mode = true;
			optioncount += 1;
			break;

		case ':':
			usage("missing parameter");
			break;

		default:
			fprintf(stderr, "option: -%c\n", ch);
			usage("unrecognized option");
			break;
		}
	}

	argc -= optioncount;
	//argv += optioncount;

	if (argc < 1)
		usage("missing command");

	if (!strcmp(argv[1 + optioncount], "start")) {

		thread_should_exit = false;
		mc_task = task_spawn_cmd("multirotor_att_control",
					 SCHED_DEFAULT,
					 SCHED_PRIORITY_MAX - 15,
					 2048,
					 mc_thread_main,
					 NULL);
		exit(0);
	}

	if (!strcmp(argv[1 + optioncount], "stop")) {
		thread_should_exit = true;
		exit(0);
	}

	usage("unrecognized command");
	exit(1);
}

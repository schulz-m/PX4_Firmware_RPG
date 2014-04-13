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

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <poll.h>
#include <systemlib/err.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/rpg/torques_and_thrust.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rpg/thrust_inputs.h>

#include <systemlib/systemlib.h>

#include "ardrone_motor_interface.h"

__EXPORT int rpg_ardrone_interface_main(int argc, char *argv[]);

static bool thread_should_exit = false;
static bool thread_running = false;
static int rpg_ardrone_interface_task;

static int ardroneInterfaceThreadMain(int argc, char *argv[])
{
  thread_running = true;

  // Initialize structs
  struct torques_and_thrust_s desired_torques_and_thrust;
  memset(&desired_torques_and_thrust, 0, sizeof(desired_torques_and_thrust));
  struct thrust_inputs_s thrust_inputs;
  memset(&thrust_inputs, 0, sizeof(thrust_inputs));

  // Subscribers
  int torques_and_thrust_sub = orb_subscribe(ORB_ID(torques_and_thrust));
  int param_sub = orb_subscribe(ORB_ID(parameter_update));

  // Limit this loop frequency to 200Hz
  orb_set_interval(torques_and_thrust_sub, 5);

  struct pollfd fds[2] = { {.fd = torques_and_thrust_sub, .events = POLLIN}, {.fd = param_sub, .events = POLLIN}};

  // Publisher of actually applied thrusts to each rotor
  orb_advert_t thrust_inputs_pub = orb_advertise(ORB_ID(thrust_inputs), &thrust_inputs);

  // Read device name from command line input
  bool use_x_configuration = false;
  char *device = "/dev/ttyS1";
  for (int i = 0; i < argc && argv[i]; i++)
  {
    if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0)
    { //device set
      if (argc > i + 1)
      {
        device = argv[i + 1];
        printf("device set to %s \n", device);
      }
      else
      {
        thread_running = false;
        fprintf(stderr, "usage: rpg_ardrone_interface {start|stop|status} [-d <UART>]\n\n");
      }
    }
    // Check if "x-configuration" should be used or not (otherwise "+-configuration")
    if (strcmp(argv[i], "-x") == 0)
    {
      use_x_configuration = true;
      printf("using x configuration \n");
    }
  }

  // open ardrone motor ports
  static int ardrone_write;
  static struct termios uart_config_original;
  static int gpios;
  if (openArdroneMotorPorts(device, &ardrone_write, &uart_config_original, &gpios) != 0)
  {
    thread_should_exit = true;
  }

  // Send zero commands to make sure rotors are not spinning
  ardroneWriteMotorCommands(ardrone_write, 0, 0, 0, 0);

  // Initializing parameters
  static struct rpg_ardrone_interface_params params;
  static struct rpg_ardrone_interface_params_handles params_handle;
  parametersInit(&params_handle);
  parametersUpdate(&params_handle, &params);

  int skip_thrust_msgs = 4;
  int skip_counter = 0;

  while (!thread_should_exit)
  {
    int ret = poll(fds, 2, 500);

    if (ret > 0)
    {
      if (fds[0].revents & POLLIN)
      {
        orb_copy(ORB_ID(torques_and_thrust), torques_and_thrust_sub, &desired_torques_and_thrust);

        uint16_t motor_commands[4];
        if (desired_torques_and_thrust.normalized_thrust > 0.1)
        {
          computeMotorCommands(motor_commands, desired_torques_and_thrust, use_x_configuration, params);
        }
        else
        {
          motor_commands[0] = 0;
          motor_commands[1] = 0;
          motor_commands[2] = 0;
          motor_commands[3] = 0;
        }

        ardroneWriteMotorCommands(ardrone_write, motor_commands[0], motor_commands[1], motor_commands[2],
                                     motor_commands[3]);

        skip_counter++;
        if (skip_counter >= skip_thrust_msgs)
        {
          // Publish the motor inputs as uorb topic
          thrust_inputs.timestamp = hrt_absolute_time();
          thrust_inputs.thrust_inputs[0] = convertMotorCommandToThrust(motor_commands[0]);
          thrust_inputs.thrust_inputs[1] = convertMotorCommandToThrust(motor_commands[1]);
          thrust_inputs.thrust_inputs[2] = convertMotorCommandToThrust(motor_commands[2]);
          thrust_inputs.thrust_inputs[3] = convertMotorCommandToThrust(motor_commands[3]);
          orb_publish(ORB_ID(thrust_inputs), thrust_inputs_pub, &thrust_inputs);

          skip_counter = 0;
        }
      }

      // only update parameters if they changed
      if (fds[1].revents & POLLIN)
      {
        // read from param to clear updated flag
        struct parameter_update_s updated_parameters;
        orb_copy(ORB_ID(parameter_update), param_sub, &updated_parameters);

        // update parameters
        parametersUpdate(&params_handle, &params);
      }
    }
  }

  // Kill motors
  ardroneWriteMotorCommands(ardrone_write, 0, 0, 0, 0);

  // Close motor ports
  closeArdroneMotorPorts(&ardrone_write, &uart_config_original, &gpios);

  close(torques_and_thrust_sub);
  close(param_sub);
  close(thrust_inputs_pub);

  thread_running = false;
  exit(0);
}

static void usage(const char *reason)
{
  if (reason)
    fprintf(stderr, "%s\n", reason);
  fprintf(stderr, "usage: rpg_ardrone_interface {start|stop|status} [-d <UART>]\n\n");
  exit(1);
}

int rpg_ardrone_interface_main(int argc, char *argv[])
{
  if (argc < 1)
    usage("missing command");

  if (!strcmp(argv[1], "start"))
  {
    printf("starting rpg_ardrone_interface\n");
    if (thread_running)
    {
      printf("rpg_ardrone_interface already running\n");
      /* this is not an error */
      exit(0);
    }

    thread_should_exit = false;
    rpg_ardrone_interface_task = task_spawn_cmd("rpg_ardrone_interface",
    SCHED_DEFAULT,
                                                SCHED_PRIORITY_MAX - 15, 2048, ardroneInterfaceThreadMain,
                                                (argv) ? (const char **)&argv[2] : (const char **)NULL);
    exit(0);
  }

  if (!strcmp(argv[1], "stop"))
  {
    printf("stopping rpg_ardrone_interface\n");
    thread_should_exit = true;
    exit(0);
  }

  if (!strcmp(argv[1], "status"))
  {
    if (thread_running)
    {
      printf("rpg_ardrone_interface is running\n");
    }
    else
    {
      printf("rpg_ardrone_interface not started\n");
    }
    exit(0);
  }

  usage("unrecognized command");
  exit(1);
}

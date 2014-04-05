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
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/camera_trigger_msg.h>

#include <systemlib/systemlib.h>

#include "camera_triggering.h"

__EXPORT int camera_triggering_main(int argc, char *argv[]);

static bool thread_should_exit = false;
static bool thread_running = false;
static int rpg_ardrone_interface_task;
static unsigned long trigger_gpio = GPIO_EXT_1;

static int cameraTriggeringThreadMain(int argc, char *argv[])
{
  thread_running = true;

  int fd = open("/dev/px4fmu", 0);
  initGPIO(fd, trigger_gpio);

  // Initialize structs
  struct sensor_combined_s sensor_raw;
  memset(&sensor_raw, 0, sizeof(sensor_raw));
  struct camera_trigger_msg_s trigger_msg;
  memset(&trigger_msg, 0, sizeof(trigger_msg));

  // Counters
  unsigned long frame_counter = 0;
  int frame_skip_counter = 1;

  // Subscribers
  int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));

  // Publishers
  orb_advert_t trigger_msg_pub = orb_advertise(ORB_ID(camera_trigger_msg), &trigger_msg);

  // Limit this loop frequency to 200Hz
  orb_set_interval(sensor_sub, 5);

  struct pollfd fds[1] = { {.fd = sensor_sub, .events = POLLIN}};

  // Read device name from command line input
  int skip_rate;
  for (int i = 0; i < argc && argv[i]; i++)
  {
    if (strcmp(argv[i], "-r") == 0)
    { //device set
      if (argc > i + 1)
      {
        skip_rate = argv[i + 1];
        printf("skip rate set to %d \n", skip_rate);
      }
      else
      {
        thread_running = false;
        fprintf(stderr, "usage: camera_triggering {start|stop|status} [-r rate]\n\n");
      }
    }
  }

  while (!thread_should_exit)
  {
    int ret = poll(fds, 1, 500);

    if (ret > 0)
    {
      if (fds[0].revents & POLLIN)
      {
        if (frame_skip_counter++ >= skip_rate)
        {
          // Trigger the camera
          setGPIOHigh(fd, trigger_gpio);
          usleep(0); // TODO: Adjust this to have the pin high for a reasonable time
          setGPIOLow(fd, trigger_gpio);

          // Update Counters
          frame_counter++;
          frame_skip_counter = 1;

          // get a local copy of the current sensor values
          orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensor_raw);

          // Send a message with camera frame info and timestamp
          trigger_msg.camera_name = "camera1";
          trigger_msg.frame_number = frame_counter;
          trigger_msg.timestamp = sensor_raw.timestamp;
          orb_publish(ORB_ID(camera_trigger_msg), trigger_msg_pub, &trigger_msg);
        }
      }
    }
  }

  close(sensor_sub);

  deinitGPIO(fd, trigger_gpio);
  close(fd);

  thread_running = false;
  exit(0);
}

static void usage(const char *reason)
{
  if (reason)
    fprintf(stderr, "%s\n", reason);
  fprintf(stderr, "usage: camera_triggering {start|stop|status} [-r rate]\n\n");
  exit(1);
}

int camera_triggering_main(int argc, char *argv[])
{
  if (argc < 1)
    usage("missing command");

  if (!strcmp(argv[1], "start"))
  {
    printf("starting camera_triggering\n");
    if (thread_running)
    {
      printf("camera_triggering already running\n");
      /* this is not an error */
      exit(0);
    }

    thread_should_exit = false;
    rpg_ardrone_interface_task = task_spawn_cmd("camera_triggering",
    SCHED_DEFAULT,
                                                SCHED_PRIORITY_MAX - 15, 2048, cameraTriggeringThreadMain,
                                                (argv) ? (const char **)&argv[2] : (const char **) NULL);
    exit(0);
  }

  if (!strcmp(argv[1], "stop"))
  {
    printf("stopping camera_triggering\n");
    thread_should_exit = true;
    exit(0);
  }

  if (!strcmp(argv[1], "status"))
  {
    if (thread_running)
    {
      printf("camera_triggering is running\n");
    }
    else
    {
      printf("camera_triggering not started\n");
    }
    exit(0);
  }

  usage("unrecognized command");
  exit(1);
}

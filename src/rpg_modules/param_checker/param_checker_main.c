/**
 * @file param_checker_main.c
 *
 * This module defines a parameter and then checks its status periodically
 *
 * Based on the module 'multirotor_att_control' by Lorenz Meier and Anton Babushkin
 *
 * by Adrian Rechy Romero <readrian@student.ethz.ch
 */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <time.h>
#include <math.h>
#include <poll.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>

__EXPORT int param_checker_main(int argc, char *argv[]);

static bool thread_should_exit = false;
static bool thread_running = false;
static int pc_task; //Handle of the task

PARAM_DEFINE_FLOAT(Z_PARAM, 0.65f);

static int pc_thread_main(int argc, char *argv[])
{
  /* subscribe to parameter_update */
  int param_sub = orb_subscribe(ORB_ID(parameter_update));

  /*
   * Do not rate-limit the loop to prevent aliasing
   * if rate-limiting would be desired later, the line below would
   * enable it.
   *
   * rate-limit the attitude subscription to 200Hz to pace our loop
   * orb_set_interval(att_sub, 5);
   */
  struct pollfd fds[1] = { {.fd = param_sub, .events = POLLIN}};

  /* welcome user */
  warnx("[param_checker] starting\n");

  param_t z_param_h = param_find("Z_PARAM");
  float z_param_value = 0;
  param_get(z_param_h, &z_param_value);

  thread_running = true;
  while (!thread_should_exit)
  {
    /* wait for a sensor update, check for exit condition every 500 ms */
    int ret = poll(fds, 2, 500);

    if (ret < 0)
    {
      /* poll error, might need to count it in perf */
    }
    else if (ret == 0)
    {
      /* no return value, ignore */
    }
    else
    {

      /* only update parameters if they changed */
      if (fds[0].revents & POLLIN)
      {
        /* read from param to clear updated flag */
        struct parameter_update_s update;
        orb_copy(ORB_ID(parameter_update), param_sub, &update);

        /* update parameters */
        param_get(z_param_h, &z_param_value);
        warnx("parameter changed %f", z_param_value);
      }
    } /* end of poll return value check */
  }
  warnx("[param_checker] exiting.\n");
  thread_running = false;

  // closing function
  exit(0);
  return 0;
}

static void usage(const char *reason)
{
  if (reason)
    fprintf(stderr, "%s\n", reason);

  errx(1, "usage: param_checker {start|status|stop}\n");
}

int param_checker_main(int argc, char *argv[])
{
  if (argc < 1)
    usage("missing command");

  if (!strcmp(argv[1], "start"))
  {
    if (thread_running)
    {
      warnx("param_checker already running \n");
      // not an error, just a notification
      exit(0);
    }
    thread_should_exit = false;
    pc_task = task_spawn_cmd("param_checker",
    SCHED_DEFAULT,
                             SCHED_PRIORITY_DEFAULT, 2048, pc_thread_main,
                             (argv) ? (const char **)&argv[2] : (const char **)NULL);
    exit(0);
  }

  if (!strcmp(argv[1], "stop"))
  {
    thread_should_exit = true;
    exit(0);
  }
  if (!strcmp(argv[1], "status"))
  {
    if (thread_running)
    {
      warnx("\trunning\n");
    }
    else
    {
      warnx("\tnot started\n");
    }
    exit(0);
  }

  usage("unrecognized command");
  exit(1);
}

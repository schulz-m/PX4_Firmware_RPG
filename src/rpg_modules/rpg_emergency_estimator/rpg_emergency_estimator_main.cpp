/* Main File of Emergency State Estimation */

/* General Libaries */

#include <nuttx/config.h>
#include <nuttx/sched.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <sys/prctl.h>

#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <mqueue.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "EKFFunction.hpp"

//Export C for makefile
extern "C" __EXPORT int rpg_emergency_estimator_main(int argc, char *argv[]);

// Thread running statics:
static bool thread_should_exit = false; /**< Deamon exit flag */
static bool thread_running = false; /**< Deamon status flag */
static int emergency_estimator_task; /**< Handle of deamon task / thread */

/*
 * EKF Attitude Estimator main function.
 *
 * Estimates the attitude recursively once started.
 *
 * @param argc number of commandline arguments (plus command name)
 * @param argv strings containing the arguments
 */
static int rpgEmergencyEstimatorThreadMain(int argc, char *argv[])
{

	/* print welcome text */
	warnx("The emergency procedure is starting...");

	using namespace math;

	thread_running = true;

	EKFFunction ekf(NULL,"EKFE");

	warnx("EKF Function Constructor succesful");

	//Initial state can be set:
//	float initial_state[8] = {0,0,0,0,1,0,0,0};
//	ekf.setState(initial_state);

	while (!thread_should_exit) {
		ekf.update();
	}

	thread_running = false;
	warnx("exiting.");
	exit(0);
}

/**
 * Print correct usage.
 */

static void usage(const char *reason)
{
  if (reason)
    fprintf(stderr, "%s\n", reason);

  fprintf(stderr, "usage: rpg_emergency_estimator {start|status|stop}\n");
  exit(1);
}

/**
 * Mainloop of emergency_estimator.
 */

int rpg_emergency_estimator_main(int argc, char *argv[])
{
  if (argc < 1)
    usage("missing command");

  if (!strcmp(argv[1], "start"))
  {
    if (thread_running)
    {
      printf("rpg_emergency_estimator already running\n");
      // this is not an error
      exit(0);
    }

    thread_should_exit = false;

    // task definition:
    emergency_estimator_task = task_spawn_cmd("rpg_emergency_estimator",
				 SCHED_DEFAULT,
				 SCHED_PRIORITY_DEFAULT, //should be lower than rpg_sensors priority
				 14000, //stack size, should be higher than 10'000 bits
				 rpgEmergencyEstimatorThreadMain,
				 (argv) ? (const char **)&argv[2] : (const char **)NULL);
    exit(0);
  }

  if (!strcmp(argv[1], "stop"))
  {
    thread_should_exit = true;
    while (thread_running)
    {
      usleep(200000);
    }
    printf("stopping rpg_emergency_estimator\n");
    exit(0);
  }

  if (!strcmp(argv[1], "status"))
  {
    if (thread_running)
    {
      warnx("rpg_attitude_controller is running");
      exit(0);
    }
    else
    {
      warnx("rpg_attitude_controller not started");
      exit(1);
    }
  }
  usage("unrecognized command");
  exit(1);
}

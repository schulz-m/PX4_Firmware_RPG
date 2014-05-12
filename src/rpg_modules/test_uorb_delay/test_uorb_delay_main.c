#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <systemlib/systemlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/rpg/thrust_inputs.h>

__EXPORT int test_uorb_delay_main(int argc, char *argv[]);

static bool thread_should_exit;
static bool thread_running = false;
static int uorb_delay_task;

static int test_uorb_delay_thread_main(int argc, char *argv[])
{
  thread_running = true;

  struct thrust_inputs_s thrust_inputs;
  memset(&thrust_inputs, 0, sizeof(thrust_inputs));

  int rotor_thrusts_sub = orb_subscribe(ORB_ID(thrust_inputs));

  struct pollfd fds[] = { {.fd = rotor_thrusts_sub, .events = POLLIN}};

  int counter = 0;
  float avg_time_delay = 0.0f;

  while (!thread_should_exit)
  {
    int ret = poll(fds, 1, 500);
    if (ret < 0)
    {
    }
    else if (ret == 0)
    {
    }
    else
    {
      if (fds[0].revents & POLLIN)
      {
        orb_copy(ORB_ID(thrust_inputs), rotor_thrusts_sub, &thrust_inputs);

        if (counter >= 100)
        {
          printf("delay: %.4f thrusts: %.3f  %.3f  %.3f  %.3f\n", avg_time_delay / 100.0f,
                 thrust_inputs.thrust_inputs[0], thrust_inputs.thrust_inputs[1], thrust_inputs.thrust_inputs[2],
                 thrust_inputs.thrust_inputs[3]);
          counter = 0;
          avg_time_delay = 0.0f;
        }
        else
        {
          avg_time_delay += ((float)(hrt_absolute_time() - thrust_inputs.timestamp)) / 1000.0f;
          counter++;
        }
      }
    }

  }

  thread_running = false;

  exit(0);
}

int test_uorb_delay_main(int argc, char *argv[])
{
  if (argc < 1)
    printf("missing command");

  if (!strcmp(argv[1], "start"))
  {
    if (thread_running)
    {
      printf("test_uorb_delay already running\n");
      // this is not an error
      exit(0);
    }

    thread_should_exit = false;
    uorb_delay_task = task_spawn_cmd("test_uorb_delay",
    SCHED_DEFAULT,
                                     SCHED_PRIORITY_MAX - 15, 2048, test_uorb_delay_thread_main,
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
      warnx("running");
      exit(0);
    }
    else
    {
      warnx("not started");
      exit(1);
    }
    exit(0);
  }

  exit(1);
}

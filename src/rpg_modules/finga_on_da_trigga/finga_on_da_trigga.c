#include <nuttx/config.h>
#include <nuttx/sched.h>

#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include <time.h>
#include <float.h>
#include <unistd.h>

#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

__EXPORT int finga_on_da_trigga_main(int argc, char *argv[]);

static int main_thread(int argc, char *argv[]);

/* thread state */
volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int mavlink_task;

/**
 * MAVLink Protocol main function.
 */
int main_thread(int argc, char *argv[])
{
  /* print welcome text */
  warnx("This is the thread...");
  fflush(stdout);
  thread_running = true;

  struct sensor_combined_s sensor_uorb_msg;
  int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
  orb_set_interval(sensor_sub, 10); //100 Hz

  struct pollfd fds[] = { {.fd = sensor_sub, .events = POLLIN}, };

  int ctr = 0;
  int max_packets = 1000;
  uint64_t timestamp;

  while (!thread_should_exit)
  {
    if (ctr >= max_packets)
    {
      ctr = 0;
      double dt = ((float)(hrt_absolute_time() - timestamp)) / ((float)max_packets) / 1000000.0;
      printf("dt: %.2f\n", 1 / dt);
      timestamp = hrt_absolute_time();
    }
    ctr++;

//    int poll_ret = poll(fds, 2, 10);
//    if (poll_ret > 0 && (fds[0].revents & POLLIN))
//    {
//      // sensors
//      orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensor_uorb_msg);
//    }
    usleep(1000);
  }
  thread_running = false;
  warnx("Thread is done");
  exit(0);
}

int finga_on_da_trigga_main(int argc, char *argv[])
{

  if (argc < 2)
  {
    warnx("missing command");
  }

  if (!strcmp(argv[1], "start"))
  {

    /* this is not an error */
    if (thread_running)
      errx(0, "thread already running\n");

    thread_should_exit = false;
    mavlink_task = task_spawn_cmd("finga_on_da_trigger_thread",
    SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT, 2048, main_thread, (const char**)argv);
    exit(0);
  }

  if (!strcmp(argv[1], "stop"))
  {
    thread_should_exit = true;
    while (thread_running)
    {
      usleep(200000);
    }
    warnx("terminated.");
    exit(0);
  }

  if (!strcmp(argv[1], "status"))
  {
    if (thread_running)
    {
      errx(0, "running");
    }
    else
    {
      errx(1, "not running");
    }
  }

  warnx("unrecognized command");
  /* not getting here */
  return 0;
}


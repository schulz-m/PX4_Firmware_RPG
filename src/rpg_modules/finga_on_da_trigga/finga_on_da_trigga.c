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
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>

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

  struct accel_report accel_report;
  memset(&accel_report, 0, sizeof(accel_report));
  int accel_sub = orb_subscribe(ORB_ID(sensor_accel));
  struct gyro_report gyro_report;
  memset(&gyro_report, 0, sizeof(gyro_report));
  int gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));

  //orb_set_interval(accel_sub, 10); //100 Hz

  struct pollfd fds[2] = { {.fd = accel_sub, .events = POLLIN}, {.fd = gyro_sub, .events = POLLIN}};

  int ctr = 0;
  int max_packets = 1000;
  uint64_t timestamp;
  uint64_t last_msg_timestamp;

  while (!thread_should_exit)
  {
    int poll_ret = poll(fds, 2, 10);
    if (poll_ret > 0 && (fds[0].revents & POLLIN))
    {
      // get accelerometer
      orb_copy(ORB_ID(sensor_accel), accel_sub, &accel_report);

      // get gyro
      bool gyro_updated;
      orb_check(gyro_sub, &gyro_updated);
      if (gyro_updated)
      {
        orb_copy(ORB_ID(sensor_gyro), gyro_sub, &gyro_report);
      }
    }

    ctr++;
    if (ctr >= max_packets)
    {
      ctr = 0;
      double dt = ((float)(hrt_absolute_time() - timestamp)) / ((float)max_packets) / 1000000.0f;
      printf("frequency: %4.2f\n", 1.0f / dt);
      printf("timestamps [acc, gyr]: %3.20f   %3.20f\n", accel_report.timestamp/1000000.0f, gyro_report.timestamp/1000000.0f);
      timestamp = hrt_absolute_time();
    }

    last_msg_timestamp = accel_report.timestamp;
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


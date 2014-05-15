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
#include <uORB/topics/rpg/imu_msg.h>

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

//  struct accel_report accel_report;
//  memset(&accel_report, 0, sizeof(accel_report));
//  int accel_sub = orb_subscribe(ORB_ID(sensor_accel));
//  struct gyro_report gyro_report;
//  memset(&gyro_report, 0, sizeof(gyro_report));
//  int gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));
  struct imu_msg_s imu_msg;
  memset(&imu_msg, 0, sizeof(imu_msg));
  int imu_sub = orb_subscribe(ORB_ID(imu_msg));

  //orb_set_interval(accel_sub, 10); //100 Hz

  struct pollfd fds[2] = { {.fd = imu_sub, .events = POLLIN}};

  int ctr = 0;
  int max_packets = 100;
  uint64_t timestamp;
  uint64_t last_msg_timestamp;

  while (!thread_should_exit)
  {
    int poll_ret = poll(fds, 1, 10);
    if (poll_ret > 0 && (fds[0].revents & POLLIN))
    {
      // get accelerometer
      orb_copy(ORB_ID(imu_msg), imu_sub, &imu_msg);
    }

    ctr++;
    if (ctr >= max_packets)
    {
      ctr = 0;
      double dt = ((float)(hrt_absolute_time() - timestamp)) / ((float)max_packets) / 1000000.0f;
      printf("frequency: %4.2f\n", 1.0f / dt);
      //printf("timestamps: %3.20f   \n", imu_msg.timestamp/1000000.0f);
      //printf("%2.2f  %2.2f  %2.2f  %2.2f  %2.2f  %2.2f\n",imu_msg.gyro_x,imu_msg.gyro_y,imu_msg.gyro_z,imu_msg.acc_x,imu_msg.acc_y,imu_msg.acc_z);
      timestamp = hrt_absolute_time();
    }
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


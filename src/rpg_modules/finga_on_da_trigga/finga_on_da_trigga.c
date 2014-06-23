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
//additional
#include <drivers/drv_baro.h>

#include <uORB/topics/rpg/imu_msg.h>
#include <drivers/drv_mag.h>

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

  struct imu_msg_s imu_msg;
  memset(&imu_msg, 0, sizeof(imu_msg));
  int imu_sub = orb_subscribe(ORB_ID(imu_msg));

  struct mag_report mag_report;
  memset(&mag_report, 0, sizeof(mag_report));
  int mag_sub = orb_subscribe(ORB_ID(sensor_mag));

  //orb_set_interval(imu_sub, 10); //100 Hz

  struct pollfd fds[2] = { {.fd = imu_sub, .events = POLLIN}, {.fd = mag_sub, .events = POLLIN}};

  int ctr = 0;
  int max_packets = 100;
  uint64_t imu_timestamp = 0;
  uint64_t baro_timestamp = 0;
  uint64_t sonar_timestamp = 0;
//  uint64_t last_msg_timestamp;

  int ctr_2 = 0;
  int max_packets_2 = 100;
  uint64_t timestamp;
  uint64_t timestamp_2;

  while (!thread_should_exit)
  {
    int poll_ret = poll(fds, 2, 10);
    if (poll_ret > 0 && (fds[0].revents & POLLIN))
    {
      // get imu msg
      orb_copy(ORB_ID(imu_msg), imu_sub, &imu_msg);
      //printf("dt: %1.10f \n", (imu_msg.timestamp - last_msg_timestamp)/1000000.0f);
      //last_msg_timestamp = imu_msg.timestamp;

      ctr++;
      if (ctr >= max_packets)
      {
        ctr = 0;
        double dt = ((float)(hrt_absolute_time() - timestamp)) / ((float)max_packets) / 1000000.0f;
        printf("imu frequency: %4.2f\n", 1.0f / dt);
        //printf("timestamps: %3.20f   \n", imu_msg.timestamp/1000000.0f);
        //printf("%2.2f  %2.2f  %2.2f  %2.2f  %2.2f  %2.2f\n",imu_msg.gyro_x,imu_msg.gyro_y,imu_msg.gyro_z,imu_msg.acc_x,imu_msg.acc_y,imu_msg.acc_z);
        timestamp = hrt_absolute_time();
      }
    }

    if (poll_ret > 0 && (fds[1].revents & POLLIN))
    {
      orb_copy(ORB_ID(sensor_mag), mag_sub, &mag_report);

      ctr_2++;
      if (ctr_2 >= max_packets_2)
      {
        ctr_2 = 0;
        double dt = ((float)(hrt_absolute_time() - timestamp_2)) / ((float)max_packets_2) / 1000000.0f;
        printf("mag frequency: %4.2f\n", 1.0f / dt);
        //printf("timestamps: %3.20f   \n", imu_msg.timestamp/1000000.0f);
        //printf("%2.2f  %2.2f  %2.2f  %2.2f  %2.2f  %2.2f\n",imu_msg.gyro_x,imu_msg.gyro_y,imu_msg.gyro_z,imu_msg.acc_x,imu_msg.acc_y,imu_msg.acc_z);
        timestamp_2 = hrt_absolute_time();
      }
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


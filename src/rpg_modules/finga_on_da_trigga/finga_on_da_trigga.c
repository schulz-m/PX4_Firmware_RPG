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
#include <uORB/topics/rpg/sonar_msg.h>

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

  struct baro_report baro_msg;
  int baro_sub = orb_subscribe(ORB_ID(sensor_baro));
  memset(&baro_msg, 0, sizeof(baro_msg));
  orb_set_interval(baro_sub, 10); //100 Hz
  // Also test pressure and sonar messages
//  struct sonar_msg_s sonar_msg;
//  int sonar_sub = orb_subscribe(ORB_ID(sonar_msg));
//  memset(&sonar_msg, 0, sizeof(sonar_msg));
//  orb_set_interval(sonar_sub, 10); //100 Hz

  //orb_set_interval(accel_sub, 10); //100 Hz
  orb_copy(ORB_ID(sensor_baro), baro_sub, &baro_msg);
  printf("First pressure: %3.3f",baro_msg.pressure);

  struct pollfd fds[2];
  fds[0].fd = imu_sub;
  fds[0].events = POLLIN;
  fds[1].fd = baro_sub;
  fds[1].events = POLLIN;


 float dt_imu;
 float dt_baro;
  int ctr = 0;
  int max_packets = 1000;
  uint64_t imu_timestamp = 0;
  uint64_t baro_timestamp = 0;
//  uint64_t last_msg_timestamp;

  while (!thread_should_exit)
  {
    int poll_ret = poll(fds, 2, 1000);
    if (poll_ret > 0 && (fds[0].revents & POLLIN))
    {
      // get accelerometer
      orb_copy(ORB_ID(imu_msg), imu_sub, &imu_msg);
		dt_imu = ((float)(imu_msg.timestamp - imu_timestamp))/ 1.0e6f; //1000000.0f;
		imu_timestamp = imu_msg.timestamp;
    }
    if (poll_ret > 0 && (fds[1].revents & POLLIN))
    {
    	orb_copy(ORB_ID(sensor_baro), baro_sub, &baro_msg);
		dt_baro = ((float)(baro_msg.timestamp - baro_timestamp))/ 1.0e6f; //1000000.0f;
		baro_timestamp = baro_msg.timestamp;
    }
    ctr++;
    if (ctr >= max_packets)
    {
      ctr = 0;
		printf("Sampling Time (IMU): %2.6f\n", dt_imu);
		printf("Acceleration (IMU-z): %2.6f\n", imu_msg.acc_z);
		printf("Sampling Time (Baro): %2.6f\n", dt_baro);
		printf("Baro Pressure: %2.3f\n", baro_msg.pressure);
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


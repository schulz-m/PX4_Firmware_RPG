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
#include <uORB/topics/rpg/emergency_ekf_msg.h>

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
  struct sonar_msg_s sonar_msg;
  int sonar_sub = orb_subscribe(ORB_ID(sonar_msg));
  memset(&sonar_msg, 0, sizeof(sonar_msg));
  orb_set_interval(sonar_sub, 10); //100 Hz

  struct emergency_ekf_msg_s emergency_ekf_msg;
  memset(&emergency_ekf_msg, 0, sizeof(emergency_ekf_msg));
  int emergency_ekf_sub = orb_subscribe(ORB_ID(emergency_ekf_msg));
  orb_set_interval(emergency_ekf_sub, 10); //100 Hz

  orb_copy(ORB_ID(sensor_baro), baro_sub, &baro_msg);
  printf("First pressure: %3.3f \n",baro_msg.pressure);

  struct pollfd fds[4];
  fds[0].fd = imu_sub;
  fds[0].events = POLLIN;
  fds[1].fd = baro_sub;
  fds[1].events = POLLIN;
  fds[2].fd = sonar_sub;
  fds[2].events = POLLIN;
  fds[3].fd = emergency_ekf_sub;
  fds[3].events = POLLIN;


 float dt_imu;
 float dt_baro;
 float dt_sonar;
  int ctr = 0;
  int max_packets = 500;
  uint64_t imu_timestamp = 0;
  uint64_t baro_timestamp = 0;
  uint64_t sonar_timestamp = 0;
//  uint64_t last_msg_timestamp;

  while (!thread_should_exit)
  {
    int poll_ret = poll(fds, 4, 1000);
    if ( poll_ret <= 0 )
    {
    	printf("Timeout\n");
    }

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
    if (poll_ret > 0 && (fds[2].revents & POLLIN))
    {
    	orb_copy(ORB_ID(sonar_msg), sonar_sub, &sonar_msg);
		dt_sonar = ((float)(sonar_msg.timestamp - sonar_timestamp))/ 1.0e6f; //1000000.0f;
		sonar_timestamp = sonar_msg.timestamp;
    }
    if (poll_ret > 0 && (fds[3].revents & POLLIN))
    {
    	orb_copy(ORB_ID(emergency_ekf_msg), emergency_ekf_sub, &emergency_ekf_msg);
    }
    ctr++;
    if (ctr >= max_packets)
    {
      ctr = 0;
//		printf("Sampling Time (IMU): %2.6f\n", dt_imu);
//		printf("Acceleration (IMU-z): %2.6f\n", imu_msg.acc_z);
//		printf("Sampling Time (Baro): %2.6f\n", dt_baro);
//		printf("Baro Pressure: %2.3f\n", baro_msg.pressure);
//		printf("Sampling Time (Sonar): %2.6f\n", dt_sonar);
      printf("******************************INPUT\n\n");
		printf("Sonar Signal: %2.3f\n", sonar_msg.sonar_down);
		printf("EKF Height: %2.3f\n",emergency_ekf_msg.h_W);
		printf("EKF Time: %2.3f\n",(float)emergency_ekf_msg.timestamp/1.0e6);
		printf("p_0: %2.3f\n",(float)emergency_ekf_msg.p_0);
		printf("b_s: %2.3f\n",(float)emergency_ekf_msg.b_s);
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


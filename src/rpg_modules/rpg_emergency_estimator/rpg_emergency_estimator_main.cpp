/* Main File of Emergency State Estimation */

/* General Libaries */
//Note: Some of them might be obselete
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

/* PX4 Drivers */
#include <drivers/drv_hrt.h>
//Additional unknown drivers:
#include <drivers/drv_gpio.h>
#include <drivers/drv_rc_input.h>
//Sensor Drivers necessary:
#include <drivers/drv_accel.h> // Accelerometer
#include <drivers/drv_gyro.h> // Gyroscope
#include <drivers/drv_baro.h> // Barometer

/*uORB libaries */
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>

/* Math libaries */
#include <../lib/mathlib/mathlib.h>

// Put every h-file in here
#include "rpg_emergency_estimator_params.hpp"

//Important because of C-Code generating!
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
  thread_running = true;
  /* print welcome text */
    warnx("The emergency procedure is starting...");

   /* Vehicle Control Mode, for sensor reading detection*/
    // State Machine / State of Vehicle
   struct vehicle_control_mode_s control_mode;
   memset(&control_mode, 0, sizeof(control_mode));
   int sub_control_mode = orb_subscribe(ORB_ID(vehicle_control_mode));

   /* Define sensor reports and subscribers */
   // Find definitions in each respective driver header
   struct accel_report accel_report;
   memset(&accel_report, 0, sizeof(accel_report));
   int accel_sub = orb_subscribe(ORB_ID(sensor_accel));

   struct gyro_report gyro_report;
   memset(&gyro_report, 0, sizeof(gyro_report));
   int gyro_sub = orb_subscribe(ORB_ID(sensor_gyro));

   // This compiles, but will be used later - first only do prediction, check all outputs of topic!
//   struct baro_report baro_report;
//   memset(&baro_report, 0, sizeof(baro_report));
//   int baro_sub = orb_subscribe(ORB_ID(sensor_baro));

   /* Define publishers */

   // This Publisher gives an error ! ~~ No Printf's allowed...
//   struct vehicle_attitude_s attitude;
//   memset(&attitude, 0, sizeof(attitude));
//   orb_advert_t attitude_pub = orb_advertise(ORB_ID(vehicle_attitude), &attitude);

   /* What about this interval setting?! */
   //orb_set_interval(accel_sub, 10); //100 Hz

   // Poll Struct for Event Calling
	struct pollfd fds[2];
	fds[0].fd = accel_sub;
	fds[0].events = POLLIN;
	fds[1].fd = gyro_sub;
	fds[1].events = POLLIN;

	// Next step here: include baro

	// Debug Variables, to show 'only' every 1000th result!
   int ctr = 0;
   int max_packets = 1000;

   uint64_t timestamp;
   uint64_t last_msg_timestamp;

   // Copy more, lets try to get topic messages

  // Initializing constant parameters
  static struct rpg_emergency_estimator_params params;
  parametersInit(&params);

  // Initial States and call of function have to be implemented further on:
  // Initial State and Covariance
  float est_cov_array[9] = {0.1,0.05,0.05,0.05,5*pow(10,-5),5*pow(10,-5),5*pow(10,-5),5*pow(10,-5),params.p_0_cov};
  math::Matrix<9, 9> est_cov;
  est_cov.from_diagonal(est_cov_array);

  math::Vector<9> est_state;
  est_state(0) = 0; // Height
  est_state(1) = 0; est_state(2) = 0; est_state(3) = 0; // Velocity
  est_state(4) = 1; est_state(5) = 0; est_state(6) = 0; est_state(7) = 0; // Attitude
  est_state(8) = 970; //Pressure p_0

  // Input Noise, Process Noise and Measurement Noise
  float sigma_u_array[4] = {params.gyro_cov,params.gyro_cov,params.gyro_cov,params.acc_cov};
  math::Matrix<4,4> Sigma_u;
  Sigma_u.from_diagonal(sigma_u_array);

  float Q_array[9] = {0.05,0.01,0.01,0.2,pow(10,-6),pow(10,-6),pow(10,-6),pow(10,-6),params.p_0_cov};
  math::Matrix<9,9> Q;
  Q.from_diagonal(Q_array);

  float R_array[2] = {params.acc_cov,params.acc_cov};
  math::Matrix<2,2> R;
  R.from_diagonal(R_array);

  // Measurement
  math::Vector<2> z;

  R.print();

/* LOOP START */

  while (!thread_should_exit)
  {
	// Smaller Timout, waits until both events have been triggered
    int poll_ret = poll(fds, 2, 10);

    if (poll_ret < 0)
    {
      // poll error
    }
    else if (poll_ret == 0)
    {
    	/* check if we're in HIL - not getting sensor data is fine then */
		orb_copy(ORB_ID(vehicle_control_mode), sub_control_mode, &control_mode);

		if (!control_mode.flag_system_hil_enabled) {
			fprintf(stderr,
				"[att ekf] WARNING: Not getting sensors - sensor app running?\n");
		}
    }
    else
    {
      if (fds[0].revents & POLLIN)
      {
    	// get accelerometer
    	orb_copy(ORB_ID(sensor_accel), accel_sub, &accel_report);

    	// also get gyro
    	bool gyro_updated;
    	orb_check(gyro_sub, &gyro_updated);

    	if (gyro_updated)
    	{
    	  orb_copy(ORB_ID(sensor_gyro), gyro_sub, &gyro_report);
    	}

      }
    }



    //Printing Procedure: (DEBUG)
      ctr++;
      if (ctr >= max_packets)
      {
        ctr = 0;
        double dt = ((float)(hrt_absolute_time() - timestamp)) / ((float)max_packets) / 1000000.0f;
        printf("frequency: %4.2f\n", 1.0f / dt);
        printf("timestamps [acc, gyr]: %3.20f   %3.20f\n", accel_report.timestamp/1000000.0f, gyro_report.timestamp/1000000.0f);
        timestamp = hrt_absolute_time();
        // Show something
      }
      // Put a different check here:
//      if (ctr >= 2) {
//      // Actually only if both updated but lets see...
//        timestamp = hrt_absolute_time();
//		double dt = ((float)(timestamp - last_msg_timestamp))/ 1000000.0f;
//      }


      last_msg_timestamp = accel_report.timestamp;



      /*SECOND ROUND OF EVENTS?! */
//      if (fds[1].revents & POLLIN)
//      {
//        struct parameter_update_s updated_parameters;
////        orb_copy(ORB_ID(parameter_update), param_sub, &updated_parameters);
//
//        // update parameters
////        parametersUpdate(&params_handle, &params);
//      }

 //Loop Closure (Thread_should_exit)
  }

  //Close Publisher
//  close(rates_setpoint_pub);

  thread_running = false;
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
    printf("starting rpg_emergency_estimator\n");
    if (thread_running)
    {
      printf("rpg_emergency_estimator already running\n");
      // this is not an error
      exit(0);
    }

    thread_should_exit = false;
    emergency_estimator_task = task_spawn_cmd("rpg_emergency_estimator",
    											SCHED_DEFAULT,
    											SCHED_PRIORITY_DEFAULT, 2048, rpgEmergencyEstimatorThreadMain,
    											(const char**)argv);
    exit(0);
  }
//  (argv) ? (const char **)&argv[2] : (const char **)NULL);
  if (!strcmp(argv[1], "stop"))
  {
    printf("stopping rpg_emergency_estimator\n");
    thread_should_exit = true;
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
    exit(0);
  }
/* Finga on da trigga has more consize design... have a look :-) */
  usage("unrecognized command");
  exit(1);
}

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/rpg/imu_msg.h>
#include <uORB/topics/rpg/torques_and_thrust.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/rpg/laird_control_setpoint.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

#include "rpg_rate_controller.h"

__EXPORT int rpg_rate_controller_main(int argc, char *argv[]);

static bool thread_should_exit;
static bool thread_running = false;
static int rate_control_task;

static int rpgRateControllerThreadMain(int argc, char *argv[])
{
  struct imu_msg_s imu_msg;
  memset(&imu_msg, 0, sizeof(imu_msg));
  struct offboard_control_setpoint_s offboard_sp;
  memset(&offboard_sp, 0, sizeof(offboard_sp));
  struct offboard_control_setpoint_s laird_sp;
  memset(&laird_sp, 0, sizeof(laird_sp));
  struct torques_and_thrust_s desired_torques_and_thrust;
  memset(&desired_torques_and_thrust, 0, sizeof(desired_torques_and_thrust));

  int param_sub = orb_subscribe(ORB_ID(parameter_update));
  int offboard_setpoint_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));
  int laird_sub = orb_subscribe(ORB_ID(laird_control_setpoint));
  int imu_sub = orb_subscribe(ORB_ID(imu_msg));

  orb_advert_t rotor_thrusts_pub = orb_advertise(ORB_ID(torques_and_thrust), &desired_torques_and_thrust);

  // Limit this loop frequency to 200Hz
  orb_set_interval(imu_sub, 5);

  struct pollfd fds[2] = { {.fd = imu_sub, .events = POLLIN}, {.fd = param_sub, .events = POLLIN}};

  // Initializing parameters
  static struct rpg_rate_controller_params params;
  static struct rpg_rate_controller_params_handles params_handle;
  parametersInit(&params_handle);
  parametersUpdate(&params_handle, &params);

  thread_running = true;

  while (!thread_should_exit)
  {
    /* wait for a sensor update, check for exit condition every 500 ms */
    int ret = poll(fds, 2, 500);

    if (ret < 0)
    {
      /* poll error */
    }
    else if (ret == 0)
    {
      /* no return value, ignore */
    }
    else
    {
      /* run controller if we received a new sensor value */
      if (fds[0].revents & POLLIN)
      {
        /* get a local copy of the current sensor values */
        orb_copy(ORB_ID(imu_msg), imu_sub, &imu_msg);
        float body_rates_meas[3];
        body_rates_meas[0] = imu_msg.gyro_x;
        body_rates_meas[1] = imu_msg.gyro_y;
        body_rates_meas[2] = imu_msg.gyro_z;

        bool updated;
        orb_check(offboard_setpoint_sub, &updated);
        if (updated)
        {
          orb_copy(ORB_ID(offboard_control_setpoint), offboard_setpoint_sub, &offboard_sp);
        }

        orb_check(laird_sub, &updated);
        if (updated)
        {
          orb_copy(ORB_ID(laird_control_setpoint), laird_sub, &laird_sp);
        }

        // TODO: Some logic to choose which setpoint input to take
        float rates_thrust_sp[4] = {0.0f};
        rates_thrust_sp[0] = offboard_sp.p1;
        rates_thrust_sp[1] = offboard_sp.p2;
        rates_thrust_sp[2] = offboard_sp.p3;
        rates_thrust_sp[3] = offboard_sp.p4;

        /*
         // Choose Laird input
         rates_thrust_sp[0] = laird_sp.p1;
         rates_thrust_sp[1] = laird_sp.p2;
         rates_thrust_sp[2] = laird_sp.p3;
         rates_thrust_sp[3] = laird_sp.p4;
         */

        /*
         // *** for testing ***
         rates_thrust_sp[0] = 0.0f;
         rates_thrust_sp[1] = 0.0f;
         rates_thrust_sp[2] = 0.0f;
         rates_thrust_sp[3] = 3.0f;
        */

        // Compute torques to be applied
        float torques_and_thrust[4];
        runRateController(rates_thrust_sp, body_rates_meas, params, torques_and_thrust);

        // Publish desired rotor thrusts to be applied by the respective motor interface
        desired_torques_and_thrust.timestamp = hrt_absolute_time();
        desired_torques_and_thrust.roll_torque = torques_and_thrust[0];
        desired_torques_and_thrust.pitch_torque = torques_and_thrust[1];
        desired_torques_and_thrust.yaw_torque = torques_and_thrust[2];
        desired_torques_and_thrust.normalized_thrust = torques_and_thrust[3];
        orb_publish(ORB_ID(torques_and_thrust), rotor_thrusts_pub, &desired_torques_and_thrust);
      }

      /* only update parameters if they changed */
      if (fds[1].revents & POLLIN)
      {
        /* read from param to clear updated flag */
        struct parameter_update_s updated_parameters;
        orb_copy(ORB_ID(parameter_update), param_sub, &updated_parameters);

        // update parameters
        parametersUpdate(&params_handle, &params);
      }
    }
  }

  // Kill motors
  desired_torques_and_thrust.timestamp = hrt_absolute_time();
  desired_torques_and_thrust.roll_torque = 0.0f;
  desired_torques_and_thrust.pitch_torque = 0.0f;
  desired_torques_and_thrust.yaw_torque = 0.0f;
  desired_torques_and_thrust.normalized_thrust = 0.0f;
  orb_publish(ORB_ID(torques_and_thrust), rotor_thrusts_pub, &desired_torques_and_thrust);

  close(param_sub);
  close(offboard_setpoint_sub);
  close(laird_sub);
  close(imu_sub);
  close(rotor_thrusts_pub);

  thread_running = false;

  exit(0);
}

static void usage(const char *reason)
{
  if (reason)
    fprintf(stderr, "%s\n", reason);

  fprintf(stderr, "usage: rpg_rate_controller {start|status|stop}\n");
  exit(1);
}

int rpg_rate_controller_main(int argc, char *argv[])
{
  if (argc < 1)
    usage("missing command");

  if (!strcmp(argv[1], "start"))
  {
    printf("starting rpg_rate_controller\n");
    if (thread_running)
    {
      printf("rpg_rate_controller already running\n");
      // this is not an error
      exit(0);
    }

    thread_should_exit = false;
    rate_control_task = task_spawn_cmd("rpg_rate_controller",
    SCHED_DEFAULT,
                                       SCHED_PRIORITY_MAX - 15, 1500, rpgRateControllerThreadMain,
                                       (argv) ? (const char **)&argv[2] : (const char **)NULL);
    exit(0);
  }

  if (!strcmp(argv[1], "stop"))
  {
    printf("stopping rpg_rate_controller\n");
    thread_should_exit = true;
    exit(0);
  }

  if (!strcmp(argv[1], "status"))
  {
    if (thread_running)
    {
      warnx("rpg_rate_controller is running");
      exit(0);
    }
    else
    {
      warnx("rpg_rate_controller not started");
      exit(1);
    }
    exit(0);
  }

  usage("unrecognized command");
  exit(1);
}

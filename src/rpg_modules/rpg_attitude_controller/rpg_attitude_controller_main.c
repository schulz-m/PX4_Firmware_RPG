#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/laird_control_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

#include "rpg_attitude_controller.h"

__EXPORT int rpg_attitude_controller_main(int argc, char *argv[]);

static bool thread_should_exit;
static bool thread_running = false;
static int attitude_control_task;

static int rpgAttitudeControllerThreadMain(int argc, char *argv[])
{
  thread_running = true;

  struct vehicle_attitude_s attitude;
  memset(&attitude, 0, sizeof(attitude));
  attitude.q[0] = 1.0f;
  struct vehicle_attitude_setpoint_s attitude_sp;
  memset(&attitude_sp, 0, sizeof(attitude_sp));
  attitude_sp.q_d[0] = 1.0f;
  struct offboard_control_setpoint_s desired_body_rates;
  memset(&desired_body_rates, 0, sizeof(desired_body_rates));

  int param_sub = orb_subscribe(ORB_ID(parameter_update));
  int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
  int att_setpoint_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));

  orb_advert_t rates_setpoint_pub = orb_advertise(ORB_ID(offboard_control_setpoint), &desired_body_rates);

  struct pollfd fds[2] = { {.fd = att_sub, .events = POLLIN}, {.fd = param_sub, .events = POLLIN}};

  // Initializing parameters
  static struct rpg_attitude_controller_params params;
  static struct rpg_attitude_controller_params_handles params_handle;
  parametersInit(&params_handle);
  parametersUpdate(&params_handle, &params);

  while (!thread_should_exit)
  {
    int ret = poll(fds, 2, 500);

    if (ret < 0)
    {
      // poll error
    }
    else if (ret == 0)
    {
      // no return value, ignore
    }
    else
    {
      if (fds[0].revents & POLLIN)
      {
        orb_copy(ORB_ID(vehicle_attitude), att_sub, &attitude);

        // Create an array with the current attitude quaternion
        float attitude_est[4] = {0.0f};
        attitude_est[0] = attitude.q[0];
        attitude_est[1] = attitude.q[1];
        attitude_est[2] = attitude.q[2];
        attitude_est[3] = attitude.q[3];

        // Make local copy of attitude setpoint
        orb_copy(ORB_ID(vehicle_attitude_setpoint), att_setpoint_sub, &attitude_sp);

        // Create an array with the desired attitude quaternion
        float attitude_des[4] = {0.0f};
        attitude_des[0] = attitude_sp.q_d[0];
        attitude_des[1] = attitude_sp.q_d[1];
        attitude_des[2] = attitude_sp.q_d[2];
        attitude_des[3] = attitude_sp.q_d[3];

        // Run attitude controller
        float body_rates_des[3] = {0.0f};
        runAttitudeController(attitude_des, attitude_est, params, body_rates_des);

        // Publish desired body rates to be processed by the rate controller
        desired_body_rates.timestamp = hrt_absolute_time();
        desired_body_rates.p1 = body_rates_des[0];
        desired_body_rates.p2 = body_rates_des[1];
        desired_body_rates.p3 = body_rates_des[2];
        desired_body_rates.p4 = attitude_sp.thrust;
        orb_publish(ORB_ID(offboard_control_setpoint), rates_setpoint_pub, &desired_body_rates);
      }

      if (fds[1].revents & POLLIN)
      {
        struct parameter_update_s updated_parameters;
        orb_copy(ORB_ID(parameter_update), param_sub, &updated_parameters);

        // update parameters
        parametersUpdate(&params_handle, &params);
      }
    }
  }

  close(param_sub);
  close(att_sub);
  close(att_setpoint_sub);

  close(rates_setpoint_pub);

  thread_running = false;
  exit(0);
}

static void usage(const char *reason)
{
  if (reason)
    fprintf(stderr, "%s\n", reason);

  fprintf(stderr, "usage: rpg_attitude_controller {start|status|stop}\n");
  exit(1);
}

int rpg_attitude_controller_main(int argc, char *argv[])
{
  if (argc < 1)
    usage("missing command");

  if (!strcmp(argv[1], "start"))
  {
    printf("starting rpg_attitude_controller\n");
    if (thread_running)
    {
      printf("rpg_attitude_controller already running\n");
      // this is not an error
      exit(0);
    }

    thread_should_exit = false;
    attitude_control_task = task_spawn_cmd("rpg_attitude_controller",
    SCHED_DEFAULT,
                                           SCHED_PRIORITY_MAX - 15, 2048, rpgAttitudeControllerThreadMain,
                                           (argv) ? (const char **)&argv[2] : (const char **)NULL);
    exit(0);
  }

  if (!strcmp(argv[1], "stop"))
  {
    printf("stopping rpg_attitude_controller\n");
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

  usage("unrecognized command");
  exit(1);
}

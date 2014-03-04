#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <debug.h>
#include <time.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/laird_control_setpoint.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

#include "rpg_rate_controller.h"

__EXPORT int rpg_rate_controller_main(int argc, char *argv[]);

static int parameters_init(struct rpg_rate_controller_params_handles *h);
static int parameters_update(const struct rpg_rate_controller_params_handles *h, struct rpg_rate_controller_params *p);

static bool thread_should_exit;
static bool thread_running = false;
static int rate_control_task;

static int rpg_rate_controller_thread_main(int argc, char *argv[])
{
  struct sensor_combined_s sensor_raw;
  memset(&sensor_raw, 0, sizeof(sensor_raw));
  struct offboard_control_setpoint_s offboard_sp;
  memset(&offboard_sp, 0, sizeof(offboard_sp));
  struct offboard_control_setpoint_s laird_sp;
  memset(&laird_sp, 0, sizeof(laird_sp));

  int param_sub = orb_subscribe(ORB_ID(parameter_update));
  int offboard_setpoint_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));
  int laird_sub = orb_subscribe(ORB_ID(laird_control_setpoint));
  int sensor_sub = orb_subscribe(ORB_ID(sensor_combined));

  // Uncomment the line below to rate limit this loop
  // orb_set_interval(sensor_sub, 5);

  struct pollfd fds[2] = {
    { .fd = sensor_sub, .events = POLLIN },
    { .fd = param_sub, .events = POLLIN }
  };

  // TODO: Send zero motor commands

  // Initializing parameters
  static struct rpg_rate_controller_params params;
  static struct rpg_rate_controller_params_handles params_handle;
  parameters_init(&params_handle);
  parameters_update(&params_handle, &params);

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
        orb_copy(ORB_ID(sensor_combined), sensor_sub, &sensor_raw);

        bool updated;
        orb_check(offboard_setpoint_sub, &updated);
        if (updated) {
                orb_copy(ORB_ID(offboard_control_setpoint), offboard_setpoint_sub, &offboard_sp);
        }

        orb_check(laird_sub, &updated);
        if (updated) {
                orb_copy(ORB_ID(laird_control_setpoint), laird_sub, &laird_sp);
        }

        // TODO: Some logic to choose which setpoint input to take
        float rates_sp[4];

        // Compute torques to be applied
        uint16_t motor_commands[4];
        run_rate_controller(rates_sp, sensor_raw.gyro_rad_s, params, motor_commands);

        // TODO: Set motor commands
      }

      /* only update parameters if they changed */
      if (fds[1].revents & POLLIN)
      {
        /* read from param to clear updated flag */
        struct parameter_update_s updated_parameters;
        orb_copy(ORB_ID(parameter_update), param_sub, &updated_parameters);

        // update parameters
        parameters_update(&params_handle, &params);
      }
    }
  }

  // TODO: Kill motors

  close(param_sub);
  close(offboard_setpoint_sub);
  close(laird_sub);
  close(sensor_sub);

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
    if (thread_running)
    {
      printf("attitude_estimator_ekf already running\n");
      /* this is not an error */
      exit(0);
    }

    thread_should_exit = false;
    rate_control_task = task_spawn_cmd("attitude_estimator_ekf",
                                    SCHED_DEFAULT,
                                    SCHED_PRIORITY_MAX - 15,
                                    2048,
                                    rpg_rate_controller_thread_main,
                                    NULL);
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

  usage("unrecognized command");
  exit(1);
}

static int parameters_init(struct rpg_rate_controller_params_handles *h)
{
  h->mass = param_find("RPG_MASS");
  h->arm_length = param_find("RPG_L");

  h->moment_of_inertia_x = param_find("RPG_IXX");
  h->moment_of_inertia_y = param_find("RPG_IYY");
  h->moment_of_inertia_z = param_find("RPG_IZZ");

  h->rotor_drag_coeff = param_find("RPG_KAPPA");

  h->tau_pq = param_find("RPG_TAU_PQ");
  h->tau_r = param_find("RPG_TAU_R");

  h->gamma_1 = param_find("RPG_GAMMA1");
  h->gamma_2 = param_find("RPG_GAMMA2");
  h->gamma_3 = param_find("RPG_GAMMA3");
  h->gamma_4 = param_find("RPG_GAMMA4");

  h->gyro_bias_x = param_find("SENS_GYRO_XOFF");
  h->gyro_bias_y = param_find("SENS_GYRO_YOFF");
  h->gyro_bias_z = param_find("SENS_GYRO_ZOFF");

  return OK;
}

static int parameters_update(const struct rpg_rate_controller_params_handles *h, struct rpg_rate_controller_params *p)
{
  param_get(h->mass, &(p->mass));
  param_get(h->arm_length, &(p->arm_length));

  param_get(h->moment_of_inertia_x, &(p->moment_of_inertia_x));
  param_get(h->moment_of_inertia_y, &(p->moment_of_inertia_y));
  param_get(h->moment_of_inertia_z, &(p->moment_of_inertia_z));

  param_get(h->rotor_drag_coeff, &(p->rotor_drag_coeff));

  param_get(h->tau_pq, &(p->tau_pq));
  param_get(h->tau_r, &(p->tau_r));

  param_get(h->gamma_1, &(p->gamma_1));
  param_get(h->gamma_2, &(p->gamma_2));
  param_get(h->gamma_3, &(p->gamma_3));
  param_get(h->gamma_4, &(p->gamma_4));

  param_get(h->gyro_bias_x, &(p->gyro_bias_x));
  param_get(h->gyro_bias_y, &(p->gyro_bias_y));
  param_get(h->gyro_bias_z, &(p->gyro_bias_z));

  return OK;
}

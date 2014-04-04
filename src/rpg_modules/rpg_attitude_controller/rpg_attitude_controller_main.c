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
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/roll_pitch_yawrate_thrust_setpoint.h>

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

#include "rpg_attitude_controller.h"

__EXPORT int rpg_attitude_controller_main(int argc, char *argv[]);

static bool thread_should_exit;
static bool thread_running = false;
static int attitude_control_task;

void PX4EulerAnglesToRPGQuaternion(float q[], const float roll, const float pitch, const float yaw)
{
  // Conversion to RPG Euler angles
  float R_3_2 = sin(roll);
  float R_3_3 = cos(pitch) * cos(roll);
  float R_3_1 = cos(roll) * sin(pitch);
  float R_2_1 = -(cos(pitch) * sin(yaw) + cos(yaw) * sin(pitch) * sin(roll));
  float R_1_1 = cos(pitch) * cos(yaw) - sin(pitch) * sin(roll) * sin(yaw);

  float roll_rpg = atan2(R_3_2, R_3_3);
  float pitch_rpg = -asin(R_3_1);
  float yaw_rpg = atan2(R_2_1, R_1_1);

  // Conversion from RPG Euler angles to Quaternion
  float r = roll_rpg / 2.0f;
  float p = pitch_rpg / 2.0f;
  float y = yaw_rpg / 2.0f;

  q[0] = cos(r) * cos(p) * cos(y) + sin(r) * sin(p) * sin(y);
  q[1] = sin(r) * cos(p) * cos(y) - cos(r) * sin(p) * sin(y);
  q[2] = cos(r) * sin(p) * cos(y) + sin(r) * cos(p) * sin(y);
  q[3] = cos(r) * cos(p) * sin(y) - sin(r) * sin(p) * cos(y);
}

static int rpgAttitudeControllerThreadMain(int argc, char *argv[])
{
  thread_running = true;

  struct vehicle_attitude_s attitude;
  memset(&attitude, 0, sizeof(attitude));
  struct roll_pitch_yawrate_thrust_setpoint_s att_cont_sp;
  memset(&att_cont_sp, 0, sizeof(att_cont_sp));
  struct offboard_control_setpoint_s desired_body_rates;
  memset(&desired_body_rates, 0, sizeof(desired_body_rates));

  int param_sub = orb_subscribe(ORB_ID(parameter_update));
  int att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
  int att_cont_setpoint_sub = orb_subscribe(ORB_ID(roll_pitch_yawrate_thrust_setpoint));

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
        PX4EulerAnglesToRPGQuaternion(attitude_est, attitude.roll, attitude.pitch, attitude.yaw);

        //printf("RPY: %.4f  %.4f  %.4f \n", attitude.roll, attitude.pitch, attitude.yaw);
        //printf("attitude: %.4f  %.4f  %.4f  %.4f \n", attitude_est[0], attitude_est[1], attitude_est[2], attitude_est[3]);

        // Make local copy of attitude setpoint if updated
        bool updated;
        orb_check(att_cont_setpoint_sub, &updated);
        if (updated)
        {
          orb_copy(ORB_ID(roll_pitch_yawrate_thrust_setpoint), att_cont_setpoint_sub, &att_cont_sp);
        }

        // Run attitude controller
        float body_rates_cmds[3] = {0.0f};
        runAttitudeController(att_cont_sp.roll, att_cont_sp.pitch, att_cont_sp.yaw_rate, attitude_est, params, body_rates_cmds);

        //printf("body_rates_des: %.4f  %.4f  %.4f \n", body_rates_des[0], body_rates_des[1], body_rates_des[2]);

        // Publish desired body rates to be processed by the rate controller
        desired_body_rates.timestamp = hrt_absolute_time();
        desired_body_rates.p1 = body_rates_cmds[0];
        desired_body_rates.p2 = -body_rates_cmds[1]; // Conversion to px4 coordinate frame
        desired_body_rates.p3 = -body_rates_cmds[2]; // Conversion to px4 coordinate frame
        desired_body_rates.p4 = att_cont_sp.normalized_thrust;
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
  close(att_cont_setpoint_sub);

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

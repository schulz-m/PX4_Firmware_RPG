#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
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
#include "ardrone_motor_interface.h"

__EXPORT int rpg_rate_controller_main(int argc, char *argv[]);

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

  // Open uart to motors
  // TODO: Also make this a parameter as in the ardrone interface
  char *device = "/dev/ttyS1";
  // enable UART, writes potentially an empty buffer, but multiplexing is disabled
  static int ardrone_write;
  struct termios uart_config_original;
  int gpios;

  // open ardrone motor ports
  if (open_ardrone_motor_ports(device, &ardrone_write, &uart_config_original, &gpios) != 0)
  {
    thread_should_exit = true;
  }

  // Send zero commands to make sure rotors are not spinning
  ardrone_write_motor_commands(ardrone_write, 0, 0, 0, 0);

  // Check if "x-configuration" should be used or not (otherwise "+-configuration")
  bool use_x_configuration = false;
  if (argc > 1)
  {
    if (strcmp(argv[1], "-x") == 0)
    {
      use_x_configuration = true;
    }
  }

  // Limit this loop frequency to 200Hz
  orb_set_interval(sensor_sub, 5);

  struct pollfd fds[2] = { {.fd = sensor_sub, .events = POLLIN}, {.fd = param_sub, .events = POLLIN}};

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

        // *** for testing ***
        rates_thrust_sp[0] = 0.0f;
        rates_thrust_sp[1] = 0.0f;
        rates_thrust_sp[2] = 0.0f;
        rates_thrust_sp[3] = 5.0f;

        // Compute torques to be applied
        uint16_t motor_commands[4];
        run_rate_controller(rates_thrust_sp, sensor_raw.gyro_rad_s, params, use_x_configuration, motor_commands);

        // Set motor commands
        printf("%3.3d  %3.3d   %3.3d   %3.3d\n", motor_commands[0], motor_commands[1], motor_commands[2],
               motor_commands[3]);
        ardrone_write_motor_commands(ardrone_write, motor_commands[0], motor_commands[1], motor_commands[2],
                                     motor_commands[3]);

        // TODO: Publish single rotor thrusts as uorb topic (this is interesting for system id) maybe it makes sense to create a custom message for this
        // thrust_message.rotor_1 = convert_motor_command_to_thrust(motor_commands[0]);
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

  // Kill motors
  ardrone_write_motor_commands(ardrone_write, 0, 0, 0, 0);

  close_ardrone_motor_ports(&ardrone_write, &uart_config_original, &gpios);

  close(param_sub);
  close(offboard_setpoint_sub);
  close(laird_sub);
  close(sensor_sub);

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
                                       SCHED_PRIORITY_MAX - 15, 2048, rpg_rate_controller_thread_main,
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

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <poll.h>
#include <systemlib/err.h>
#include <sys/prctl.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/torques_and_thrust.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/motor_inputs.h>

#include <systemlib/systemlib.h>

#include "ardrone_motor_interface.h"

__EXPORT int rpg_ardrone_interface_main(int argc, char *argv[]);

static bool thread_should_exit = false;
static bool thread_running = false;
static int rpg_ardrone_interface_task;

static int ardrone_interface_thread_main(int argc, char *argv[])
{
  thread_running = true;

  // Initialize structs
  struct torques_and_thrust_s desired_torques_and_thrust;
  memset(&desired_torques_and_thrust, 0, sizeof(desired_torques_and_thrust));
  struct motor_inputs_s motor_inputs;
  memset(&motor_inputs, 0, sizeof(motor_inputs));

  // Subscribers
  int torques_and_thrust_sub = orb_subscribe(ORB_ID(torques_and_thrust));
  int param_sub = orb_subscribe(ORB_ID(parameter_update));

  struct pollfd fds[2] = { {.fd = torques_and_thrust_sub, .events = POLLIN}, {.fd = param_sub, .events = POLLIN}};

  // Publisher of actually applied motor commands
  orb_advert_t motor_inputs_pub = orb_advertise(ORB_ID(motor_inputs), &motor_inputs);

  // Read device name from command line input
  bool use_x_configuration = false;
  char *device = "/dev/ttyS1";
  for (int i = 0; i < argc && argv[i]; i++)
  {
    if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0)
    { //device set
      if (argc > i + 1)
      {
        device = argv[i + 1];
        printf("device set to %s \n", device);
      }
      else
      {
        thread_running = false;
        fprintf(stderr, "usage: rpg_ardrone_interface {start|stop|status} [-d <UART>]\n\n");
      }
    }
    // Check if "x-configuration" should be used or not (otherwise "+-configuration")
    if (strcmp(argv[i], "-x") == 0)
    {
      use_x_configuration = true;
      printf("using x configuration \n");
    }
  }

  // open ardrone motor ports
  static int ardrone_write;
  static struct termios uart_config_original;
  static int gpios;
  if (open_ardrone_motor_ports(device, &ardrone_write, &uart_config_original, &gpios) != 0)
  {
    thread_should_exit = true;
  }

  // Send zero commands to make sure rotors are not spinning
  ardrone_write_motor_commands(ardrone_write, 0, 0, 0, 0);

  // Initializing parameters
  static struct rpg_ardrone_interface_params params;
  static struct rpg_ardrone_interface_params_handles params_handle;
  parameters_init(&params_handle);
  parameters_update(&params_handle, &params);

  while (!thread_should_exit)
  {
    int ret = poll(fds, 1, 500);

    if (ret > 0)
    {
      if (fds[0].revents & POLLIN)
      {
        orb_copy(ORB_ID(torques_and_thrust), torques_and_thrust_sub, &desired_torques_and_thrust);

        uint16_t motor_commands[4];
        compute_motor_commands(motor_commands, desired_torques_and_thrust, use_x_configuration, params);

        ardrone_write_motor_commands(ardrone_write, motor_commands[0], motor_commands[1], motor_commands[2],
                                     motor_commands[3]);

        // Publish the motor inputs as uorb topic
        motor_inputs.timestamp = hrt_absolute_time();
        motor_inputs.motor_inputs[0] = motor_commands[0];
        motor_inputs.motor_inputs[1] = motor_commands[1];
        motor_inputs.motor_inputs[2] = motor_commands[2];
        motor_inputs.motor_inputs[3] = motor_commands[3];
        orb_publish(ORB_ID(motor_inputs), motor_inputs_pub, &motor_inputs);
      }

      // only update parameters if they changed
      if (fds[1].revents & POLLIN)
      {
        // read from param to clear updated flag
        struct parameter_update_s updated_parameters;
        orb_copy(ORB_ID(parameter_update), param_sub, &updated_parameters);

        // update parameters
        parameters_update(&params_handle, &params);
      }
    }
    // run at approximately 200 Hz
    usleep(4500);
  }

  // Kill motors
  ardrone_write_motor_commands(ardrone_write, 0, 0, 0, 0);

  // Close motor ports
  close_ardrone_motor_ports(&ardrone_write, &uart_config_original, &gpios);

  close(torques_and_thrust_sub);
  close(param_sub);
  close(motor_inputs_pub);

  thread_running = false;
  exit(0);
}

static void usage(const char *reason)
{
  if (reason)
    fprintf(stderr, "%s\n", reason);
  fprintf(stderr, "usage: rpg_ardrone_interface {start|stop|status} [-d <UART>]\n\n");
  exit(1);
}

int rpg_ardrone_interface_main(int argc, char *argv[])
{
  if (argc < 1)
    usage("missing command");

  if (!strcmp(argv[1], "start"))
  {

    if (thread_running)
    {
      printf("rpg_ardrone_interface already running\n");
      /* this is not an error */
      exit(0);
    }

    thread_should_exit = false;
    rpg_ardrone_interface_task = task_spawn_cmd("rpg_ardrone_interface",
    SCHED_DEFAULT,
                                                SCHED_PRIORITY_MAX - 15, 2048, ardrone_interface_thread_main,
                                                (argv) ? (const char **)&argv[2] : (const char **)NULL);
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
      printf("rpg_ardrone_interface is running\n");
    }
    else
    {
      printf("rpg_ardrone_interface not started\n");
    }
    exit(0);
  }

  usage("unrecognized command");
  exit(1);
}

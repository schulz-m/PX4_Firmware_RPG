#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <time.h>
#include <sys/prctl.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/offboard_control_setpoint.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/rpg/rpg_vehicle_status.h>

#include "buzzer_helper.h"
#include "state_machine_helper.h"

#include <systemlib/systemlib.h>
#include <systemlib/param/param.h>

extern "C" __EXPORT int rpg_commander_main(int argc, char *argv[]);

static bool thread_should_exit;
static bool thread_running = false;
static int commander_task;

void *rpg_commander_low_prio_loop(void *arg);

static int rpgCommanderThreadMain(int argc, char *argv[])
{
  // Starting low priority loop
  pthread_attr_t commander_low_prio_attr;
  pthread_attr_init(&commander_low_prio_attr);
  pthread_attr_setstacksize(&commander_low_prio_attr, 1500);
  pthread_t rpg_commander_low_prio_thread;
  struct sched_param param;
  (void)pthread_attr_getschedparam(&commander_low_prio_attr, &param);
  param.sched_priority = SCHED_PRIORITY_DEFAULT - 50;
  (void)pthread_attr_setschedparam(&commander_low_prio_attr, &param);
  pthread_create(&rpg_commander_low_prio_thread, &commander_low_prio_attr, rpg_commander_low_prio_loop, NULL);
  pthread_attr_destroy(&commander_low_prio_attr);

  if (buzzer_init() != OK) {
    warnx("ERROR: Failed to initialize buzzer");
  }

  // Subscribe to offboard commands
  int offboard_setpoint_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));
  struct offboard_control_setpoint_s offboard_sp;
  memset(&offboard_sp, 0, sizeof(offboard_sp));

  // Subscribe to battery
  struct battery_status_s battery_status;
  int battery_sub = orb_subscribe(ORB_ID(battery_status));
  memset(&battery_status, 0, sizeof(battery_status));
  orb_set_interval(battery_sub, 1000); // 1 Hz

  // Publish rpg vehicle status
  struct rpg_vehicle_status_s rpg_vehicle_status;
  memset(&rpg_vehicle_status, 0, sizeof(rpg_vehicle_status));
  orb_advert_t rpg_vehicle_status_pub = orb_advertise(ORB_ID(rpg_vehicle_status), &rpg_vehicle_status);

  struct pollfd fds[2];
  fds[0].fd = offboard_setpoint_sub;
  fds[0].events = POLLIN;
  fds[1].fd = battery_sub;
  fds[1].events = POLLIN;

  // Set initial state to LANDED
  states_t commander_state = LANDED;
  battery_states_t battery_state = GOOD;

  rpg_vehicle_status.commander_state = commander_state;
  rpg_vehicle_status.battery_state = battery_state;
  orb_publish(ORB_ID(rpg_vehicle_status), rpg_vehicle_status_pub, &rpg_vehicle_status);

  hrt_abstime time_last_offboard_cmd_received = 0;

  thread_running = true;
  while (!thread_should_exit)
  {
    // waiting a maximum of 10ms for an offboard control command message
    // This should give us a 100Hz rate of checking wether we receive control commands
    int ret = poll(fds, 2, 10);

    if (ret <= 0)
    {
      // did not receive a message, just continue
    }
    else
    {
      if (fds[0].revents & POLLIN)
      {
        orb_copy(ORB_ID(offboard_control_setpoint), offboard_setpoint_sub, &offboard_sp);
        time_last_offboard_cmd_received = hrt_absolute_time();
      }
      if (fds[1].revents & POLLIN)
      {
        orb_copy(ORB_ID(battery_status), battery_sub, &battery_status);
        updateBatteryStateMachine(battery_state, battery_status.voltage_filtered_v);
        rpg_vehicle_status.battery_state = battery_state;
      }
    }

    // Update commander state machine
    switch (commander_state)
    {
      case LANDED:
        // check if thrust is big enough -> FLYING
        if (!isThrustCmdZero(offboard_sp))
        {
          if (battery_state == GOOD) // Only allow transition if battery is OK
          {
            commander_state = FLYING;
            // Send out uorb message
            rpg_vehicle_status.commander_state = commander_state;
            orb_publish(ORB_ID(rpg_vehicle_status), rpg_vehicle_status_pub, &rpg_vehicle_status);
          }
        }
        break;
      case FLYING:
        // check if we did not receive a recent offboard control command -> EMERGENCY_LANDING
        if (!isOffboardCmdTimeValid(time_last_offboard_cmd_received))
        {
          commander_state = EMERGENCY_LANDING;
          // Send out uorb message
          rpg_vehicle_status.commander_state = commander_state;
          orb_publish(ORB_ID(rpg_vehicle_status), rpg_vehicle_status_pub, &rpg_vehicle_status);
        }
        // Check if battery voltage is critical -> EMERGENCY_LANDING
        if (battery_state == CRITICAL)
        {
          commander_state = EMERGENCY_LANDING;
          // Send out uorb message
          rpg_vehicle_status.commander_state = commander_state;
          orb_publish(ORB_ID(rpg_vehicle_status), rpg_vehicle_status_pub, &rpg_vehicle_status);
        }
        // else check if thrust is zero again -> LANDED
        else if (isThrustCmdZero(offboard_sp))
        {
          commander_state = LANDED;
          // Send out uorb message
          rpg_vehicle_status.commander_state = commander_state;
          orb_publish(ORB_ID(rpg_vehicle_status), rpg_vehicle_status_pub, &rpg_vehicle_status);
        }
        break;
      case EMERGENCY_LANDING:
        // check if we recently received an offboard command again -> FLYING
        if (isOffboardCmdTimeValid(time_last_offboard_cmd_received))
        {
          // Only allow transition if battery is NOT CRITICAL
          // If EMERGENCY_LANDING was triggered by critical battery voltage there is no escape from it
          if (battery_state != CRITICAL)
          {
            commander_state = FLYING;
            // Send out uorb message
            rpg_vehicle_status.commander_state = commander_state;
            orb_publish(ORB_ID(rpg_vehicle_status), rpg_vehicle_status_pub, &rpg_vehicle_status);
          }
        }
        // TODO: check if emergency landing is finished -> LANDED
        break;
    }

    // Testing
    if (commander_state == LANDED)
      printf("State: LANDED           ");
    if (commander_state == FLYING)
      printf("State: FLYING           ");
    if (commander_state == EMERGENCY_LANDING)
      printf("State: EMERGENCY_LANDING");
    if (battery_state == GOOD)
      printf("Battery: GOOD V: %2.2f\n",battery_status.voltage_filtered_v);
    if (battery_state == LOW)
      printf("Battery: LOW V: %2.2f\n",battery_status.voltage_filtered_v);
    if (battery_state == CRITICAL)
      printf("Battery: CRITICAL V: %2.2f\n",battery_status.voltage_filtered_v);
    if (battery_state == INVALID)
      printf("Battery: INVALID V: %2.2f\n",battery_status.voltage_filtered_v);
  }

  buzzer_deinit();
  close (offboard_setpoint_sub);
  close(battery_sub);

  thread_running = false;

  exit(0);
}

void *rpg_commander_low_prio_loop(void *arg)
{
  // Set thread name
  prctl(PR_SET_NAME, "rpg_commander_low_prio", getpid());

  // Subscribe to command topic
  int cmd_sub = orb_subscribe(ORB_ID(vehicle_command));
  struct vehicle_command_s cmd;
  memset(&cmd, 0, sizeof(cmd));
  orb_set_interval(cmd_sub, 100); // 10 Hz

  struct pollfd fds[1];
  fds[0].fd = cmd_sub;
  fds[0].events = POLLIN;

  while (!thread_should_exit)
  {
    int pret = poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 200);

    // timed out or poll error
    if (pret <= 0)
    {
      continue;
    }

    // if we reach here, we have a valid command
    orb_copy(ORB_ID(vehicle_command), cmd_sub, &cmd);

    switch (cmd.command)
    {
      case VEHICLE_CMD_PREFLIGHT_STORAGE:
      {
        // Check if parameters should be written to sd card or eeprom
        if (((int)(cmd.param7)) == 0)
        {
          param_set_default_file("/eeprom/parameters"); // write to eeprom
        }
        if (((int)(cmd.param7)) == 1)
        {
          param_set_default_file("/fs/microsd/parameters"); // write to sd card
        }

        if (((int)(cmd.param1)) == 0) // Loading parameters
        {
          int ret = param_load_default();

          if (ret == OK)
          {
            tune_positive(true);
          }
          else
          {
            tune_negative(true);
          }

        }
        else if (((int)(cmd.param1)) == 1) // Writing parameters
        {
          int ret = param_save_default();

          if (ret == OK)
          {
            tune_positive(true);
          }
          else
          {
            tune_negative(true);
          }
        }

        break;
      }
    }
  }

  close(cmd_sub);

  return NULL;
}

static void usage(const char *reason)
{
  if (reason)
    fprintf(stderr, "%s\n", reason);

  fprintf(stderr, "usage: rpg_commander {start|status|stop}\n");
  exit(1);
}

int rpg_commander_main(int argc, char *argv[])
{
  if (argc < 1)
    usage("missing command");

  if (!strcmp(argv[1], "start"))
  {
    printf("starting rpg_commander\n");
    if (thread_running)
    {
      printf("rpg_commander already running\n");
      // this is not an error
      exit(0);
    }

    thread_should_exit = false;
    commander_task = task_spawn_cmd("rpg_commander",
    SCHED_DEFAULT,
                                    SCHED_PRIORITY_MAX - 40, 2048, rpgCommanderThreadMain,
                                    (argv) ? (const char **)&argv[2] : (const char **)NULL);
    exit(0);
  }

  if (!strcmp(argv[1], "stop"))
  {
    printf("stopping rpg_commander\n");
    thread_should_exit = true;
    exit(0);
  }

  if (!strcmp(argv[1], "status"))
  {
    if (thread_running)
    {
      warnx("rpg_commander is running");
      exit(0);
    }
    else
    {
      warnx("rpg_commander not started");
      exit(1);
    }
    exit(0);
  }

  usage("unrecognized command");
  exit(1);
}

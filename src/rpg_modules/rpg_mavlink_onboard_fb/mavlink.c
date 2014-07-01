/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink.c
 * MAVLink 1.0 protocol implementation.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#include <nuttx/config.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <fcntl.h>
#include <mqueue.h>
#include <string.h>
#include "mavlink_bridge_header.h"
#include <drivers/drv_hrt.h>
#include <time.h>
#include <float.h>
#include <unistd.h>
#include <nuttx/sched.h>
#include <sys/prctl.h>
#include <termios.h>
#include <errno.h>
#include <stdlib.h>
#include <poll.h>

#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include "orb_topics.h"
#include "util.h"

#include "mavlink_parameters.h"

//Additional Drivers:
/*********************************/
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

//State Estimation Publish:
#include <uORB/topics/rpg/emergency_ekf_msg.h>


/*********************************/

__EXPORT int rpg_mavlink_onboard_fb_main(int argc, char *argv[]);

static int rpg_mavlink_fb_thread_main(int argc, char *argv[]);

/* thread state */
volatile bool thread_should_exit = false;
static volatile bool thread_running = false;
static int mavlink_task;

/* pthreads */
static pthread_t receive_thread;

/* terminate MAVLink on user request - disabled by default */
static bool mavlink_link_termination_allowed = false;

mavlink_system_t mavlink_system = {100, 50, MAV_TYPE_QUADROTOR, 0, 0, 0}; // System ID, 1-255, Component/Subsystem ID, 1-255

/* XXX not widely used */
uint8_t chan = MAVLINK_COMM_0;

/* XXX probably should be in a header... */
extern pthread_t receive_start(int uart);

bool mavlink_hil_enabled = false;

/* protocol interface */
static int uart;
static int baudrate;
bool gcs_link = true;

/* interface mode */
static enum
{
  MAVLINK_INTERFACE_MODE_OFFBOARD, MAVLINK_INTERFACE_MODE_ONBOARD
} mavlink_link_mode = MAVLINK_INTERFACE_MODE_OFFBOARD;

static void mavlink_update_system(void);
static int mavlink_open_uart(int baudrate, const char *uart_name, struct termios *uart_config_original, bool *is_usb);
static void usage(void);

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int mavlink_open_uart(int baud, const char *uart_name, struct termios *uart_config_original, bool *is_usb)
{
  /* process baud rate */
  int speed;

  switch (baud)
  {
    case 0:
      speed = B0;
      break;

    case 50:
      speed = B50;
      break;

    case 75:
      speed = B75;
      break;

    case 110:
      speed = B110;
      break;

    case 134:
      speed = B134;
      break;

    case 150:
      speed = B150;
      break;

    case 200:
      speed = B200;
      break;

    case 300:
      speed = B300;
      break;

    case 600:
      speed = B600;
      break;

    case 1200:
      speed = B1200;
      break;

    case 1800:
      speed = B1800;
      break;

    case 2400:
      speed = B2400;
      break;

    case 4800:
      speed = B4800;
      break;

    case 9600:
      speed = B9600;
      break;

    case 19200:
      speed = B19200;
      break;

    case 38400:
      speed = B38400;
      break;

    case 57600:
      speed = B57600;
      break;

    case 115200:
      speed = B115200;
      break;

    case 230400:
      speed = B230400;
      break;

    case 460800:
      speed = B460800;
      break;

    case 921600:
      speed = B921600;
      break;

    default:
      fprintf(
          stderr,
          "[mavlink] ERROR: Unsupported baudrate: %d\n\tsupported examples:\n\n\t9600\n19200\n38400\n57600\n115200\n230400\n460800\n921600\n\n",
          baud);
      return -EINVAL;
  }

  /* open uart */
  printf("[mavlink] UART is %s, baudrate is %d\n", uart_name, baud);
  uart = open(uart_name, O_RDWR | O_NOCTTY);

  /* Try to set baud rate */
  struct termios uart_config;
  int termios_state;
  *is_usb = false;

  if (strcmp(uart_name, "/dev/ttyACM0") != OK)
  {
    /* Back up the original uart configuration to restore it after exit */
    if ((termios_state = tcgetattr(uart, uart_config_original)) < 0)
    {
      fprintf(stderr, "[mavlink] ERROR getting baudrate / termios config for %s: %d\n", uart_name, termios_state);
      close(uart);
      return -1;
    }

    /* Fill the struct for the new configuration */
    tcgetattr(uart, &uart_config);

    /* Clear ONLCR flag (which appends a CR for every LF) */
    uart_config.c_oflag &= ~ONLCR;

    /* Set baud rate */
    if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0)
    {
      fprintf(stderr, "[mavlink] ERROR setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)\n",
              uart_name, termios_state);
      close(uart);
      return -1;
    }

    if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0)
    {
      fprintf(stderr, "[mavlink] ERROR setting baudrate / termios config for %s (tcsetattr)\n", uart_name);
      close(uart);
      return -1;
    }

  }
  else
  {
    *is_usb = true;
  }

  return uart;
}

void mavlink_send_uart_bytes(mavlink_channel_t channel, uint8_t *ch, int length)
{
  write(uart, ch, (size_t)(sizeof(uint8_t) * length));
}

/*
 * Internal function to give access to the channel status for each channel
 */
mavlink_status_t* mavlink_get_channel_status(uint8_t channel)
{
  static mavlink_status_t m_mavlink_status[MAVLINK_COMM_NUM_BUFFERS];
  return &m_mavlink_status[channel];
}

/*
 * Internal function to give access to the channel buffer for each channel
 */
mavlink_message_t* mavlink_get_channel_buffer(uint8_t channel)
{
  static mavlink_message_t m_mavlink_buffer[MAVLINK_COMM_NUM_BUFFERS];
  return &m_mavlink_buffer[channel];
}

void mavlink_update_system(void)
{
  static bool initialized = false;
  param_t param_system_id;
  param_t param_component_id;
  param_t param_system_type;

  if (!initialized)
  {
    param_system_id = param_find("MAV_SYS_ID");
    param_component_id = param_find("MAV_COMP_ID");
    param_system_type = param_find("MAV_TYPE");
  }

  /* update system and component id */
  int32_t system_id;
  param_get(param_system_id, &system_id);
  if (system_id > 0 && system_id < 255)
  {
    mavlink_system.sysid = system_id;
  }

  int32_t component_id;
  param_get(param_component_id, &component_id);
  if (component_id > 0 && component_id < 255)
  {
    mavlink_system.compid = component_id;
  }

  int32_t system_type;
  param_get(param_system_type, &system_type);
  if (system_type >= 0 && system_type < MAV_TYPE_ENUM_END)
  {
    mavlink_system.type = system_type;
  }
}

void get_mavlink_mode_and_state(const struct vehicle_control_mode_s *control_mode, const struct actuator_armed_s *armed,
                                uint8_t *mavlink_state, uint8_t *mavlink_mode)
{
  /* reset MAVLink mode bitfield */
  *mavlink_mode = 0;

  /* set mode flags independent of system state */
  if (control_mode->flag_control_manual_enabled)
  {
    *mavlink_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
  }

  if (control_mode->flag_system_hil_enabled)
  {
    *mavlink_mode |= MAV_MODE_FLAG_HIL_ENABLED;
  }

  /* set arming state */
  if (armed->armed)
  {
    *mavlink_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
  }
  else
  {
    *mavlink_mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
  }

  if (control_mode->flag_control_velocity_enabled)
  {
    *mavlink_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
  }
  else
  {
    *mavlink_mode &= ~MAV_MODE_FLAG_GUIDED_ENABLED;
  }
}

/**
 * MAVLink Protocol main function.
 */
int rpg_mavlink_fb_thread_main(int argc, char *argv[])
{
  int ch;
  char *device_name = "/dev/ttyS1";
  baudrate = 57600;

  /* XXX this is never written? */
  struct vehicle_status_s v_status;
  struct vehicle_control_mode_s control_mode;
  struct actuator_armed_s armed;

  /* work around some stupidity in task_create's argv handling */
  argc -= 2;
  argv += 2;

  while ((ch = getopt(argc, argv, "b:d:eo")) != EOF)
  {
    switch (ch)
    {
      case 'b':
        baudrate = strtoul(optarg, NULL, 10);
        if (baudrate == 0)
          errx(1, "invalid baud rate '%s'", optarg);
        break;

      case 'd':
        device_name = optarg;
        break;

      case 'e':
        mavlink_link_termination_allowed = true;
        break;

      case 'o':
        mavlink_link_mode = MAVLINK_INTERFACE_MODE_ONBOARD;
        break;

      default:
        usage();
    }
  }

  struct termios uart_config_original;
  bool usb_uart;

  /* print welcome text */
  warnx("MAVLink v1.0 serial interface starting...");

  /* inform about mode */
  warnx((mavlink_link_mode == MAVLINK_INTERFACE_MODE_ONBOARD) ? "ONBOARD MODE" : "DOWNLINK MODE");

  /* Flush stdout in case MAVLink is about to take it over */
  fflush(stdout);

  /* default values for arguments */
  uart = mavlink_open_uart(baudrate, device_name, &uart_config_original, &usb_uart);
  if (uart < 0)
    err(1, "could not open %s", device_name);

  /* Initialize system properties */
  mavlink_update_system();

  /* start the MAVLink receiver */
  receive_thread = receive_start(uart);

  thread_running = true;

  /////////////////////////////////////
  // RPG
  /////////////////////////////////////

  struct imu_msg_s imu_msg;
  int imu_sub = orb_subscribe(ORB_ID(imu_msg));
  memset(&imu_msg, 0, sizeof(imu_msg));
  orb_set_interval(imu_sub, 10); // 100 Hz
  //Should have been read out with 200 Hz...

  struct mag_msg_s mag_msg;
  int mag_sub = orb_subscribe(ORB_ID(mag_msg));
  memset(&mag_msg, 0, sizeof(mag_msg));
  orb_set_interval(mag_sub, 10); // 100 Hz

  struct battery_status_s battery_status;
  int battery_sub = orb_subscribe(ORB_ID(battery_status));
  memset(&battery_status, 0, sizeof(battery_status));
  orb_set_interval(battery_sub, 1000); // 1 Hz

  struct baro_report baro_msg;
  int baro_sub = orb_subscribe(ORB_ID(sensor_baro));
  memset(&baro_msg, 0, sizeof(baro_msg));
  orb_set_interval(baro_sub, 10); // 100 Hz

  struct sonar_msg_s sonar_msg;
  int sonar_sub = orb_subscribe(ORB_ID(sonar_msg));
  memset(&sonar_msg, 0, sizeof(sonar_msg));
  orb_set_interval(sonar_sub, 10); // 100 Hz

  struct thrust_inputs_s thrust_inputs_uorb_msg;
  memset(&thrust_inputs_uorb_msg, 0, sizeof(thrust_inputs_uorb_msg));
  int thrust_inputs_sub = orb_subscribe(ORB_ID(thrust_inputs));
  orb_set_interval(thrust_inputs_sub, 10); // 100 Hz

  struct emergency_ekf_msg_s emergency_ekf_msg;
  memset(&emergency_ekf_msg, 0, sizeof(emergency_ekf_msg));
  int emergency_ekf_sub = orb_subscribe(ORB_ID(emergency_ekf_msg));
  orb_set_interval(emergency_ekf_sub, 10); //100 Hz

  struct pollfd fds[7] = { {.fd = imu_sub, .events = POLLIN}, {.fd = mag_sub, .events = POLLIN}, {.fd = baro_sub,
                                                                                                  .events = POLLIN},
                          {.fd = sonar_sub, .events = POLLIN}, {.fd = battery_sub, .events = POLLIN}, {
                              .fd = thrust_inputs_sub, .events = POLLIN}, {.fd = emergency_ekf_sub, .events = POLLIN}};

  /////////////////////////////////////
  // RPG END
  /////////////////////////////////////

  while (!thread_should_exit)
  {

    /////////////////////////////////////
    // RPG
    /////////////////////////////////////
    int poll_ret = poll(fds, 7, 10);

    if (poll_ret > 0 && (fds[0].revents & POLLIN))
    {
      // IMU
      orb_copy(ORB_ID(imu_msg), imu_sub, &imu_msg);

      // Send through mavlink

      mavlink_msg_rpg_imu_send(chan, imu_msg.timestamp, imu_msg.gyro_x, imu_msg.gyro_y, imu_msg.gyro_z, imu_msg.acc_x,
                               imu_msg.acc_y, imu_msg.acc_z);
    }

    if (poll_ret > 0 && (fds[1].revents & POLLIN))
    {
      // magnetometer
      orb_copy(ORB_ID(mag_msg), mag_sub, &mag_msg);

      // Send through mavlink
      mavlink_msg_magnetic_field_send(chan, mag_msg.timestamp, mag_msg.x, mag_msg.y, mag_msg.z);
    }

    if (poll_ret > 0 && (fds[2].revents & POLLIN))
    {
      // barometer
      orb_copy(ORB_ID(sensor_baro), baro_sub, &baro_msg);

      //Send through mavlink
      mavlink_msg_scaled_pressure_send(chan, baro_msg.timestamp, baro_msg.pressure, 0.0f, baro_msg.temperature * 100.0f);
    }

    if (poll_ret > 0 && (fds[3].revents & POLLIN))
    {
      // sonar
      orb_copy(ORB_ID(sonar_msg), sonar_sub, &sonar_msg);
      // Send through mavlink
      mavlink_msg_named_value_float_send(chan, sonar_msg.timestamp, "sonar", sonar_msg.sonar_down);

      // XXX In my opinion should also be changed...
    }

    if (poll_ret > 0 && (fds[4].revents & POLLIN))
    {
      // battery status
      orb_copy(ORB_ID(battery_status), battery_sub, &battery_status);

      // Send through mavlink
      mavlink_msg_named_value_float_send(chan, battery_status.timestamp, "v_bat", battery_status.voltage_filtered_v);
    }

    if (poll_ret > 0 && (fds[5].revents & POLLIN))
    {
      // commanded rotor thrusts
      orb_copy(ORB_ID(thrust_inputs), thrust_inputs_sub, &thrust_inputs_uorb_msg);
      mavlink_msg_quad_rotor_thrusts_send(chan, thrust_inputs_uorb_msg.timestamp,
                                          thrust_inputs_uorb_msg.thrust_inputs[0],
                                          thrust_inputs_uorb_msg.thrust_inputs[1],
                                          thrust_inputs_uorb_msg.thrust_inputs[2],
                                          thrust_inputs_uorb_msg.thrust_inputs[3]);
    }

    if (poll_ret > 0 && (fds[6].revents & POLLIN))
    {
      // emergency EKF state
      orb_copy(ORB_ID(emergency_ekf_msg), emergency_ekf_sub, &emergency_ekf_msg);

      printf("received emergency ekf state \n");

      // Send through mavlink
      mavlink_msg_emergency_ekf_send(chan,
                                     emergency_ekf_msg.timestamp,
                                     emergency_ekf_msg.h_W,
                                     emergency_ekf_msg.u_B,
                                     emergency_ekf_msg.v_B,
                                     emergency_ekf_msg.w_B,
                                     emergency_ekf_msg.q_w,
                                     emergency_ekf_msg.q_x,
                                     emergency_ekf_msg.q_y,
                                     emergency_ekf_msg.q_z,
                                     emergency_ekf_msg.p_0,
                                     emergency_ekf_msg.phi,
                                     emergency_ekf_msg.theta,
                                     emergency_ekf_msg.psi,
                                     emergency_ekf_msg.h_0,
                                     emergency_ekf_msg.b_T);
    }

    // If there are parameters queued for sending, send 1
    mavlink_pm_queued_send();

    /////////////////////////////////////
    // RPG END
    /////////////////////////////////////

  }

  /* wait for threads to complete */
  pthread_join(receive_thread, NULL);

  /* Reset the UART flags to original state */
  if (!usb_uart)
    tcsetattr(uart, TCSANOW, &uart_config_original);

  thread_running = false;

  exit(0);
}

static void usage()
{
  fprintf(stderr, "usage: mavlink start [-d <devicename>] [-b <baud rate>]\n"
          "       mavlink stop\n"
          "       mavlink status\n");
  exit(1);
}

// Adapted the function name as we ass the thread name in task_spawn_cmd
int rpg_mavlink_onboard_fb_main(int argc, char *argv[])
{

  if (argc < 2)
  {
    warnx("missing command");
    usage();
  }

  if (!strcmp(argv[1], "start"))
  {

    /* this is not an error */
    if (thread_running)
      errx(0, "rpg_mavlink_onboard_fb already running\n");

    thread_should_exit = false;
    mavlink_task = task_spawn_cmd("rpg_mavlink_onboard_fb",
    SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT, 2048, rpg_mavlink_fb_thread_main, (const char**)argv);
    exit(0);
  }

  if (!strcmp(argv[1], "stop"))
  {
    thread_should_exit = true;
    while (thread_running)
    {
      usleep(200000);
    }
    warnx("rpg_mavlink_onboard_fb terminated.");
    exit(0);
  }

  if (!strcmp(argv[1], "status"))
  {
    if (thread_running)
    {
      errx(0, "rpg_mavlink_onboard_fb running");
    }
    else
    {
      errx(1, "rpg_mavlink_onboard_fb not running");
    }
  }

  warnx("unrecognized command");
  usage();
  /* not getting here */
  return 0;
}


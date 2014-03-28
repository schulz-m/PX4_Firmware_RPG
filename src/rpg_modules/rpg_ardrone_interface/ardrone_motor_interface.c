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
 * @file ardrone_motor_control.c
 * Implementation of AR.Drone 1.0 / 2.0 motor control interface
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include <termios.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/torques_and_thrust.h>
#include <systemlib/param/param.h>

#include "ardrone_motor_interface.h"

PARAM_DEFINE_FLOAT(RPG_ARDINT_MASS, 0.44f);
PARAM_DEFINE_FLOAT(RPG_ARDINT_L, 0.178f);

PARAM_DEFINE_FLOAT(RPG_ARDINT_KAPPA, 0.0195f);

PARAM_DEFINE_FLOAT(RPG_ARDINT_G1, 1.0f);
PARAM_DEFINE_FLOAT(RPG_ARDINT_G2, 1.0f);
PARAM_DEFINE_FLOAT(RPG_ARDINT_G3, 1.0f);
PARAM_DEFINE_FLOAT(RPG_ARDINT_G4, 1.0f);

const int MAX_MOTOR_CMD = 510;
const int MIN_SPINNING_MOTOR_CMD = 10;
const float THRUST_MAPPING_A = 4.4854e-06;
const float THRUST_MAPPING_B = 0.0013;
const float THRUST_MAPPING_C = 0.1088;

static unsigned long motor_gpios = GPIO_EXT_1 | GPIO_EXT_2 | GPIO_MULTI_1 | GPIO_MULTI_2;
static unsigned long motor_gpio[4] = {GPIO_EXT_1, GPIO_EXT_2, GPIO_MULTI_1, GPIO_MULTI_2};

typedef union
{
  uint16_t motor_value;
  uint8_t bytes[2];
} motor_union_t;

#define UART_TRANSFER_TIME_BYTE_US (9+50) /**< 9 us per byte at 115200k plus overhead */

/**
 * @brief Generate the 8-byte motor set packet
 *
 * @return the number of bytes (8)
 */
void arGetMotorPacket(uint8_t *motor_buf, uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4)
{
  motor_buf[0] = 0x20;
  motor_buf[1] = 0x00;
  motor_buf[2] = 0x00;
  motor_buf[3] = 0x00;
  motor_buf[4] = 0x00;
  /*
   * {0x20, 0x00, 0x00, 0x00, 0x00};
   * 0x20 is start sign / motor command
   */
  motor_union_t curr_motor;
  uint16_t nineBitMask = 0x1FF;

  /* Set motor 1 */
  curr_motor.motor_value = (motor1 & nineBitMask) << 4;
  motor_buf[0] |= curr_motor.bytes[1];
  motor_buf[1] |= curr_motor.bytes[0];

  /* Set motor 2 */
  curr_motor.motor_value = (motor2 & nineBitMask) << 3;
  motor_buf[1] |= curr_motor.bytes[1];
  motor_buf[2] |= curr_motor.bytes[0];

  /* Set motor 3 */
  curr_motor.motor_value = (motor3 & nineBitMask) << 2;
  motor_buf[2] |= curr_motor.bytes[1];
  motor_buf[3] |= curr_motor.bytes[0];

  /* Set motor 4 */
  curr_motor.motor_value = (motor4 & nineBitMask) << 1;
  motor_buf[3] |= curr_motor.bytes[1];
  motor_buf[4] |= curr_motor.bytes[0];
}

void arEnableBroadcast(int fd)
{
  ar_select_motor(fd, 0);
}

int arMultiplexingInit()
{
  int fd;

  fd = open(PX4FMU_DEVICE_PATH, 0);

  if (fd < 0)
  {
    warn("GPIO: open fail");
    return fd;
  }

  /* deactivate all outputs */
  if (ioctl(fd, GPIO_SET, motor_gpios))
  {
    warn("GPIO: clearing pins fail");
    close(fd);
    return -1;
  }

  /* configure all motor select GPIOs as outputs */
  if (ioctl(fd, GPIO_SET_OUTPUT, motor_gpios) != 0)
  {
    warn("GPIO: output set fail");
    close(fd);
    return -1;
  }

  return fd;
}

int arMultiplexingDeinit(int fd)
{
  if (fd < 0)
  {
    printf("GPIO: no valid descriptor\n");
    return fd;
  }

  int ret = 0;

  /* deselect motor 1-4 */
  ret += ioctl(fd, GPIO_SET, motor_gpios);

  if (ret != 0)
  {
    printf("GPIO: clear failed %d times\n", ret);
  }

  if (ioctl(fd, GPIO_SET_INPUT, motor_gpios) != 0)
  {
    printf("GPIO: input set fail\n");
    return -1;
  }

  close(fd);

  return ret;
}

int arSelectMotor(int fd, uint8_t motor)
{
  int ret = 0;
  /*
   *  Four GPIOS:
   *		GPIO_EXT1
   *		GPIO_EXT2
   *		GPIO_UART2_CTS
   *		GPIO_UART2_RTS
   */

  /* select motor 0 to enable broadcast */
  if (motor == 0)
  {
    /* select motor 1-4 */
    ret += ioctl(fd, GPIO_CLEAR, motor_gpios);

  }
  else
  {
    /* select reqested motor */
    ret += ioctl(fd, GPIO_CLEAR, motor_gpio[motor - 1]);
  }

  return ret;
}

int arDeselectMotor(int fd, uint8_t motor)
{
  int ret = 0;
  /*
   *  Four GPIOS:
   *		GPIO_EXT1
   *		GPIO_EXT2
   *		GPIO_UART2_CTS
   *		GPIO_UART2_RTS
   */

  if (motor == 0)
  {
    /* deselect motor 1-4 */
    ret += ioctl(fd, GPIO_SET, motor_gpios);

  }
  else
  {
    /* deselect reqested motor */
    ret = ioctl(fd, GPIO_SET, motor_gpio[motor - 1]);
  }

  return ret;
}

int arInitMotors(int ardrone_uart, int gpios)
{
  /* Write ARDrone commands on UART2 */
  uint8_t initbuf[] = {0xE0, 0x91, 0xA1, 0x00, 0x40};
  uint8_t multicastbuf[] = {0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0};

  /* deselect all motors */
  arDeselectMotor(gpios, 0);

  /* initialize all motors
   * - select one motor at a time
   * - configure motor
   */
  int i;
  int errcounter = 0;

  /* initial setup run */
  for (i = 1; i < 5; ++i)
  {
    /* Initialize motors 1-4 */
    errcounter += arSelectMotor(gpios, i);
    usleep(200);

    /*
     * write 0xE0 - request status
     * receive one status byte
     */
    write(ardrone_uart, &(initbuf[0]), 1);
    fsync(ardrone_uart);
    usleep(UART_TRANSFER_TIME_BYTE_US * 1);

    /*
     * write 0x91 - request checksum
     * receive 120 status bytes
     */
    write(ardrone_uart, &(initbuf[1]), 1);
    fsync(ardrone_uart);
    usleep(UART_TRANSFER_TIME_BYTE_US * 120);

    /*
     * write 0xA1 - set status OK
     * receive one status byte - should be A0
     * to confirm status is OK
     */
    write(ardrone_uart, &(initbuf[2]), 1);
    fsync(ardrone_uart);
    usleep(UART_TRANSFER_TIME_BYTE_US * 1);

    /*
     * set as motor i, where i = 1..4
     * receive nothing
     */
    initbuf[3] = (uint8_t)i;
    write(ardrone_uart, &(initbuf[3]), 1);
    fsync(ardrone_uart);

    /*
     * write 0x40 - check version
     * receive 11 bytes encoding the version
     */
    write(ardrone_uart, &(initbuf[4]), 1);
    fsync(ardrone_uart);
    usleep(UART_TRANSFER_TIME_BYTE_US * 11);

    arDeselectMotor(gpios, i);
    /* sleep 200 ms */
    usleep(200000);
  }

  /* start the multicast part */
  errcounter += arSelectMotor(gpios, 0);
  usleep(200);

  /*
   * first round
   * write six times A0 - enable broadcast
   * receive nothing
   */
  write(ardrone_uart, multicastbuf, sizeof(multicastbuf));
  fsync(ardrone_uart);
  usleep(UART_TRANSFER_TIME_BYTE_US * sizeof(multicastbuf));

  /*
   * second round
   * write six times A0 - enable broadcast
   * receive nothing
   */
  write(ardrone_uart, multicastbuf, sizeof(multicastbuf));
  fsync(ardrone_uart);
  usleep(UART_TRANSFER_TIME_BYTE_US * sizeof(multicastbuf));

  /* set motors to zero speed (fsync is part of the write command */
  ardroneWriteMotorCommands(ardrone_uart, 0, 0, 0, 0);

  if (errcounter != 0)
  {
    fprintf(stderr, "[ardrone_interface] init sequence incomplete, failed %d times", -errcounter);
    fflush(stdout);
  }
  return errcounter;
}

/**
 * Sets the leds on the motor controllers, 1 turns led on, 0 off.
 */
void arSetLeds(int ardrone_uart, uint8_t led1_red, uint8_t led1_green, uint8_t led2_red, uint8_t led2_green,
                 uint8_t led3_red, uint8_t led3_green, uint8_t led4_red, uint8_t led4_green)
{
  /*
   * 2 bytes are sent. The first 3 bits describe the command: 011 means led control
   * the following 4 bits are the red leds for motor 4, 3, 2, 1
   * then 4 bits with unknown function, then 4 bits for green leds for motor 4, 3, 2, 1
   * the last bit is unknown.
   *
   * The packet is therefore:
   * 011 rrrr 0000 gggg 0
   */
  uint8_t leds[2];
  leds[0] = 0x60 | ((led4_red & 0x01) << 4) | ((led3_red & 0x01) << 3) | ((led2_red & 0x01) << 2)
      | ((led1_red & 0x01) << 1);
  leds[1] = ((led4_green & 0x01) << 4) | ((led3_green & 0x01) << 3) | ((led2_green & 0x01) << 2)
      | ((led1_green & 0x01) << 1);
  write(ardrone_uart, leds, 2);
}

int openArdroneMotorPorts(char *device, int *ardrone_write, struct termios* uart_config_original, int* gpios)
{
  *ardrone_write = ardroneOpenUart(device, uart_config_original);

  // initialize multiplexing, deactivate all outputs - must happen after UART open to claim GPIOs on PX4FMU
  *gpios = arMultiplexingInit();

  if (*ardrone_write < 0)
  {
    fprintf(stderr, "[rpg_rate_controller] Failed opening AR.Drone UART, exiting.\n");
    return -1;
    exit(ERROR);
  }

  // initialize motors
  if (OK != arInitMotors(*ardrone_write, *gpios))
  {
    close(*ardrone_write);
    fprintf(stderr, "[rpg_rate_controller] Failed initializing AR.Drone motors, exiting.\n");
    return -1;
    exit(ERROR);
  }

  close(*ardrone_write);

  // enable UART, writes potentially an empty buffer, but multiplexing is disabled
  *ardrone_write = ardroneOpenUart(device, uart_config_original);

  // initialize multiplexing, deactivate all outputs - must happen after UART open to claim GPIOs on PX4FMU
  *gpios = arMultiplexingInit();

  if (*ardrone_write < 0)
  {
    fprintf(stderr, "[rpg_rate_controller] Failed opening AR.Drone UART, exiting.\n");
    return -1;
  }

  // initialize motors
  if (OK != arInitMotors(*ardrone_write, *gpios))
  {
    close(*ardrone_write);
    fprintf(stderr, "[rpg_rate_controller] Failed initializing AR.Drone motors, exiting.\n");
    return -1;
  }

  return 0;
}

int closeArdroneMotorPorts(int* ardrone_write, struct termios* uart_config_original, int* gpios)
{
  // restore old UART config
  int termios_state;

  if ((termios_state = tcsetattr(*ardrone_write, TCSANOW, uart_config_original)) < 0)
  {
    fprintf(stderr, "[rpg_rate_controller] ERROR setting baudrate / termios config for (tcsetattr)\n");
  }

  /* close uarts */
  close(*ardrone_write);
  arMultiplexingDeinit(*gpios);

  return 0;
}

int ardroneOpenUart(char *uart_name, struct termios *uart_config_original)
{
  /* baud rate */
  int speed = B115200;
  int uart;

  /* open uart */
  uart = open(uart_name, O_RDWR | O_NOCTTY);

  /* Try to set baud rate */
  struct termios uart_config;
  int termios_state;

  /* Back up the original uart configuration to restore it after exit */
  if ((termios_state = tcgetattr(uart, uart_config_original)) < 0)
  {
    fprintf(stderr, "[ardrone_interface] ERROR getting baudrate / termios config for %s: %d\n", uart_name,
            termios_state);
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
    fprintf(stderr,
            "[ardrone_interface] ERROR setting baudrate / termios config for %s: %d (cfsetispeed, cfsetospeed)\n",
            uart_name, termios_state);
    close(uart);
    return -1;
  }

  if ((termios_state = tcsetattr(uart, TCSANOW, &uart_config)) < 0)
  {
    fprintf(stderr, "[ardrone_interface] ERROR setting baudrate / termios config for %s (tcsetattr)\n", uart_name);
    close(uart);
    return -1;
  }

  return uart;
}

int ardroneWriteMotorCommands(int ardrone_fd, uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4)
{
  const unsigned int min_motor_interval = 4900;
  static uint64_t last_motor_time = 0;

  static struct actuator_outputs_s outputs;
  outputs.timestamp = hrt_absolute_time();
  outputs.output[0] = motor1;
  outputs.output[1] = motor2;
  outputs.output[2] = motor3;
  outputs.output[3] = motor4;
  static orb_advert_t pub = 0;
  if (pub == 0)
  {
    pub = orb_advertise(ORB_ID_VEHICLE_CONTROLS, &outputs);
  }

  if (hrt_absolute_time() - last_motor_time > min_motor_interval)
  {
    uint8_t buf[5] = {0};
    arGetMotorPacket(buf, motor1, motor2, motor3, motor4);
    int ret;
    ret = write(ardrone_fd, buf, sizeof(buf));
    fsync(ardrone_fd);

    /* publish just written values */
    orb_publish(ORB_ID_VEHICLE_CONTROLS, pub, &outputs);

    if (ret == sizeof(buf))
    {
      return OK;
    }
    else
    {
      return ret;
    }
  }
  else
  {
    return -ERROR;
  }
}

void computeMotorCommands(uint16_t motor_commands[], struct torques_and_thrust_s desired_torques_and_thrust,
bool use_x_configuration,
                            const struct rpg_ardrone_interface_params params)
{
  // This function distinguishes between the x-configuration or the +-configuration defined as follows:

  // x-configuration assumes that the IMU is aligned to the "x-configuration" where rotor 1 is spinning clockwise as seen from above
  // 1    2    x
  //  \  /     ^
  //   \/      |__> y
  //   /\
  //  /  \
  // 4    3

  // +-configuration assumes that the IMU is aligned to the "+-configuration" where rotor 1 is spinning clockwise as seen from above
  //     1        x
  //     |        ^
  // 4-------2    |__> y
  //     |
  //     3

  float rotor_thrusts[4] = {0.0f};

  // Compute single rotor thrusts for given torques and normalized thrust
  computeSingleRotorThrusts(rotor_thrusts, desired_torques_and_thrust.roll_torque,
                               desired_torques_and_thrust.pitch_torque, desired_torques_and_thrust.yaw_torque,
                               desired_torques_and_thrust.thrust, use_x_configuration, params);

  // Lower collective thrust if one or more of the rotors is saturated
  float max_nom_rotor_thrust = convertMotorCommandToThrust(MAX_MOTOR_CMD);
  float collective_thrust_above_saturation = 0.0f;

  if (rotor_thrusts[0] > params.gamma_1 * max_nom_rotor_thrust)
  {
    collective_thrust_above_saturation = fmaxf(4.0f * params.gamma_1 * (rotor_thrusts[0] - max_nom_rotor_thrust),
                                               collective_thrust_above_saturation);
  }
  if (rotor_thrusts[1] > params.gamma_2 * max_nom_rotor_thrust)
  {
    collective_thrust_above_saturation = fmaxf(4.0f * params.gamma_1 * (rotor_thrusts[0] - max_nom_rotor_thrust),
                                               collective_thrust_above_saturation);
  }
  if (rotor_thrusts[2] > params.gamma_3 * max_nom_rotor_thrust)
  {
    collective_thrust_above_saturation = fmaxf(4.0f * params.gamma_1 * (rotor_thrusts[0] - max_nom_rotor_thrust),
                                               collective_thrust_above_saturation);
  }
  if (rotor_thrusts[3] > params.gamma_4 * max_nom_rotor_thrust)
  {
    collective_thrust_above_saturation = fmaxf(4.0f * params.gamma_1 * (rotor_thrusts[0] - max_nom_rotor_thrust),
                                               collective_thrust_above_saturation);
  }

  if (collective_thrust_above_saturation > 0.0f)
  {
    // Recompute rotor thrusts with saturated collective thrust (rates_thrust_sp[3] - collective_thrust_above_saturation/params.mass)
    computeSingleRotorThrusts(rotor_thrusts, desired_torques_and_thrust.roll_torque,
                                 desired_torques_and_thrust.pitch_torque, desired_torques_and_thrust.yaw_torque,
                                 desired_torques_and_thrust.thrust - collective_thrust_above_saturation / params.mass,
                                 use_x_configuration, params);
  }

  // Convert forces into motor commands
  uint16_t motor_1_cmd = convertThrustToMotorCommand(rotor_thrusts[0]);
  uint16_t motor_2_cmd = convertThrustToMotorCommand(rotor_thrusts[1]);
  uint16_t motor_3_cmd = convertThrustToMotorCommand(rotor_thrusts[2]);
  uint16_t motor_4_cmd = convertThrustToMotorCommand(rotor_thrusts[3]);

  // Saturate motor commands to ensure valid range
  if (desired_torques_and_thrust.thrust < 2.5f) // this is an acceleration in [m/s^2], so 9.81 would be hover.
  {
    // This is a small collective thrust so we can allow 0 motor commands. This is important since with 0 thrust commands we want to have the motors not spinning!!!
    motor_commands[0] = saturateMotorCommand(motor_1_cmd, 0, MAX_MOTOR_CMD);
    motor_commands[1] = saturateMotorCommand(motor_2_cmd, 0, MAX_MOTOR_CMD);
    motor_commands[2] = saturateMotorCommand(motor_3_cmd, 0, MAX_MOTOR_CMD);
    motor_commands[3] = saturateMotorCommand(motor_4_cmd, 0, MAX_MOTOR_CMD);
  }
  else
  {
    // This is a decent collective thrust so we don't allow the rotors to stop entirely
    motor_commands[0] = saturateMotorCommand(motor_1_cmd, MIN_SPINNING_MOTOR_CMD, MAX_MOTOR_CMD);
    motor_commands[1] = saturateMotorCommand(motor_2_cmd, MIN_SPINNING_MOTOR_CMD, MAX_MOTOR_CMD);
    motor_commands[2] = saturateMotorCommand(motor_3_cmd, MIN_SPINNING_MOTOR_CMD, MAX_MOTOR_CMD);
    motor_commands[3] = saturateMotorCommand(motor_4_cmd, MIN_SPINNING_MOTOR_CMD, MAX_MOTOR_CMD);
  }
}

uint16_t convertThrustToMotorCommand(float thrust)
{
  // compute thrust for one rotor with second order polynomial according to
  // force = a*mot_cmd^2 + b*mot_cmd + c
  // take the inverse of this function
  uint16_t motor_command = round(
      (-THRUST_MAPPING_B
          + sqrt(THRUST_MAPPING_B * THRUST_MAPPING_B - 4.0 * THRUST_MAPPING_A * (THRUST_MAPPING_C - thrust)))
          / (2 * THRUST_MAPPING_A));

  return motor_command;
}

float convertMotorCommandToThrust(uint16_t motor_command)
{
  // compute thrust for one rotor with second order polynomial according to
  // force = a*mot_cmd^2 + b*mot_cmd + c
  float thrust = THRUST_MAPPING_A * motor_command * motor_command + THRUST_MAPPING_B * motor_command + THRUST_MAPPING_C;

  return thrust;
}

uint16_t saturateMotorCommand(uint16_t value, uint16_t min, uint16_t max)
{
  if (value < min)
    value = min;
  if (value > max)
    value = max;

  return value;
}

void computeSingleRotorThrusts(float* rotor_thrusts, float roll_torque, float pitch_torque, float yaw_torque,
                                  float normalized_thrust, bool use_x_configuration,
                                  const struct rpg_ardrone_interface_params params)
{
  float K = params.rotor_drag_coeff;
  float L = params.arm_length;

  if (use_x_configuration)
  {
    // Compute the desired thrust for each rotor for x-configuration
    rotor_thrusts[0] = 1.0f / params.gamma_1
        * ((K * L * params.mass * normalized_thrust - L * yaw_torque + sqrt(2) * K * roll_torque
            + sqrt(2) * K * pitch_torque) / (4 * K * L));
    rotor_thrusts[1] = 1.0f / params.gamma_2
        * ((L * yaw_torque + K * L * params.mass * normalized_thrust - sqrt(2) * K * roll_torque
            + sqrt(2) * K * pitch_torque) / (4 * K * L));
    rotor_thrusts[2] = 1.0f / params.gamma_3
        * (-(sqrt(2)
            * (2 * K * roll_torque + 2 * K * pitch_torque + sqrt(2) * L * yaw_torque
                - sqrt(2) * K * L * params.mass * normalized_thrust)) / (8 * K * L));
    rotor_thrusts[3] = 1.0f / params.gamma_4
        * ((sqrt(2)
            * (2 * K * roll_torque - 2 * K * pitch_torque + sqrt(2) * L * yaw_torque
                + sqrt(2) * K * L * params.mass * normalized_thrust)) / (8 * K * L));
  }
  else
  {
    // Compute the desired thrust for each rotor for +-configuration
    rotor_thrusts[0] = 1.0f / params.gamma_1
        * ((2 * K * pitch_torque - L * yaw_torque + K * L * params.mass * normalized_thrust) / (4 * K * L));
    rotor_thrusts[1] = 1.0f / params.gamma_2
        * ((L * yaw_torque - 2 * K * roll_torque + K * L * params.mass * normalized_thrust) / (4 * K * L));
    rotor_thrusts[2] = 1.0f / params.gamma_3
        * (-(2 * K * pitch_torque + L * yaw_torque - K * L * params.mass * normalized_thrust) / (4 * K * L));
    rotor_thrusts[3] = 1.0f / params.gamma_4
        * ((2 * K * roll_torque + L * yaw_torque + K * L * params.mass * normalized_thrust) / (4 * K * L));
  }
}

int parametersInit(struct rpg_ardrone_interface_params_handles *h)
{
  h->mass = param_find("RPG_ARDINT_MASS");
  h->arm_length = param_find("RPG_ARDINT_L");

  h->rotor_drag_coeff = param_find("RPG_ARDINT_KAPPA");

  h->gamma_1 = param_find("RPG_ARDINT_G1");
  h->gamma_2 = param_find("RPG_ARDINT_G2");
  h->gamma_3 = param_find("RPG_ARDINT_G3");
  h->gamma_4 = param_find("RPG_ARDINT_G4");

  return 0;
}

int parametersUpdate(const struct rpg_ardrone_interface_params_handles *h, struct rpg_ardrone_interface_params *p)
{
  param_get(h->mass, &(p->mass));
  param_get(h->arm_length, &(p->arm_length));

  param_get(h->rotor_drag_coeff, &(p->rotor_drag_coeff));

  param_get(h->gamma_1, &(p->gamma_1));
  param_get(h->gamma_2, &(p->gamma_2));
  param_get(h->gamma_3, &(p->gamma_3));
  param_get(h->gamma_4, &(p->gamma_4));

  return 0;
}

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
 * @file ardrone_motor_control.h
 * Definition of AR.Drone 1.0 / 2.0 motor control interface
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include <termios.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/rpg/torques_and_thrust.h>
#include <systemlib/param/param.h>

struct rpg_ardrone_interface_params
{

  float mass;
  float arm_length;

  float rotor_drag_coeff;

  float gamma_1;
  float gamma_2;
  float gamma_3;
  float gamma_4;
};

struct rpg_ardrone_interface_params_handles
{

  param_t mass;
  param_t arm_length;

  param_t rotor_drag_coeff;

  param_t gamma_1;
  param_t gamma_2;
  param_t gamma_3;
  param_t gamma_4;
};

/**
 * Generate the 5-byte motor set packet.
 *
 * @return the number of bytes (5)
 */
void arGetMotorPacket(uint8_t *motor_buf, uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);

/**
 * Select a motor in the multiplexing.
 *
 * @param fd GPIO file descriptor
 * @param motor Motor number, from 1 to 4, 0 selects all
 */
int arSelectMotor(int fd, uint8_t motor);

/**
 * Deselect a motor in the multiplexing.
 *
 * @param fd GPIO file descriptor
 * @param motor Motor number, from 1 to 4, 0 deselects all
 */
int arDeselectMotor(int fd, uint8_t motor);

void arEnableBroadcast(int fd);

int arMultiplexingInit(void);
int arMultiplexingDeinit(int fd);

/**
 * Write four motor commands to an already initialized port.
 *
 * Writing 0 stops a motor, values from 1-512 encode the full thrust range.
 * on some motor controller firmware revisions a minimum value of 10 is
 * required to spin the motors.
 */
int ardroneWriteMotorCommands(int ardrone_fd, uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4);

/**
 * Initialize the motors.
 */
int arInitMotors(int ardrone_uart, int gpio);

/**
 * Set LED pattern.
 */
void arSetLeds(int ardrone_uart, uint8_t led1_red, uint8_t led1_green, uint8_t led2_red, uint8_t led2_green,
                 uint8_t led3_red, uint8_t led3_green, uint8_t led4_red, uint8_t led4_green);

int openArdroneMotorPorts(char *device, int* ardrone_write, struct termios* uart_config_original, int* gpios);

int closeArdroneMotorPorts(int* ardrone_write, struct termios* uart_config_original, int* gpios);

int ardroneOpenUart(char *uart_name, struct termios *uart_config_original);

uint16_t convertThrustToMotorCommand(float thrust);

float convertMotorCommandToThrust(uint16_t motor_command);

uint16_t saturateMotorCommand(uint16_t value, uint16_t min, uint16_t max);

void computeSingleRotorThrusts(float* rotor_thrusts, float roll_torque, float pitch_torque, float yaw_torque,
                                  float normalized_thrust, bool use_x_configuration,
                                  const struct rpg_ardrone_interface_params params);

void computeMotorCommands(uint16_t motor_commands[], struct torques_and_thrust_s desired_torques_and_thrust,
bool use_x_configuration,
                            const struct rpg_ardrone_interface_params params);

int parametersInit(struct rpg_ardrone_interface_params_handles *h);

int parametersUpdate(const struct rpg_ardrone_interface_params_handles *h, struct rpg_ardrone_interface_params *p);

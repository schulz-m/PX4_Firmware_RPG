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
#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls_effective.h>

#include "ardrone_motor_interface.h"

static unsigned long motor_gpios = GPIO_EXT_1 | GPIO_EXT_2 | GPIO_MULTI_1 | GPIO_MULTI_2;
static unsigned long motor_gpio[4] = { GPIO_EXT_1, GPIO_EXT_2, GPIO_MULTI_1, GPIO_MULTI_2 };

typedef union {
	uint16_t motor_value;
	uint8_t bytes[2];
} motor_union_t;

#define UART_TRANSFER_TIME_BYTE_US (9+50) /**< 9 us per byte at 115200k plus overhead */

/**
 * @brief Generate the 8-byte motor set packet
 *
 * @return the number of bytes (8)
 */
void ar_get_motor_packet(uint8_t *motor_buf, uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4)
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

void ar_enable_broadcast(int fd)
{
	ar_select_motor(fd, 0);
}

int ar_multiplexing_init()
{
	int		fd;
	
	fd = open(PX4FMU_DEVICE_PATH, 0);

	if (fd < 0) {
		warn("GPIO: open fail");
		return fd;
	}

	/* deactivate all outputs */
	if (ioctl(fd, GPIO_SET, motor_gpios)) {
		warn("GPIO: clearing pins fail");
		close(fd);
		return -1;
	}

	/* configure all motor select GPIOs as outputs */
	if (ioctl(fd, GPIO_SET_OUTPUT, motor_gpios) != 0) {
		warn("GPIO: output set fail");
		close(fd);
		return -1;
	}

	return fd;
}

int ar_multiplexing_deinit(int fd)
{
	if (fd < 0) {
		printf("GPIO: no valid descriptor\n");
		return fd;
	}

	int ret = 0;

	/* deselect motor 1-4 */
	ret += ioctl(fd, GPIO_SET, motor_gpios);

	if (ret != 0) {
		printf("GPIO: clear failed %d times\n", ret);
	}

	if (ioctl(fd, GPIO_SET_INPUT, motor_gpios) != 0) {
		printf("GPIO: input set fail\n");
		return -1;
	}

	close(fd);

	return ret;
}

int ar_select_motor(int fd, uint8_t motor)
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
	if (motor == 0) {
		/* select motor 1-4 */
		ret += ioctl(fd, GPIO_CLEAR, motor_gpios);

	} else {
		/* select reqested motor */	
		ret += ioctl(fd, GPIO_CLEAR, motor_gpio[motor - 1]);
	}

	return ret;
}

int ar_deselect_motor(int fd, uint8_t motor)
{
	int ret = 0;
	/*
	 *  Four GPIOS:
	 *		GPIO_EXT1
	 *		GPIO_EXT2
	 *		GPIO_UART2_CTS
	 *		GPIO_UART2_RTS
	 */

	if (motor == 0) {
		/* deselect motor 1-4 */
		ret += ioctl(fd, GPIO_SET, motor_gpios);

	} else {
		/* deselect reqested motor */	
		ret = ioctl(fd, GPIO_SET, motor_gpio[motor - 1]);
	}

	return ret;
}

int ar_init_motors(int ardrone_uart, int gpios)
{
	/* Write ARDrone commands on UART2 */
	uint8_t initbuf[] = {0xE0, 0x91, 0xA1, 0x00, 0x40};
	uint8_t multicastbuf[] = {0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0};

	/* deselect all motors */
	ar_deselect_motor(gpios, 0);

	/* initialize all motors
	 * - select one motor at a time
	 * - configure motor
	 */
	int i;
	int errcounter = 0;


	/* initial setup run */
	for (i = 1; i < 5; ++i) {
		/* Initialize motors 1-4 */
		errcounter += ar_select_motor(gpios, i);
		usleep(200);

		/*
		 * write 0xE0 - request status
		 * receive one status byte
		 */
		write(ardrone_uart, &(initbuf[0]), 1);
		fsync(ardrone_uart);
		usleep(UART_TRANSFER_TIME_BYTE_US*1);

		/*
		 * write 0x91 - request checksum
		 * receive 120 status bytes
		 */
		write(ardrone_uart, &(initbuf[1]), 1);
		fsync(ardrone_uart);
		usleep(UART_TRANSFER_TIME_BYTE_US*120);

		/*
		 * write 0xA1 - set status OK
		 * receive one status byte - should be A0
		 * to confirm status is OK
		 */
		write(ardrone_uart, &(initbuf[2]), 1);
		fsync(ardrone_uart);
		usleep(UART_TRANSFER_TIME_BYTE_US*1);

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
		usleep(UART_TRANSFER_TIME_BYTE_US*11);

		ar_deselect_motor(gpios, i);
		/* sleep 200 ms */
		usleep(200000);
	}

	/* start the multicast part */
	errcounter += ar_select_motor(gpios, 0);
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
	ardrone_write_motor_commands(ardrone_uart, 0, 0, 0, 0);

	if (errcounter != 0) {
		fprintf(stderr, "[ardrone_interface] init sequence incomplete, failed %d times", -errcounter);
		fflush(stdout);
	}
	return errcounter;
}

/**
 * Sets the leds on the motor controllers, 1 turns led on, 0 off.
 */
void ar_set_leds(int ardrone_uart, uint8_t led1_red, uint8_t led1_green, uint8_t led2_red, uint8_t led2_green, uint8_t led3_red, uint8_t led3_green, uint8_t led4_red, uint8_t led4_green)
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
	leds[0] = 0x60 | ((led4_red & 0x01) << 4) | ((led3_red & 0x01) << 3) | ((led2_red & 0x01) << 2) | ((led1_red & 0x01) << 1);
	leds[1] = ((led4_green & 0x01) << 4) | ((led3_green & 0x01) << 3) | ((led2_green & 0x01) << 2) | ((led1_green & 0x01) << 1);
	write(ardrone_uart, leds, 2);
}

int open_ardrone_motor_ports(char *device, int *ardrone_write, struct termios* uart_config_original, int* gpios)
{
  *ardrone_write = ardrone_open_uart(device, uart_config_original);

  // initialize multiplexing, deactivate all outputs - must happen after UART open to claim GPIOs on PX4FMU
  *gpios = ar_multiplexing_init();

  if (*ardrone_write < 0)
  {
    fprintf(stderr, "[rpg_rate_controller] Failed opening AR.Drone UART, exiting.\n");
    return -1;
    exit(ERROR);
  }

  // initialize motors
  if (OK != ar_init_motors(*ardrone_write, *gpios))
  {
    close(*ardrone_write);
    fprintf(stderr, "[rpg_rate_controller] Failed initializing AR.Drone motors, exiting.\n");
    return -1;
    exit(ERROR);
  }

  close(*ardrone_write);

  // enable UART, writes potentially an empty buffer, but multiplexing is disabled
  *ardrone_write = ardrone_open_uart(device, uart_config_original);

  // initialize multiplexing, deactivate all outputs - must happen after UART open to claim GPIOs on PX4FMU
  *gpios = ar_multiplexing_init();

  if (*ardrone_write < 0)
  {
    fprintf(stderr, "[rpg_rate_controller] Failed opening AR.Drone UART, exiting.\n");
    return -1;
  }

  // initialize motors
  if (OK != ar_init_motors(*ardrone_write, *gpios))
  {
    close(*ardrone_write);
    fprintf(stderr, "[rpg_rate_controller] Failed initializing AR.Drone motors, exiting.\n");
    return -1;
  }

  return 0;
}

int close_ardrone_motor_ports(int* ardrone_write, struct termios* uart_config_original, int* gpios)
{
  // restore old UART config
  int termios_state;

  if ((termios_state = tcsetattr(*ardrone_write, TCSANOW, uart_config_original)) < 0)
  {
    fprintf(stderr, "[rpg_rate_controller] ERROR setting baudrate / termios config for (tcsetattr)\n");
  }

  /* close uarts */
  close(*ardrone_write);
  ar_multiplexing_deinit(*gpios);

  return 0;
}

int ardrone_open_uart(char *uart_name, struct termios *uart_config_original)
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

int ardrone_write_motor_commands(int ardrone_fd, uint16_t motor1, uint16_t motor2, uint16_t motor3, uint16_t motor4) {
	const unsigned int min_motor_interval = 4900;
	static uint64_t last_motor_time = 0;

	static struct actuator_outputs_s outputs;
	outputs.timestamp = hrt_absolute_time();
	outputs.output[0] = motor1;
	outputs.output[1] = motor2;
	outputs.output[2] = motor3;
	outputs.output[3] = motor4;
	static orb_advert_t pub = 0;
	if (pub == 0) {
		pub = orb_advertise(ORB_ID_VEHICLE_CONTROLS, &outputs);
	}

	if (hrt_absolute_time() - last_motor_time > min_motor_interval) {
		uint8_t buf[5] = {0};
		ar_get_motor_packet(buf, motor1, motor2, motor3, motor4);
		int ret;
		ret = write(ardrone_fd, buf, sizeof(buf));
		fsync(ardrone_fd);

		/* publish just written values */
		orb_publish(ORB_ID_VEHICLE_CONTROLS, pub, &outputs);

		if (ret == sizeof(buf)) {
			return OK;
		} else {
			return ret;
		}
	} else {
		return -ERROR;
	}
}

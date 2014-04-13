#include <nuttx/config.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls_effective.h>
#include <uORB/topics/offboard_control_setpoint.h>

#include <systemlib/err.h>

__EXPORT int infra_red_led_driver_main(int argc, char *argv[]);

static unsigned long led_gpio = GPIO_EXT_1;

int init_gpio(int fd)
{
  /* deactivate all outputs */
  if (ioctl(fd, GPIO_SET, led_gpio))
  {
    warn("GPIO: clearing pins fail");
    close(fd);
    return -1;
  }

  /* configure all motor select GPIOs as outputs */
  if (ioctl(fd, GPIO_SET_OUTPUT, led_gpio) != 0)
  {
    warn("GPIO: output set fail");
    close(fd);
    return -1;
  }

  return fd;
}

int deinit_gpio(int fd)
{
  if (fd < 0)
  {
    printf("GPIO: no valid descriptor\n");
    return fd;
  }

  int ret = ioctl(fd, GPIO_SET, led_gpio);

  if (ret != 0)
  {
    printf("GPIO: clear failed %d times\n", ret);
  }

  if (ioctl(fd, GPIO_SET_INPUT, led_gpio) != 0)
  {
    printf("GPIO: input set fail\n");
    return -1;
  }
  return ret;
}

int led_on(int fd)
{
  return ioctl(fd, GPIO_SET, led_gpio);
}

int led_off(int fd)
{
  return ioctl(fd, GPIO_CLEAR, led_gpio);
}

int infra_red_led_driver_main(int argc, char *argv[])
{
  printf("here i am\n");

  int fd = open("/dev/px4fmu", 0);
  init_gpio(fd);

  struct offboard_control_setpoint_s offboard_sp;
  memset(&offboard_sp, 0, sizeof(offboard_sp));

  int setpoint_sub = orb_subscribe(ORB_ID(offboard_control_setpoint));

  int ctr = 0;
  while (true )
  {
    bool updated;
    orb_check(setpoint_sub, &updated);
    if (updated)
    {
      ctr++;
      orb_copy(ORB_ID(offboard_control_setpoint), setpoint_sub, &offboard_sp);
//			printf("p1: %f, p2 %f \n", offboard_sp.p1, offboard_sp.p2  );
      if (offboard_sp.p4 > 10)
      {
        led_on(fd);
        printf("led is on now\n");
      }
      else
      {
        led_off(fd);
        printf("led is off now\n");
      }
    }
  }
  deinit_gpio(fd);
  close(fd);
  return 0;
}

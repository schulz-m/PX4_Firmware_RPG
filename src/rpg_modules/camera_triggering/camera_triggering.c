#include <math.h>
#include <drivers/drv_hrt.h>

#include "camera_triggering.h"

int initGPIO(int fd, unsigned long gpio)
{
  /* deactivate all outputs */
  if (ioctl(fd, GPIO_SET, gpio))
  {
    warn("GPIO: clearing pins fail");
    close(fd);
    return -1;
  }

  /* configure all motor select GPIOs as outputs */
  if (ioctl(fd, GPIO_SET_OUTPUT, gpio) != 0)
  {
    warn("GPIO: output set fail");
    close(fd);
    return -1;
  }

  return fd;
}

int deinitGPIO(int fd, unsigned long gpio)
{
  if (fd < 0)
  {
    printf("GPIO: no valid descriptor\n");
    return fd;
  }

  int ret = ioctl(fd, GPIO_SET, gpio);

  if (ret != 0)
  {
    printf("GPIO: clear failed %d times\n", ret);
  }

  if (ioctl(fd, GPIO_SET_INPUT, gpio) != 0)
  {
    printf("GPIO: input set fail\n");
    return -1;
  }
  return ret;
}

int setGPIOHigh(int fd, unsigned long gpio)
{
  return ioctl(fd, GPIO_SET, gpio);
}

int setGPIOLow(int fd, unsigned long gpio)
{
  return ioctl(fd, GPIO_CLEAR, gpio);
}

#ifndef CAMERA_TRIGGERING_H_
#define CAMERA_TRIGGERING_H_

#include <math.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_gpio.h>

int initGPIO(int fd, unsigned long gpio);
int deinitGPIO(int fd, unsigned long gpio);

int setGPIOHigh(int fd, unsigned long gpio);
int setGPIOLow(int fd, unsigned long gpio);

#endif /* CAMERA_TRIGGERING_H_ */

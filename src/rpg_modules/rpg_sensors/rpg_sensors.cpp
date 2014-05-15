/*
 * rpg_sensors.cpp
 *
 *  Created on: May 12, 2014
 *      Author: matthias
 */

#include <nuttx/config.h>

#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>

#include <poll.h>
#include <systemlib/param/param.h>
#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/rpg/imu_msg.h>
#include <uORB/topics/rpg/mag_msg.h>
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_gyro.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_mag.h>
#include <drivers/drv_baro.h>

extern "C" __EXPORT int rpg_sensors_main(int argc, char *argv[]);

namespace rpg_sensors
{

class RPGSensors
{
public:

  RPGSensors();
  ~RPGSensors();

  int start();

  int imu_poll_interval_;

private:

  int parametersInit();
  int parametersUpdate();
  void gyroInit();
  void accelInit();
  void magInit();
  void baroInit();
  void imuPoll(struct imu_msg_s &imu_msg);
  void magPoll(struct mag_msg_s &mag_msg);
  void taskMainTrampoline(int argc, char *argv[]);
  void taskMain();

  int sensor_task_;
  bool task_should_exit_;

  int params_sub_;
  int gyro_sub_;
  int accel_sub_;
  int mag_sub_;

  // Republish the values who need a transformation to RPG coordinates
  orb_advert_t imu_pub_;
  orb_advert_t mag_pub_;

  struct
  {
    float gyro_offset[3];
    float gyro_scale[3];
    float accel_offset[3];
    float accel_scale[3];
    float mag_offset[3];
    float mag_scale[3];
  } parameters_;

  struct
  {
    param_t gyro_offset[3];
    param_t gyro_scale[3];
    param_t accel_offset[3];
    param_t accel_scale[3];
    param_t mag_offset[3];
    param_t mag_scale[3];
  } parameters_handles_;
};

RPGSensors *rpg_sensors_ptr = nullptr;

RPGSensors::RPGSensors() :
    sensor_task_(-1), task_should_exit_(false), params_sub_(-1), gyro_sub_(-1), accel_sub_(-1), mag_sub_(-1), imu_poll_interval_(
        1), imu_pub_(-1), mag_pub_(-1)
{
  // Get parameter handles
  parametersInit();
  // Set initial parameters
  parametersUpdate();
}

RPGSensors::~RPGSensors()
{
  if (sensor_task_ != -1)
  {
    // task wakes up every 100ms or so at the longest
    task_should_exit_ = true;

    // wait for a second for the task to quit at our request
    unsigned i = 0;
    do
    {
      // wait 20ms
      usleep(20000);

      // if we have given up, kill it
      if (++i > 50)
      {
        task_delete(sensor_task_);
        break;
      }
    } while (sensor_task_ != -1);
  }

  rpg_sensors_ptr = nullptr;
}

int RPGSensors::parametersInit()
{
  // gyro offsets and scale
  parameters_handles_.gyro_offset[0] = param_find("SENS_GYRO_XOFF");
  parameters_handles_.gyro_offset[1] = param_find("SENS_GYRO_YOFF");
  parameters_handles_.gyro_offset[2] = param_find("SENS_GYRO_ZOFF");
  parameters_handles_.gyro_scale[0] = param_find("SENS_GYRO_XSCALE");
  parameters_handles_.gyro_scale[1] = param_find("SENS_GYRO_YSCALE");
  parameters_handles_.gyro_scale[2] = param_find("SENS_GYRO_ZSCALE");

  // accel offsets and scale
  parameters_handles_.accel_offset[0] = param_find("SENS_ACC_XOFF");
  parameters_handles_.accel_offset[1] = param_find("SENS_ACC_YOFF");
  parameters_handles_.accel_offset[2] = param_find("SENS_ACC_ZOFF");
  parameters_handles_.accel_scale[0] = param_find("SENS_ACC_XSCALE");
  parameters_handles_.accel_scale[1] = param_find("SENS_ACC_YSCALE");
  parameters_handles_.accel_scale[2] = param_find("SENS_ACC_ZSCALE");

  // magnetometer offsets and scale
  parameters_handles_.mag_offset[0] = param_find("SENS_MAG_XOFF");
  parameters_handles_.mag_offset[1] = param_find("SENS_MAG_YOFF");
  parameters_handles_.mag_offset[2] = param_find("SENS_MAG_ZOFF");
  parameters_handles_.mag_scale[0] = param_find("SENS_MAG_XSCALE");
  parameters_handles_.mag_scale[1] = param_find("SENS_MAG_YSCALE");
  parameters_handles_.mag_scale[2] = param_find("SENS_MAG_ZSCALE");

  return OK;
}

int RPGSensors::parametersUpdate()
{
  // gyro offsets and scale
  param_get(parameters_handles_.gyro_offset[0], &(parameters_.gyro_offset[0]));
  param_get(parameters_handles_.gyro_offset[1], &(parameters_.gyro_offset[1]));
  param_get(parameters_handles_.gyro_offset[2], &(parameters_.gyro_offset[2]));
  param_get(parameters_handles_.gyro_scale[0], &(parameters_.gyro_scale[0]));
  param_get(parameters_handles_.gyro_scale[1], &(parameters_.gyro_scale[1]));
  param_get(parameters_handles_.gyro_scale[2], &(parameters_.gyro_scale[2]));

  // Update gyro offsets and scales on gyro driver
  int fd = open(GYRO_DEVICE_PATH, 0);
  struct gyro_scale gscale = {parameters_.gyro_offset[0] / parameters_.gyro_scale[0], parameters_.gyro_scale[0],
                              -parameters_.gyro_offset[1] / parameters_.gyro_scale[1], parameters_.gyro_scale[1], // negative offset due to RPG convention
                              -parameters_.gyro_offset[2] / parameters_.gyro_scale[2], parameters_.gyro_scale[2]}; // negative offset due to RPG convention
  if (OK != ioctl(fd, GYROIOCSSCALE, (long unsigned int)&gscale))
  {
    warn("WARNING: failed to set scale / offsets for gyro");
  }
  close(fd);

  // accel offsets and scale
  param_get(parameters_handles_.accel_offset[0], &(parameters_.accel_offset[0]));
  param_get(parameters_handles_.accel_offset[1], &(parameters_.accel_offset[1]));
  param_get(parameters_handles_.accel_offset[2], &(parameters_.accel_offset[2]));
  param_get(parameters_handles_.accel_scale[0], &(parameters_.accel_scale[0]));
  param_get(parameters_handles_.accel_scale[1], &(parameters_.accel_scale[1]));
  param_get(parameters_handles_.accel_scale[2], &(parameters_.accel_scale[2]));

  // Update accel offsets and scales on accel driver
  fd = open(ACCEL_DEVICE_PATH, 0);
  struct accel_scale ascale = {parameters_.accel_offset[0] / parameters_.accel_scale[0], parameters_.accel_scale[0],
                               -parameters_.accel_offset[1] / parameters_.accel_scale[1], parameters_.accel_scale[1], // negative offset due to RPG convention
                               -parameters_.accel_offset[2] / parameters_.accel_scale[2], parameters_.accel_scale[2]}; // negative offset due to RPG convention
  if (OK != ioctl(fd, ACCELIOCSSCALE, (long unsigned int)&ascale))
  {
    warn("WARNING: failed to set scale / offsets for accel");
  }
  close(fd);

  // magnetometer offsets and scale
  param_get(parameters_handles_.mag_offset[0], &(parameters_.mag_offset[0]));
  param_get(parameters_handles_.mag_offset[1], &(parameters_.mag_offset[1]));
  param_get(parameters_handles_.mag_offset[2], &(parameters_.mag_offset[2]));
  param_get(parameters_handles_.mag_scale[0], &(parameters_.mag_scale[0]));
  param_get(parameters_handles_.mag_scale[1], &(parameters_.mag_scale[1]));
  param_get(parameters_handles_.mag_scale[2], &(parameters_.mag_scale[2]));

  // Update magnetometer offsets and scales on magnetometer driver
  fd = open(MAG_DEVICE_PATH, 0);
  struct mag_scale mscale = {parameters_.mag_offset[0] / parameters_.mag_scale[0], parameters_.mag_scale[0],
                             -parameters_.mag_offset[1] / parameters_.mag_scale[1], parameters_.mag_scale[1], // negative offset due to RPG convention
                             -parameters_.mag_offset[2] / parameters_.mag_scale[2], parameters_.mag_scale[2]}; // negative offset due to RPG convention
  if (OK != ioctl(fd, MAGIOCSSCALE, (long unsigned int)&mscale))
  {
    warn("WARNING: failed to set scale / offsets for mag");
  }
  close(fd);

  return OK;
}

void RPGSensors::gyroInit()
{
  int fd;

  fd = open(GYRO_DEVICE_PATH, 0);

  if (fd < 0)
  {
    warn("%s", GYRO_DEVICE_PATH);
    errx(1, "FATAL: no gyro found");
  }
  else
  {

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1

    /* set the gyro internal sampling rate up to at least 1000Hz */
    if (ioctl(fd, GYROIOCSSAMPLERATE, 1000) != OK)
    {
      ioctl(fd, GYROIOCSSAMPLERATE, 800);
    }

    /* set the driver to poll at 1000Hz */
    if (ioctl(fd, SENSORIOCSPOLLRATE, 1000) != OK)
    {
      ioctl(fd, SENSORIOCSPOLLRATE, 800);
    }

#else

    /* set the gyro internal sampling rate up to at least 760Hz */
    ioctl(fd, GYROIOCSSAMPLERATE, 760);

    /* set the driver to poll at 760Hz */
    ioctl(fd, SENSORIOCSPOLLRATE, 760);

#endif

    close(fd);
  }
}

void RPGSensors::accelInit()
{
  int fd;

  fd = open(ACCEL_DEVICE_PATH, 0);

  if (fd < 0)
  {
    warn("%s", ACCEL_DEVICE_PATH);
    errx(1, "FATAL: no accelerometer found");
  }
  else
  {

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1

    /* set the accel internal sampling rate up to at leat 1000Hz */
    ioctl(fd, ACCELIOCSSAMPLERATE, 1000);

    /* set the driver to poll at 1000Hz */
    ioctl(fd, SENSORIOCSPOLLRATE, 1000);

#elif CONFIG_ARCH_BOARD_PX4FMU_V2

    /* set the accel internal sampling rate up to at leat 800Hz */
    ioctl(fd, ACCELIOCSSAMPLERATE, 800);

    /* set the driver to poll at 800Hz */
    ioctl(fd, SENSORIOCSPOLLRATE, 800);

#else
#error Need a board configuration, either CONFIG_ARCH_BOARD_PX4FMU_V1 or CONFIG_ARCH_BOARD_PX4FMU_V2

#endif

    close(fd);
  }
}

void RPGSensors::magInit()
{
  int fd;
  int ret;

  fd = open(MAG_DEVICE_PATH, 0);

  if (fd < 0)
  {
    warn("%s", MAG_DEVICE_PATH);
    errx(1, "FATAL: no magnetometer found");
  }

  /* try different mag sampling rates */
  ret = ioctl(fd, MAGIOCSSAMPLERATE, 150);
  if (ret == OK)
  {
    /* set the pollrate accordingly */
    ioctl(fd, SENSORIOCSPOLLRATE, 150);
  }
  else
  {
    ret = ioctl(fd, MAGIOCSSAMPLERATE, 100);

    /* if the slower sampling rate still fails, something is wrong */
    if (ret == OK)
    {
      /* set the driver to poll also at the slower rate */
      ioctl(fd, SENSORIOCSPOLLRATE, 100);
    }
    else
    {
      errx(1, "FATAL: mag sampling rate could not be set");
    }
  }

  ret = ioctl(fd, MAGIOCGEXTERNAL, 0);

  if (ret < 0)
  {
    errx(1, "FATAL: unknown if magnetometer is external or onboard");
  }

  close(fd);
}

void RPGSensors::baroInit()
{
  int fd;

  fd = open(BARO_DEVICE_PATH, 0);

  if (fd < 0)
  {
    warn("%s", BARO_DEVICE_PATH);
    errx(1, "FATAL: No barometer found");
  }

  /* set the driver to poll at 150Hz */
  ioctl(fd, SENSORIOCSPOLLRATE, 150);

  close(fd);
}

void RPGSensors::imuPoll(struct imu_msg_s &imu_msg)
{
  // Double check if we really got a gyro measurement
  bool gyro_updated;
  orb_check(gyro_sub_, &gyro_updated);

  if (gyro_updated)
  {
    struct gyro_report gyro_report;
    orb_copy(ORB_ID(sensor_gyro), gyro_sub_, &gyro_report);

    // Set the timestamp from the gyro (the accelerometer reading should always have the same timestamp)
    imu_msg.timestamp = gyro_report.timestamp;

    // Set gyro readings into imu msg
    imu_msg.gyro_x = gyro_report.x;
    imu_msg.gyro_y = -gyro_report.y; // RPG coordinates
    imu_msg.gyro_z = -gyro_report.z; // RPG coordinates
  }

  // Check if we also got an accelerometer measurement (this should always be the case since they are read at the same time)
  bool accel_updated;
  orb_check(accel_sub_, &accel_updated);

  if (accel_updated)
  {
    // Get accel readings and set them into imu msg
    struct accel_report accel_report;
    orb_copy(ORB_ID(sensor_accel), accel_sub_, &accel_report);

    imu_msg.acc_x = accel_report.x;
    imu_msg.acc_y = -accel_report.y; // RPG coordinates
    imu_msg.acc_z = -accel_report.z; // RPG coordinates
  }
}

void RPGSensors::magPoll(struct mag_msg_s &mag_msg)
{
  // Double check if we really got a magnetometer measurement
  bool mag_updated;
  orb_check(mag_sub_, &mag_updated);

  if (mag_updated)
  {
    // Get magnetometer readings and set them into mag msg
    struct mag_report mag_report;
    orb_copy(ORB_ID(sensor_mag), mag_sub_, &mag_report);

    mag_msg.timestamp = mag_report.timestamp;
    mag_msg.x = mag_report.x;
    mag_msg.y = -mag_report.y; // RPG coordinates
    mag_msg.z = -mag_report.z; // RPG coordinates
  }
}

void RPGSensors::taskMainTrampoline(int argc, char *argv[])
{
  rpg_sensors_ptr->taskMain();
}

void RPGSensors::taskMain()
{
  // Initialize sensors
  gyroInit();
  accelInit();
  magInit();
  baroInit();

  // Start subscribing
  params_sub_ = orb_subscribe(ORB_ID(parameter_update));
  gyro_sub_ = orb_subscribe(ORB_ID(sensor_gyro));
  accel_sub_ = orb_subscribe(ORB_ID(sensor_accel));
  mag_sub_ = orb_subscribe(ORB_ID(sensor_mag));

  // Rate limit imu subscription
  if (imu_poll_interval_ > 1)
  {
    orb_set_interval(gyro_sub_, imu_poll_interval_);
  }

  struct imu_msg_s imu_msg;
  struct mag_msg_s mag_msg;

  // Get initial imu reading
  imuPoll(imu_msg);
  magPoll(mag_msg);

  imu_pub_ = orb_advertise(ORB_ID(imu_msg), &imu_msg);
  mag_pub_ = orb_advertise(ORB_ID(mag_msg), &mag_msg);

  struct pollfd fds[2];
  fds[0].fd = gyro_sub_;
  fds[0].events = POLLIN;
  fds[1].fd = mag_sub_;
  fds[1].events = POLLIN;

  while (!task_should_exit_)
  {
    int poll_ret = poll(fds, 2, 20);
    if (poll_ret > 0 && (fds[0].revents & POLLIN))
    {
      // Get received values from gyros and accelerometer
      imuPoll(imu_msg);

      // Publish IMU message
      orb_publish(ORB_ID(imu_msg), imu_pub_, &imu_msg);
    }

    if (poll_ret > 0 && (fds[1].revents & POLLIN))
    {
      // Get received values from magnetometer
      magPoll(mag_msg);

      // Publish mag message
      orb_publish(ORB_ID(mag_msg), mag_pub_, &mag_msg);
    }

    // Update parameters if we received new ones
    bool param_updated;
    orb_check(params_sub_, &param_updated);
    if (param_updated)
    {
      parametersUpdate();
    }
  }

  sensor_task_ = -1;
  _exit(0);
}

int RPGSensors::start()
{
  ASSERT(sensor_task_ == -1);

  /* start the task */
  sensor_task_ = task_spawn_cmd("rpg_sensors_task", SCHED_DEFAULT,
  SCHED_PRIORITY_MAX - 5,
                                2048, (main_t)&RPGSensors::taskMainTrampoline, nullptr);

  if (sensor_task_ < 0)
  {
    warn("task start failed");
    return -errno;
  }

  return OK;
}

} // namespace rpg_sensors

int rpg_sensors_main(int argc, char *argv[])
{
  if (argc < 1)
  {
    errx(1, "usage: sensors {start|stop|status}");
  }

  if (!strcmp(argv[1], "start"))
  {

    if (rpg_sensors::rpg_sensors_ptr != nullptr)
    {
      errx(0, "rpg_sensors is already running");
    }

    rpg_sensors::rpg_sensors_ptr = new rpg_sensors::RPGSensors;

    if (rpg_sensors::rpg_sensors_ptr == nullptr)
    {
      errx(1, "allocation failed");
    }

    for (int i = 0; i < argc && argv[i]; i++)
    {
      if (strcmp(argv[i], "--imu_poll_interval") == 0)
      {
        if (argc > i + 1)
        {
          rpg_sensors::rpg_sensors_ptr->imu_poll_interval_ = strtoul(argv[i + 1], NULL, 10);
        }
      }
    }

    if (OK != rpg_sensors::rpg_sensors_ptr->start())
    {
      delete rpg_sensors::rpg_sensors_ptr;
      rpg_sensors::rpg_sensors_ptr = nullptr;
      err(1, "start failed");
    }

    exit(0);
  }

  if (!strcmp(argv[1], "stop"))
  {
    if (rpg_sensors::rpg_sensors_ptr == nullptr)
    {
      errx(1, "rpg_sensors is not running");
    }

    delete rpg_sensors::rpg_sensors_ptr;
    rpg_sensors::rpg_sensors_ptr = nullptr;
    exit(0);
  }

  if (!strcmp(argv[1], "status"))
  {
    if (rpg_sensors::rpg_sensors_ptr)
    {
      errx(0, "rpg_sensors is running");
    }
    else
    {
      errx(1, "rpg_sensors is not running");
    }
  }

  warnx("unrecognized command");
  return 1;
}

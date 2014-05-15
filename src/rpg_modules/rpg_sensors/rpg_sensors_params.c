/*
 * rpg_sensors_params.c
 *
 *  Created on: May 12, 2014
 *      Author: matthias
 */

#include <nuttx/config.h>
#include <systemlib/param/param.h>

/**
 * Gyro X-axis offset
 *
 * @min -10.0
 * @max 10.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_GYRO_XOFF, 0.0f);

/**
 * Gyro Y-axis offset
 *
 * @min -10.0
 * @max 10.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_GYRO_YOFF, 0.0f);

/**
 * Gyro Z-axis offset
 *
 * @min -5.0
 * @max 5.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_GYRO_ZOFF, 0.0f);

/**
 * Gyro X-axis scaling factor
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_GYRO_XSCALE, 1.0f);

/**
 * Gyro Y-axis scaling factor
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_GYRO_YSCALE, 1.0f);

/**
 * Gyro Z-axis scaling factor
 *
 * @min -1.5
 * @max 1.5
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_GYRO_ZSCALE, 1.0f);

/**
 * Accelerometer X-axis offset
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_ACC_XOFF, 0.0f);

/**
 * Accelerometer Y-axis offset
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_ACC_YOFF, 0.0f);

/**
 * Accelerometer Z-axis offset
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_ACC_ZOFF, 0.0f);

/**
 * Accelerometer X-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_ACC_XSCALE, 1.0f);

/**
 * Accelerometer Y-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_ACC_YSCALE, 1.0f);

/**
 * Accelerometer Z-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_ACC_ZSCALE, 1.0f);

/**
 * Magnetometer X-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_MAG_XOFF, 0.0f);

/**
 * Magnetometer Y-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_MAG_YOFF, 0.0f);

/**
 * Magnetometer Z-axis offset
 *
 * @min -500.0
 * @max 500.0
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_MAG_ZOFF, 0.0f);

/**
 * Magnetometer X-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_MAG_XSCALE, 1.0f);

/**
 * Magnetometer Y-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_MAG_YSCALE, 1.0f);

/**
 * Magnetometer Z-axis scaling factor
 *
 * @group Sensor Calibration
 */
PARAM_DEFINE_FLOAT(SENS_MAG_ZSCALE, 1.0f);

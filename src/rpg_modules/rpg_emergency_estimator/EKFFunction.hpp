/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file KalmanNav.hpp
 *
 * kalman filter navigation code
 */

#pragma once

//#define MATRIX_ASSERT
//#define VECTOR_ASSERT

#include <nuttx/config.h>

#include <mathlib/mathlib.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
// Listen to Sensor Combined now until "custom"-sensors resolved!
#include <uORB/topics/sensor_combined.h>

#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_baro.h>
//#include <drivers/drv_mag.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_gpio.h>

#include <poll.h>
#include <unistd.h>

/**
 * Extended Kalman filter function class
 * http://en.wikipedia.org/wiki/Extended_Kalman_filter
 * Discrete-time extended Kalman filter
 */
class EKFFunction : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	EKFFunction(SuperBlock *parent, const char *name); //NULL, why?

	/**
	 * Deconstuctor
	 */

	virtual ~EKFFunction() {};

	// Initialization
//	math::Vector<9> init();
	// INPUT HAS TO BE SET HERE ~ Maybe no use -- originally it is used to have an initial quaternion

	/**
	 * The main callback function for the class
	 */
	void update();


	/**
	 * Publication update
	 */
	virtual void updatePublications();

	/**
	 * State prediction
	 * Continuous, non-linear
	 */
	int predictState(float dt);

	/**
	 * IMU - Drag Correction
	 */
	int correctIMU();

	/**
	 * Barometer Altitude Correction
	 */
	int correctBar();

	/**
	 * Overloaded update parameters
	 */
	virtual void updateParams();

protected:
	// Extended Kalman filter
	// Prediction:
	math::Matrix<9,9> A;             /**< Jacobian(f,x), where dx/dt = f(x,u) */
	math::Matrix<9,4> U;             /**< input shaping matrix for gyro/accel */
	math::Matrix<9,9> Q;             /**< process noise Matrix*/
	math::Matrix<9,9> P;             /**< state covariance matrix */
	math::Matrix<9,9> P_0;            /**< initial state covariance matrix */
	math::Matrix<4,4> Sigma_u;             /**< gyro/ accel noise matrix */
	double f_vec[9];				 // Estimator Prediction Function/Vector

	math::Matrix<2,9> HDrag;          /**< drag measurement matrix */
	math::Matrix<2,2> RDrag;          /**< drag measurement noise matrix */

	math::Matrix<1,9> HPress;          /**< pressure measurement jacobian matrix */
	math::Matrix<1,1> RPress;          /**< position measurement noise matrix */

	// subscriptions
	// TODO Edit this accordingly...
	uORB::Subscription<sensor_combined_s> _sensors;          /**< sensors sub. */
	uORB::Subscription<parameter_update_s> _param_update;    /**< parameter update sub. */
//
//	// publications
	uORB::Publication<vehicle_local_position_s> _localPos;   /**< local position pub. */
	uORB::Publication<vehicle_attitude_s> _att;              /**< attitude pub. */

	/* Of course ADD also velocity ... */

	// time stamps
	uint64_t _pubTimeStamp;     /**< output data publication time stamp */
	uint64_t _predictTimeStamp; /**< prediction time stamp */
	uint64_t _correctTimeStamp; /**< correction time stamp */
	uint64_t _outTimeStamp;     /**< output time stamp */
	// frame count
	uint16_t _navFrames;        /**< navigation frames completed in output cycle */
	// What are in navigation frames?!

	// miss counts
	uint16_t _miss;         	/**< number of times fast prediction loop missed */


	// SOMEHOW HERE ITS STATE Declaration & Stuff:
	// states
	enum {HW = 0, UB, VB, WB, QW, QX, QY, QZ, P0};  /**< state enumeration  - to access!*/
	// TODO could be done nicer
	float h_W;           /**< height in world frame */
	float u_B, v_B, w_B;                  /**< navigation velocity in body frame, m/s */
	float q_w, q_x, q_y, q_z;                       /**< quaternion, [-] */
	double p_0;                   	/**< reference pressure */

	// parameters to publish and helper:
	math::Matrix<3,3> R_WB;
	float phi, theta, psi;

//	struct map_projection_reference_s ref;	/**< local projection reference */
	float h_0;                   		/**<  refeerence altitude (ground height) */
	float last_press; // Pressure to see if changed
	//Load Parameters
	// Initializing constant parameters

	control::BlockParamFloat _uGyro;      /**< gyro process noise */
	control::BlockParamFloat _uAccel;     /**< accelerometer process noise  */
	control::BlockParamFloat _rDrag;     /**< accelerometer measurement noise */
	control::BlockParamFloat _rPress;     /**< accelerometer measurement noise */
	control::BlockParamFloat _faultSonar;   /**< fault detection threshold for position */
	control::BlockParamFloat _thresSonar;   /**< fault detection threshold for attitude */

	// status
	// TODO
//	bool _attitudeInitialized; // Do own initialization -- copied from att_pos_estimator_kf
//	bool _heightInitialized;
	// lets see counter?!
	uint16_t _attitudeInitCounter;
	// accessors - were defined for GPS conversion...
};

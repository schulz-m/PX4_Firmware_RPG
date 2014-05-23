/**
 * @file EKFFunction.hpp
 *
 * EKFFunction Class
 */

#pragma once

//#define MATRIX_ASSERT
//#define VECTOR_ASSERT

#include <nuttx/config.h>

#include <mathlib/mathlib.h>
#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

// Topics:
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>

#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
//#include <uORB/topics/vehicle_control_mode.h>
//#include <uORB/topics/vehicle_attitude.h>
//#include <uORB/topics/vehicle_local_position.h>

// Drivers
#include <drivers/drv_hrt.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>

//additional
#include <drivers/drv_baro.h>
#include <uORB/topics/rpg/imu_msg.h>
#include <uORB/topics/rpg/sonar_msg.h>
// added: ekf topic
#include <uORB/topics/rpg/emergency_ekf_msg.h>

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

	// TODO Sonar Correction

	int correctSonar();

	/**
	 * Overloaded update parameters
	 */
	virtual void updateParams();

	float abs_float(float input_abs); // Yes indeed, you need to write your own abs function...

protected:
	// Extended Kalman filter
	// Prediction:
	math::Matrix<9,9> A;             /**< Jacobian(f,x), where dx/dt = f(x,u) */
	math::Matrix<9,6> U;             /**< input shaping matrix for gyro/accel */
	math::Matrix<9,9> Q;             /**< process noise Matrix*/
	math::Matrix<9,9> P;             /**< state covariance matrix */
	math::Matrix<9,9> P_0;            /**< initial state covariance matrix */
	math::Matrix<6,6> Sigma_u;             /**< gyro/ accel noise matrix */
	double f_vec[9];				 // Estimator Prediction Function/Vector

	math::Matrix<2,9> HDrag;          /**< drag measurement matrix */
	math::Matrix<2,2> RDrag;          /**< drag measurement noise matrix */

	math::Matrix<1,9> HPress;          /**< pressure measurement jacobian matrix */
	math::Matrix<1,1> RPress;          /**< position measurement noise matrix */

	math::Matrix<1,9> HSonar;          /**< pressure measurement jacobian matrix */
	math::Matrix<1,1> RSonar;          /**< position measurement noise matrix */

	// subscriptions
	  struct imu_msg_s _imu_msg;
	  int _imu_sub;

	  struct baro_report _bar_msg;
	  int _bar_sub;

	  struct sonar_msg_s _sonar_msg;
	  int _sonar_sub;

	uORB::Subscription<parameter_update_s> _param_update;    /**< parameter update sub. */

	// publications
	uORB::Publication<emergency_ekf_msg_s> _emergency_ekf;   /**< custom message pub. */

	// time stamps
	uint64_t _pubTimeStamp;     /**< output data publication time stamp */
	uint64_t _predictTimeStamp; /**< prediction time stamp */
	uint64_t _correctTimeStamp; /**< correction time stamp */
	uint64_t _outTimeStamp;     /**< output time stamp */

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

	float h_0;                   		/**<  refeerence altitude (ground height) */

	float b_s;						    /**<  ground bias for sonar estimation */

	// Initializing constant parameters

	control::BlockParamFloat _uGyro;      /**< gyro process noise */
	control::BlockParamFloat _uAccel;     /**< accelerometer process noise  */ //TODO Comment
	control::BlockParamFloat _rDrag;     /**< accelerometer measurement noise */
	control::BlockParamFloat _rPress;     /**< accelerometer measurement noise */
	control::BlockParamFloat _rSonar;     /**< accelerometer measurement noise */
	control::BlockParamFloat _faultSonar;   /**< fault detection threshold for position */
	control::BlockParamFloat _thresSonar;   /**< fault detection threshold for attitude */

};

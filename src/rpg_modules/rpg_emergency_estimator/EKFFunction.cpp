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
 * @file KalmanNav.cpp
 *
 * Kalman filter navigation code
 */

#include <poll.h>

#include "EKFFunction.hpp"
#include <systemlib/err.h>
//#include <../lib/mathlib/mathlib.h>
//#include <mathlib/mathlib.h>

// Include codegen headers:
#ifdef __cplusplus
extern "C" {
#endif
#include "codegen/composeCPPPrediction_initialize.h"
#include "codegen/composeCPPPrediction.h"
#ifdef __cplusplus
}
#endif

//Physical Parameters: - Should I have this here or also as environment variable?
static const float g_0 = 9.806f; // [m/s^2] standard gravitational accel.
static const float R_0 = 287.0f; // [J/kgK] Physical Gas constant of Air
static const float T_0 = 273.15 + 15; // [K] Ambient Temperature ~ Assumed to be constant!
static const float mu_N = 0.4; // [1/s] - Normalized Drag coefficient

// Function Parameters
static const int8_t ret_ok = 0; 		// no error in function
static const int8_t ret_error = -1; 	// error occurred

EKFFunction::EKFFunction(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),

	// subscriptions
	_sensors(&getSubscriptions(), ORB_ID(sensor_combined), 10), // limit to 100 Hz
	_param_update(&getSubscriptions(), ORB_ID(parameter_update), 1000), // limit to 1 Hz
	// publications
	_localPos(&getPublications(), ORB_ID(vehicle_local_position)),
	_att(&getPublications(), ORB_ID(vehicle_attitude)),
	// Timestamps
	_pubTimeStamp(hrt_absolute_time()),
	_predictTimeStamp(hrt_absolute_time()),
	_correctTimeStamp(hrt_absolute_time()),
	_outTimeStamp(hrt_absolute_time()),
	// frame count - don't understand...
	_navFrames(0),
	// miss counts
	_miss(0),
	// state
	h_W(0),
	u_B(0), v_B(0), w_B(0),
	q_w(1), q_x(0), q_y(0), q_z(0),
	p_0(0),
	phi(0),theta(0),psi(0),
	h_0(0),
	_uGyro(this, "U_GYRO"),
	_uAccel(this, "U_ACCEL"),
	_rDrag(this, "R_DRAG"),
	_rPress(this, "R_PRESS"),
	_faultSonar(this, "FAULT_SONAR"),
	_thresSonar(this, "THRES_SONAR"),
//	_attitudeInitialized(false),
//	_heightInitialized(false),
	_attitudeInitCounter(0)
{

	using namespace math;

	// TODO look where ref is defined...
//	memset(&ref, 0, sizeof(ref));

	// Set all matrices to zero for some reason..
	A.zero();
	U.zero();
	Q.zero();
	P.zero();
	Sigma_u.zero();
	HDrag.zero();
	RDrag.zero();
	HPress.zero();
	RPress.zero();

	R_WB.identity();

	// initial state covariance matrix
	float est_cov_array[9] = {0.1,0.05,0.05,0.05,5*pow(10,-5),5*pow(10,-5),5*pow(10,-5),5*pow(10,-5),0.1};
	P_0.from_diagonal(est_cov_array);
	P = P_0;

	// Also Initialize Q here! tuning parameter and not dependent on Noise, overview hard otherwise..
	float q_array[9] = {0.05,0.01,0.01,0.2,pow(10,-6),pow(10,-6),pow(10,-6),pow(10,-6),0.1};
	Q.from_diagonal(q_array);

	_sensors.update();
	// initial state
	h_W = 0.0f;
	u_B = 0.0f;
	v_B = 0.0f;
	w_B = 0.0f;
	q_w = 1.0f;
	q_x = 0.0f;
	q_y = 0.0f;
	q_z = 0.0f;
	p_0 = _sensors.baro_pres_mbar;
	h_0 = 0; // TODO Initialize this height...

	// HDrag is constant
	HDrag(0, 1) = -mu_N;
	HDrag(1, 2) = -mu_N;

	// TODO HSonar will be constant as well...
	// initialize all parameters
	updateParams();
}

// This function is called immediatelly at EKFFunction Class Creation
// Seperate because of Parameter getting
void EKFFunction::updateParams()
{
	using namespace math;
	using namespace control;
	SuperBlock::updateParams();

	// gyro noise
	Sigma_u(0, 0) = _uGyro.get();   // gyro x, rad/s
	Sigma_u(1, 1) = _uGyro.get();   // gyro y
	Sigma_u(2, 2) = _uGyro.get();   // gyro z

	// accel noise
	Sigma_u(3, 3) = _uAccel.get();   // accel z, m/s^2

	RDrag(0, 0) = _rDrag.get();   // accel x - drag
	RDrag(1, 1) = _rDrag.get();   // accel y - drag

	RPress(0, 0) = _rPress.get(); // Barometer Height Calculation

}

void EKFFunction::update()
{
	using namespace math;

	// poll defined here... _sensors nice structure
	struct pollfd fds[1];
	fds[0].fd = _sensors.getHandle();
	fds[0].events = POLLIN;

	warnx("EKFFunction update method is triggered");

	// poll for new data
	int ret = poll(fds, 1, 1000);
	// 1000 timeout ...

	if (ret < 0) {
		return;

	} else if (ret == 0) { // timeout
		return;
	}

	// get new timestamp
	uint64_t newTimeStamp = hrt_absolute_time();

	// check updated subscriptions
	if (_param_update.updated()) updateParams();

	// Check if updated
	bool sensorsUpdate = _sensors.updated();

	// get new information from subscriptions
	// this clears update flag
	updateSubscriptions();

//	TODO Use this somehow...
//			// Wait some time for this!
//			warnx("initialized EKF attitude");
//			// Put Conversion in here... this will be 0...
//			warnx("phi: %8.4f, theta: %8.4f, psi: %8.4f",
//			       double(phi), double(theta), double(psi));

	// prediction step
	// using sensors timestamp so we can account for packet lag
	float dt = (_sensors.timestamp - _predictTimeStamp) / 1.0e6f;
	// Show Timestamps here:
	printf("dt: %15.10f\n", double(dt));

	_predictTimeStamp = _sensors.timestamp;

	// TODO See where to do this initialization
	// Initialize Prediction
	composeCPPPrediction_initialize();

	// don't predict if time greater than a second
	if (dt < 1.0f) {
		warnx("Trying to predict");
		predictState(dt);
		warnx("Predicted");
//		predictStateCovariance(dt);
		// count fast frames TODO why navframes?
		_navFrames += 1;
	}

	// count times 100 Hz rate isn't met
	if (dt > 0.01f) _miss++;

	//correct immediatelly: (~recursive)
	warnx("Trying to correct");
	correctIMU();
	warnx("Corrected");

	// do barometer correction only for 50 Hz ... this will be done differently!
//	if (sensorsUpdate 								// new data
//	    && _sensors.timestamp - _correctTimeStamp > 1e6 / 50 	// 50 Hz
//	   ) {
//		_correctTimeStamp = _sensors.timestamp;
//		correctBar();
//	}

	// publication
	if (newTimeStamp - _pubTimeStamp > 1e6 / 50) { // 50 Hz
		_pubTimeStamp = newTimeStamp;
		// is it this printing?
//		printf("Roll[deg] %4.3f\n",phi/180*M_PI);
//		printf("Pitch[deg] %4.3f\n",theta/180*M_PI);
//		printf("Yaw[deg] %4.3f\n",psi/180*M_PI);
		updatePublications();
	}

	// output
	if (newTimeStamp - _outTimeStamp > 10e6) { // 0.1 Hz
		_outTimeStamp = newTimeStamp;
		//printf("nav: %4d Hz, miss #: %4d\n",
		//       _navFrames / 10, _miss / 10);
		_navFrames = 0;
		_miss = 0;
	}
}

void EKFFunction::updatePublications()
{
	using namespace math;

	// local position publication
//	float x;
//	float y;

//	TODO TODO TODO XXX - somehow publications let the system crash...
	// Landing Bool - Doesn't fit for emergency procedure...
//	bool landed = h_W < (h_0 + 0.1); // XXX improve?
	_localPos.timestamp = _pubTimeStamp;
	_localPos.xy_valid = true;
	_localPos.z_valid = true;
	_localPos.v_xy_valid = true;
	_localPos.v_z_valid = true;
	_localPos.z = h_W;
	_localPos.vx = u_B;
	_localPos.vy = v_B;
	_localPos.vz = w_B;
	_localPos.yaw = psi;
	_localPos.xy_global = true;
	_localPos.z_global = true;
	_localPos.ref_timestamp = _pubTimeStamp;
	_localPos.ref_alt = 0;
	// TODO check this topic publication ~ should be handled by commander
//	_localPos.landed = landed;

	// attitude publication
	_att.timestamp = _pubTimeStamp;
	// TODO, check Conversion Function
	_att.roll = phi;
	_att.pitch = theta;
	_att.yaw = psi;
	_att.rollspeed = _sensors.gyro_rad_s[0];
	_att.pitchspeed = _sensors.gyro_rad_s[1];
	_att.yawspeed = _sensors.gyro_rad_s[2];
	// TODO, add gyro offsets to filter, somehow input?!
	_att.rate_offsets[0] = 0.0f;
	_att.rate_offsets[1] = 0.0f;
	_att.rate_offsets[2] = 0.0f;

	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++)
			_att.R[i][j] = R_WB(i, j);

	_att.q[0] = q_w;
	_att.q[1] = q_x;
	_att.q[2] = q_y;
	_att.q[3] = q_z;

	_att.R_valid = true;
	_att.q_valid = true;

	// Selectively update - interresting :-)
	// selectively update publications,
	// do NOT call superblock do-all method
		_localPos.update();

		_att.update();
}

int EKFFunction::predictState(float dt)
{
	using namespace math;

	double state[9] = {h_W, u_B, v_B, w_B, q_w, q_x, q_y, q_z, p_0};
	double inputs[6] = {_sensors.gyro_rad_s[0],_sensors.gyro_rad_s[1],_sensors.gyro_rad_s[2],
			_sensors.accelerometer_m_s2[0],_sensors.accelerometer_m_s2[1],_sensors.accelerometer_m_s2[2]};
//	double inputs[6] = {0,0,0,0,0,0};
	double parameters[8] = {0.5, g_0, R_0, T_0, mu_N, dt, h_0};

	// Matrices.. will be changed in the function:
	double f_vec_temp[9];
	double A_matrix_temp[81];
	double U_matrix_temp[54];

	// Somehow this should work.. check if only value or adress is forwarded ^^ - Array/Pointer zusammenhang
	composeCPPPrediction(state,inputs, parameters,f_vec_temp,A_matrix_temp,U_matrix_temp);

	// Predict State
	h_W = f_vec[0];
	u_B = f_vec[1];
	v_B = f_vec[2];
	w_B = f_vec[3];
	q_w = f_vec[4];
	q_x = f_vec[5];
	q_y = f_vec[6];
	q_z = f_vec[7];
	p_0 = f_vec[8];

	for (int i = 0; i < 9; i++) for (int j = 0; j < 9; j++)
	A(i, j) = A_matrix_temp[i * 9 + j];

	for (int i = 0; i < 4; i++) for (int j = 0; j < 9; j++)
    U(i, j) = U_matrix_temp[i * 9 + j];

	// Predict State Covariance
	P = A * P * A.transposed() + U * Sigma_u * U.transposed() + Q;

	return ret_ok;
}

int EKFFunction::predictStateCovariance(float dt)
{
	using namespace math;

/* TODO I DONT USE THIS METHOD */

	//Sigma_u & Q Defined below...

	// continuous prediction equations
	// for discrete time EKF
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
//	P = P + (A * P + P * A.transposed() + U * Sigma_u * U.transposed() + Q) * dt;

	return ret_ok;
}

int EKFFunction::correctIMU()
{
	using namespace math;

//	double state[9] = {h_W, u_B, v_B, w_B, q_w, q_x, q_y, q_z, p_0}; //TODO doesnt work...
	Vector<9> s_predict;

	// This is not stupid at all:
	s_predict(0) = h_W;
	s_predict(1) = u_B;
	s_predict(2) = v_B;
	s_predict(3) = w_B;
	s_predict(4) = q_w;
	s_predict(5) = q_x;
	s_predict(6) = q_y;
	s_predict(7) = q_z;
	s_predict(8) = p_0;

	// accel measurement
	Vector<2> zAccel({_sensors.accelerometer_m_s2[0],_sensors.accelerometer_m_s2[1]});
//	Vector<2> zAccel({0,0});

	// accel predicted measurement
	Vector<2> zAccelHat({-mu_N*u_B,-mu_N*v_B});

	// calculate residual
	Vector<2> y(zAccel(0) - zAccelHat(0), zAccel(1) - zAccelHat(1));

	// compute correction
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	Matrix<2,2> S = HDrag * P * HDrag.transposed() + RDrag; // residual covariance
	Matrix<9, 2> K = P * HDrag.transposed() * S.inversed();
	Vector<9> sCorrect = s_predict +  K * y;

	// check correciton is sane
	for (size_t i = 0; i < sCorrect.get_size(); i++) {
		float val = sCorrect(i);

		if (isnan(val) || isinf(val)) {
			// abort correction and return
			// TODO Analyze numerical failure for drag correction! (and make usleep...)
			warnx("numerical failure in drag correction"); // Yeah this somehow happens ...
			// reset P matrix to P0
			P = P_0;
			return ret_error;
		}
	}

	// update state covariance
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	P = P - K * HDrag * P;

	//Return State:
	// TODO could all be done better with some state vector over all...
	h_W = f_vec[HW];
	u_B = f_vec[UB];
	v_B = f_vec[VB];
	w_B = f_vec[WB];
	q_w = f_vec[QW];
	q_x = f_vec[QX];
	q_y = f_vec[QY];
	q_z = f_vec[QZ];
	p_0 = f_vec[P0];

	// Get Rotation Matrices out:
	float temp_array[4] = {q_w,q_x,q_y,q_z};
	Quaternion q(temp_array);
	R_WB = q.to_dcm();

	// euler update
	Vector<3> euler = R_WB.to_euler();
	phi = euler.data[0];
	theta = euler.data[1];
	psi = euler.data[2];

	return ret_ok;
}

int EKFFunction::correctBar()
{
	using namespace math;

// Only one dimensional corrections...

	return ret_ok;
}



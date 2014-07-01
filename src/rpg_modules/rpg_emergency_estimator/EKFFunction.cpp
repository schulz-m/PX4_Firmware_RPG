/**
 * @file EKFFunction.cpp
 *
 * Kalman filter navigation code
 */

//For the implementation first a direct EKF is tested without sonar bias state.

#include <poll.h>

#include "EKFFunction.hpp"
#include <systemlib/err.h>

// Include codegen headers:
#ifdef __cplusplus
extern "C" {
#endif
#include "codegen/composeCPPPrediction_initialize.h"
#include "codegen/composeCPPPrediction.h"
#ifdef __cplusplus
}
#endif

//Physical Parameters:
static const float g_0 = 9.806f; 		// [m/s^2] standard gravitational accel.
static const float R_0 = 287.0f; 		// [J/kgK] Physical Gas constant of Air
static const float T_0 = 273.15 + 15;	// [K] Ambient Temperature ~ Assumed to be constant!
static const float mu_N = 0.4; 		// [1/s] - Normalized Drag coefficient

//Memory Parameters:
static float last_barometer_pressure;
static float last_sonar_down;

// Static to compare maximum computation times:
//static float max_comp_time = 0;

// Function Parameters
static const int8_t ret_ok = 0; 		// no error in function
static const int8_t ret_error = -1; 	// error occurred

EKFFunction::EKFFunction(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),

	// subscriptions - Other ones in cunstructor below
	_param_update(&getSubscriptions(), ORB_ID(parameter_update), 1000), // limit to 1 Hz

	// Timestamps
	_pubTimeStamp(hrt_absolute_time()),
	_predictTimeStamp(hrt_absolute_time()),
	_correctTimeStamp(hrt_absolute_time()),
	_outTimeStamp(hrt_absolute_time()),
	// state
	h_W(0),
	u_B(0), v_B(0), w_B(0),
	q_w(1), q_x(0), q_y(0), q_z(0),
	p_0(0),
	phi(0),theta(0),psi(0),
	h_0(0),
	b_T(0),
	_uGyro(this, "U_GYRO"),
	_uAccel(this, "U_ACCEL"),
	_rDrag(this, "R_DRAG"),
	_rPress(this, "R_PRESS"),
	_rSonar(this,"R_SONAR"),
	_faultSonar(this, "FAULT_SONAR"),
	_thresSonar(this, "THRES_SONAR"),
    imuUpdate(false),
    baroUpdate(false),
    sonarUpdate(false)
{

	using namespace math;

	//Subscribers:
	  memset(&_imu_msg, 0, sizeof(_imu_msg));
	  _imu_sub = orb_subscribe(ORB_ID(imu_msg));
	  orb_set_interval(_imu_sub, 5); //200 Hz
	  memset(&_bar_msg, 0, sizeof(_bar_msg));
	  _bar_sub = orb_subscribe(ORB_ID(sensor_baro));
	  orb_set_interval(_bar_sub, 10); //100 Hz
	  _sonar_sub = orb_subscribe(ORB_ID(sonar_msg));
	  orb_set_interval(_sonar_sub, 20); //50 Hz ~ (20 Hz in datasheet)

	//Publisher:
	  memset(&_emergency_ekf_msg, 0, sizeof(_emergency_ekf_msg));
	  _emergency_ekf_pub = orb_advertise(ORB_ID(emergency_ekf_msg), &_emergency_ekf_msg);

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
	HSonar.zero();
	RSonar.zero();

	R_WB.identity();

	// Initial state covariance matrix
	float est_cov_array[9] = {0.1,0.01,0.01,0.01,1*pow(10,-5),1*pow(10,-5),1*pow(10,-5),1*pow(10,-5),0.1};
	P_0.from_diagonal(est_cov_array);
	P = P_0;

	// Process Noise - Tuning Parameter
	float q_array[9] = {0.05,0,0,0.2,pow(10,-6),pow(10,-6),pow(10,-6),pow(10,-6),0};
	Q.from_diagonal(q_array);

	// Get Sensor Msgs:
	 orb_copy(ORB_ID(imu_msg), _imu_sub, &_imu_msg);
	 orb_copy(ORB_ID(sensor_baro), _bar_sub, &_bar_msg);
	 orb_copy(ORB_ID(sonar_msg), _sonar_sub, &_sonar_msg);

	// Initialize differently for sonar range:
	if (_sonar_msg.sonar_down > _faultSonar.get())
	{
	   h_W = 0;
	   last_sonar_down = 0;
	   Q(8,8) = 0;
	}
	else
	{
	   h_W = _sonar_msg.sonar_down;
	   last_sonar_down = _sonar_msg.sonar_down;
	}

	// initial state
	u_B = 0.0f;
	v_B = 0.0f;
	w_B = 0.0f;
	q_w = 1.0f;
	q_x = 0.0f;
	q_y = 0.0f;
	q_z = 0.0f;
	p_0 = 0; // Changed in baro correction

	// Initialize statics:
	last_barometer_pressure = 0;

	// HDrag is constant
	HDrag(0, 1) = -mu_N;
	HDrag(1, 2) = -mu_N;

	// Only one dimensional corrections... Sonar, change if bias state
	HSonar(0,0) = 1;

	// Initialize Prediction
	composeCPPPrediction_initialize();

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
	Sigma_u(4, 4) = _uAccel.get();   // accel z, m/s^2
	Sigma_u(5, 5) = _uAccel.get();   // accel z, m/s^2

	RDrag(0, 0) = _rDrag.get();   // accel x - drag
	RDrag(1, 1) = _rDrag.get();   // accel y - drag

	RPress(0, 0) = _rPress.get(); // Barometer Height Calculation

	RSonar(0, 0) = _rSonar.get(); // Sonar Height Calculation

}

void EKFFunction::update()
{
	using namespace math;

	float dt;
	float dt_sonar = 0.02; // Assumed to be constant for threshold function
	// poll defined here:
	struct pollfd fds[3];
	fds[0].fd = _imu_sub; //_imu_data.getHandle();
	fds[0].events = POLLIN;
	fds[1].fd = _bar_sub;//_bar_data.getHandle();
    fds[1].events = POLLIN;
    fds[2].fd = _sonar_sub;
    fds[2].events = POLLIN;

	// poll for new data
	int ret = poll(fds, 3, 100); // 0.1s timeout

	if (ret < 0) {
		return;

	} else if (ret == 0) { // timeout
		warnx("TimeOut occured, long time no sensor data");
		return;
	}

	// get new timestamp
	uint64_t newTimeStamp = hrt_absolute_time();

	// check updated subscriptions
	if (_param_update.updated()) updateParams();

	// get new information from subscriptions
	// this clears update flag
	updateSubscriptions();

	//Get Msg
	if (fds[0].revents & POLLIN)
	{
		orb_copy(ORB_ID(imu_msg), _imu_sub, &_imu_msg);
		imuUpdate = true;
	}
	if (fds[1].revents & POLLIN)
	{
		orb_copy(ORB_ID(sensor_baro), _bar_sub, &_bar_msg);
		// Check if Measurement really changed:
		if (abs_float(_bar_msg.pressure - last_barometer_pressure) < 0.0001f)
			baroUpdate = false;
		else
			baroUpdate = true;
	}
	if (fds[2].revents & POLLIN)
	{
		orb_copy(ORB_ID(sonar_msg), _sonar_sub, &_sonar_msg);
		if (abs_float(_sonar_msg.sonar_down - last_sonar_down) < 0.0001f)
		{
			sonarUpdate = false;
			last_sonar_down = _sonar_msg.sonar_down;
		}
		else if (_sonar_msg.sonar_down >= _faultSonar.get())
		{
			sonarUpdate = false;
			last_sonar_down = 0;
			Q(8,8) = 0;
		}
		else
		{
			sonarUpdate = true;
		}
	}

	// State Estimation is calculating with IMU Frequency
	if (imuUpdate)
	{

		/* Prediction Step */
		// using sensors timestamp so we can account for packet lag
		dt = (_imu_msg.timestamp - _predictTimeStamp) / 1.0e6f;
		predictState(dt);
		//correct immediatelly: (~recursive)
		correctIMU();

		_predictTimeStamp = _imu_msg.timestamp;
		imuUpdate = false;

		if (sonarUpdate)
		{
				float threshold_function = (abs_float((float)(pow((double)q_w,2) - pow((double)q_x,2) - pow((double)q_y,2) + pow((double)q_z,2))*
											_sonar_msg.sonar_down - last_sonar_down))/dt_sonar;

				if  (threshold_function > _thresSonar.get())
				{
					Q(8,8) = 0.0;
					b_T = h_W - (float)(pow((double)q_w,2) - pow((double)q_x,2) - pow((double)q_y,2) + pow((double)q_z,2))*_sonar_msg.sonar_down;
				}
				else
				{
					Q(8,8) = 0.01;
					correctSonar();
				}
				last_sonar_down = (float)(pow((double)q_w,2) - pow((double)q_x,2) - pow((double)q_y,2) + pow((double)q_z,2))*
								 _sonar_msg.sonar_down;
				sonarUpdate = false;
		}

		if (baroUpdate)
		{
			correctBar();
			last_barometer_pressure = _bar_msg.pressure;
			baroUpdate =false;
		}


		//Get Max. Calculation Time on Console: (if imu triggered)
//		uint64_t after_calc = hrt_absolute_time();
//		float comp_time = (float)(after_calc - newTimeStamp)/1.0e6f;
//		if (comp_time > max_comp_time)
//		{
//			printf("EKF dt: %3.6f\n",comp_time);
//			max_comp_time = comp_time;
//		}

	}

	// publication to Topic
	if (newTimeStamp - _pubTimeStamp > 1e6 / 50) { // 50 Hz
		_pubTimeStamp = newTimeStamp;
		updatePublications();
	}

	// output to Console
	if (newTimeStamp - _outTimeStamp > 10e5) { // 1 Hz
		_outTimeStamp = newTimeStamp;

		// Debug States:

//		printf("timestamp: %15.10f\n", newTimeStamp/1.0e6);
//		printf("Roll[deg] %4.3f\n",phi/180*M_PI);
//		printf("Pitch[deg] %4.3f\n",theta/180*M_PI);
//		printf("Yaw[deg] %4.3f\n",psi/180*M_PI);
//		printf("Height[m] %4.3f\n",h_W);
//		printf("Pressue Estimate p_0[mbar] %4.3f\n",p_0);

	}

}

void EKFFunction::updatePublications()
{
	using namespace math;

	_emergency_ekf_msg.timestamp = _pubTimeStamp;
	_emergency_ekf_msg.h_W = h_W;
	_emergency_ekf_msg.u_B = u_B;
	_emergency_ekf_msg.v_B = v_B;
	_emergency_ekf_msg.w_B = w_B;
	_emergency_ekf_msg.q_w = q_w;
	_emergency_ekf_msg.q_x = q_x;
	_emergency_ekf_msg.q_y = q_y;
	_emergency_ekf_msg.q_z = q_z;
	_emergency_ekf_msg.p_0 = p_0;
	_emergency_ekf_msg.phi = phi;
	_emergency_ekf_msg.theta = theta;
	_emergency_ekf_msg.psi = psi;
	_emergency_ekf_msg.h_0 = h_0;
	_emergency_ekf_msg.b_T = b_T;

    orb_publish(ORB_ID(emergency_ekf_msg), _emergency_ekf_pub, &_emergency_ekf_msg);

}

int EKFFunction::predictState(float dt)
{
	using namespace math;

	const double state[9] = {h_W, u_B, v_B, w_B, q_w, q_x, q_y, q_z, p_0};
	const double inputs[6] = {_imu_msg.gyro_x,_imu_msg.gyro_y,_imu_msg.gyro_z,
			_imu_msg.acc_x,_imu_msg.acc_y,_imu_msg.acc_z};

	const double parameters[8] = {0.5, g_0, R_0, T_0, mu_N, dt, h_0, 0};

	// Matrices.. will be changed in the function:
	double A_matrix_temp[81];
	double U_matrix_temp[54];

	composeCPPPrediction(state,inputs, parameters,f_vec,A_matrix_temp,U_matrix_temp);

	// prediction is sane
	for (size_t i = 0; i < 9; i++) {
		float val = f_vec[i];

		if (isnan(val) || isinf(val)) {
			// abort correction and return
			warnx("numerical failure in prediction");
			// reset P matrix to P0
			P = P_0;
			return ret_error;
		}
	}

	// Predict State
	h_W = f_vec[HW];
	u_B = f_vec[UB];
	v_B = f_vec[VB];
	w_B = f_vec[WB];
	q_w = f_vec[QW];
	q_x = f_vec[QX];
	q_y = f_vec[QY];
	q_z = f_vec[QZ];
	p_0 = f_vec[P0];

	// This allocation is done according to matlab generation, not as usual...
	for (int j = 0; j < 9; j++) for (int i = 0; i < 9; i++)
	A(i, j) = A_matrix_temp[i + j * 9];

	for (int j = 0; j < 6; j++) for (int i = 0; i < 9; i++)
    U(i, j) = U_matrix_temp[i + j * 9];

	// Predict State Covariance
	P = A * P * A.transposed() + U * Sigma_u * U.transposed() + Q;

	return ret_ok;
}

int EKFFunction::correctIMU()
{
	using namespace math;

	Vector<9> s_predict;

	// Get State:
	s_predict(HW) = h_W;
	s_predict(UB) = u_B;
	s_predict(VB) = v_B;
	s_predict(WB) = w_B;
	s_predict(QW) = q_w;
	s_predict(QX) = q_x;
	s_predict(QY) = q_y;
	s_predict(QZ) = q_z;
	s_predict(P0) = p_0;

	// accel measurement
	Vector<2> zAccel({_imu_msg.acc_x,_imu_msg.acc_y});

	// accel predicted measurement
	Vector<2> zAccelHat({-mu_N*u_B,-mu_N*v_B});

	// calculate residual
	Vector<2> y(zAccel(0) - zAccelHat(0), zAccel(1) - zAccelHat(1));

	// compute correction
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	Matrix<2,2> S = HDrag * P * HDrag.transposed() + RDrag; // residual covariance
	//Hard coded Inverse
	Matrix<2,2> S_inverse;
	float S_det = S(0,0)*S(1,1)-S(0,1)*S(1,0);
	S_inverse(0,0) = S(1,1)/S_det;
	S_inverse(0,1) = -S(0,1)/S_det;
	S_inverse(1,0) = -S(1,0)/S_det;
	S_inverse(1,1) = S(0,0)/S_det;

	Matrix<9, 2> K = P * HDrag.transposed() * S_inverse;
	Vector<9> sCorrect = s_predict +  K * y;

	// check correciton is sane
	for (size_t i = 0; i < sCorrect.get_size(); i++) {
		float val = sCorrect(i);

		if (isnan(val) || isinf(val)) {
			// abort correction and return
			warnx("numerical failure in drag correction"); // Yeah this somehow happens ...
			// reset P matrix to P0
			P = P_0;
			return ret_error;
		}
	}

	// update state covariance
	P = P - K * HDrag * P;

	//Return State:
	h_W = sCorrect(HW);
	u_B = sCorrect(UB);
	v_B = sCorrect(VB);
	w_B = sCorrect(WB);
	q_w = sCorrect(QW);
	q_x = sCorrect(QX);
	q_y = sCorrect(QY);
	q_z = sCorrect(QZ);
	p_0 = sCorrect(P0);

	// Get Rotation Matrices out:
	float temp_array[4] = {q_w,q_x,q_y,q_z};
	Quaternion q(temp_array);
	R_WB = q.to_dcm();
	q.normalize();

	q_w = q(0);
	q_x = q(1);
	q_y = q(2);
	q_z = q(3);

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

	if (p_0 < 1) // First Time Pressure
	{
		p_0 = _bar_msg.pressure;
		h_0 = h_W;
		warnx("this is the height at first correction: %2.3f",h_W);
	}
	Vector<9> s_predict;

	// Get State:
	s_predict(HW) = h_W;
	s_predict(UB) = u_B;
	s_predict(VB) = v_B;
	s_predict(WB) = w_B;
	s_predict(QW) = q_w;
	s_predict(QX) = q_x;
	s_predict(QY) = q_y;
	s_predict(QZ) = q_z;
	s_predict(P0) = p_0;

	Vector<1> zPress;
	zPress(0) = _bar_msg.pressure;
	// accel predicted measurement
	Vector<1> zPressHat;
	zPressHat(0) = p_0*exp(-g_0/(R_0*T_0)*(h_W-h_0));
	// calculate residual
	Vector<1> y;
	y(0) = zPress(0) - zPressHat(0);

	// Only one dimensional corrections...
	HPress(0,0) = -(g_0*p_0)/(R_0*T_0)*exp(-g_0/(R_0*T_0)*(h_W-h_0));
	HPress(0,8) = exp(-g_0/(R_0*T_0)*(h_W-h_0));

	// compute correction:
	Matrix<1,1> S = HPress * P * HPress.transposed() + RPress; // residual covariance
	Matrix<1,1> S_inverse;
	S_inverse(0,0) = 1/S(0,0);
	Matrix<9, 1> K = P * HPress.transposed() * S_inverse;
	Vector<9> sCorrect = s_predict +  K * y;

	// check correciton is sane
	for (size_t i = 0; i < sCorrect.get_size(); i++) {
		float val = sCorrect(i);

		if (isnan(val) || isinf(val)) {
			// abort correction and return
			warnx("numerical failure in pressure correction");
			// reset P matrix to P0
			P = P_0;
			return ret_error;
		}
	}

	// update state covariance
	P = P - K * HPress * P;

	//Return State:
	h_W = sCorrect(HW);
	u_B = sCorrect(UB);
	v_B = sCorrect(VB);
	w_B = sCorrect(WB);
	q_w = sCorrect(QW);
	q_x = sCorrect(QX);
	q_y = sCorrect(QY);
	q_z = sCorrect(QZ);
	p_0 = sCorrect(P0);

	return ret_ok;
}

int EKFFunction::correctSonar()
{
	using namespace math;

	Vector<9> s_predict;

	// Get State:
	s_predict(HW) = h_W;
	s_predict(UB) = u_B;
	s_predict(VB) = v_B;
	s_predict(WB) = w_B;
	s_predict(QW) = q_w;
	s_predict(QX) = q_x;
	s_predict(QY) = q_y;
	s_predict(QZ) = q_z;
	s_predict(P0) = p_0;

	Vector<1> zSonar;
	zSonar(0) = (pow(q_w,2) - pow(q_x,2) - pow(q_y,2) + pow(q_z,2))*_sonar_msg.sonar_down;
	Vector<1> zSonarHat;
	zSonarHat(0) = h_W - b_T;
	// calculate residual
	Vector<1> y;
	y(0) = zSonar(0) - zSonarHat(0);

	// compute correction
	Matrix<1,1> S;; // residual covariance
	S(0,0) = P(0,0) + RSonar(0,0);
	Matrix<1,1> S_inverse;
	S_inverse(0,0) = 1/S(0,0);
	Matrix<9, 1> K = P * HSonar.transposed() * S_inverse;
	Vector<9> sCorrect = s_predict +  K * y;

	// check correciton is sane
	for (size_t i = 0; i < sCorrect.get_size(); i++) {
		float val = sCorrect(i);

		if (isnan(val) || isinf(val)) {
			// abort correction and return
			warnx("numerical failure in sonar correction");
			// reset P matrix to P0
			P = P_0;
			return ret_error;
		}
	}

	// update state covariance
	P = P - K * HSonar * P;

	//Return State:
	h_W = sCorrect(HW);
	u_B = sCorrect(UB);
	v_B = sCorrect(VB);
	w_B = sCorrect(WB);
	q_w = sCorrect(QW);
	q_x = sCorrect(QX);
	q_y = sCorrect(QY);
	q_z = sCorrect(QZ);
	p_0 = sCorrect(P0);

	return ret_ok;
}


int EKFFunction::setState(float input_state[8])
{
	//Simply define state variables:
	h_W = input_state[0];
	u_B = input_state[1];
	v_B = input_state[2];
	w_B = input_state[3];
	q_w = input_state[4];
	q_x = input_state[5];
	q_y = input_state[6];
	q_z = input_state[7];

	return ret_ok;
}

// Custom abs function
float EKFFunction::abs_float(float input_abs)
{
	if (input_abs < 0)
	    {
		input_abs = -input_abs;
	    }
	  return input_abs;
}


/**
 * @file EKFFunction.cpp
 *
 * Kalman filter navigation code
 */

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

//Physical Parameters: - XXX not as environment variable?
static const float g_0 = 9.806f; // [m/s^2] standard gravitational accel.
static const float R_0 = 287.0f; // [J/kgK] Physical Gas constant of Air
static const float T_0 = 273.15 + 15; // [K] Ambient Temperature ~ Assumed to be constant!
static const float mu_N = 0.4; // [1/s] - Normalized Drag coefficient

// Sonic Memory Parameters:
static float last_height;
static float last_sonar;

// Function Parameters
static const int8_t ret_ok = 0; 		// no error in function
static const int8_t ret_error = -1; 	// error occurred

EKFFunction::EKFFunction(SuperBlock *parent, const char *name) :
	SuperBlock(parent, name),

	// subscriptions - Other ones in cunstructor below
	_param_update(&getSubscriptions(), ORB_ID(parameter_update), 1000), // limit to 1 Hz
	// publications
	_emergency_ekf(&getPublications(), ORB_ID(emergency_ekf_msg)),
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
	b_s(0),
	_uGyro(this, "U_GYRO"),
	_uAccel(this, "U_ACCEL"),
	_rDrag(this, "R_DRAG"),
	_rPress(this, "R_PRESS"),
	_rSonar(this,"R_SONAR"),
	_faultSonar(this, "FAULT_SONAR"),
	_thresSonar(this, "THRES_SONAR")
{

	using namespace math;

	//Subscribers:
	  memset(&_imu_msg, 0, sizeof(_imu_msg));
	  _imu_sub = orb_subscribe(ORB_ID(imu_msg));
	  orb_set_interval(_imu_sub, 1); //1000 Hz
	  memset(&_bar_msg, 0, sizeof(_bar_msg));
	  _bar_sub = orb_subscribe(ORB_ID(sensor_baro));
	  orb_set_interval(_bar_sub, 10); //100 Hz
	  _sonar_sub = orb_subscribe(ORB_ID(sonar_msg));
	  orb_set_interval(_sonar_sub, 10); //100 Hz

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

	// initial state covariance matrix
	float est_cov_array[9] = {0.1,0.05,0.05,0.05,5*pow(10,-5),5*pow(10,-5),5*pow(10,-5),5*pow(10,-5),0.1};
	P_0.from_diagonal(est_cov_array);
	P = P_0;

	// Also Initialize Q here! tuning parameter and not dependent on Noise, overview hard otherwise..
	float q_array[9] = {0.05,0.01,0.01,0.2,pow(10,-6),pow(10,-6),pow(10,-6),pow(10,-6),0.1};
	Q.from_diagonal(q_array);

	 orb_copy(ORB_ID(imu_msg), _imu_sub, &_imu_msg);
	 orb_copy(ORB_ID(sensor_baro), _bar_sub, &_bar_msg);
	 orb_copy(ORB_ID(sonar_msg), _sonar_sub, &_sonar_msg);

	// initial state
	h_W = 0.0f;
	u_B = 0.0f;
	v_B = 0.0f;
	w_B = 0.0f;
	q_w = 1.0f;
	q_x = 0.0f;
	q_y = 0.0f;
	q_z = 0.0f;
	p_0 = 0; // Changed in baro correction
	// Initialize differently for sonar range:
	if (_sonar_msg.sonar_down > _faultSonar.get())
	{
	   h_W = 0;
	   last_sonar = 0;
	   Q(8,8) = 0;
	}
	else
	{
	   h_W = _sonar_msg.sonar_down;
	   last_sonar = _sonar_msg.sonar_down;
	}

	last_height = h_W;

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

//	_faultSonar.get();
//	_thresSonar.get();
}

void EKFFunction::update()
{
	using namespace math;

	float dt;

	// poll defined here... _sensors nice structure
	struct pollfd fds[3];
	fds[0].fd = _imu_sub; //_imu_data.getHandle();
	fds[0].events = POLLIN;
	fds[1].fd = _bar_sub;//_bar_data.getHandle();
    fds[1].events = POLLIN;
    fds[2].fd = _sonar_sub;
    fds[2].events = POLLIN;


	// poll for new data
	int ret = poll(fds, 3, 1000);
	// 1000 timeout ...

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

	if (fds[0].revents & POLLIN)
	{
		orb_copy(ORB_ID(imu_msg), _imu_sub, &_imu_msg);
		// Always only future, ring algorithm could be evaluated further
		if (_imu_msg.timestamp >= _predictTimeStamp)
		{
//			printf("IMU Update \n");
			// prediction step
			// using sensors timestamp so we can account for packet lag
			dt = (_imu_msg.timestamp - _predictTimeStamp) / 1.0e6f;
			predictState(dt);
			//correct immediatelly: (~recursive)
			correctIMU();

			_predictTimeStamp = _imu_msg.timestamp;
		}
	}

// TODO Utilize Baro Time stamp and apply ring scheme?
	if (fds[1].revents & POLLIN)
	{
    	orb_copy(ORB_ID(sensor_baro), _bar_sub, &_bar_msg);
		if (_bar_msg.timestamp >= _predictTimeStamp)
		{
//			printf("Baro Update \n");
//		warnx("baroUpdate \n");
		correctBar();

		_predictTimeStamp = _bar_msg.timestamp;
		}
	}

	if (fds[2].revents & POLLIN)
	{
		orb_copy(ORB_ID(sonar_msg), _sonar_sub, &_sonar_msg);
		if (_sonar_msg.timestamp >= _predictTimeStamp)
		{
			// XXX Debugging Prints
//			printf("****Sonar Update **** \n");
//			printf("Threshold function: %2.6f \n",abs_float(abs_float(h_W - last_height) - abs_float((pow(q_w,2) - pow(q_x,2) - pow(q_y,2) + pow(q_z,2))*(_sonar_msg.sonar_down - last_sonar))));
//			printf("Height Diff: %2.6f \n",abs_float(h_W - last_height));
//			printf("Meas Diff: %2.6f, %2.6f \n",(pow(q_w,2) - pow(q_x,2) - pow(q_y,2) + pow(q_z,2)),abs_float((pow(q_w,2) - pow(q_x,2) - pow(q_y,2) + pow(q_z,2))*(_sonar_msg.sonar_down - last_sonar)));
//			printf("q_w = %2.4f, q_x = %2.4f,q_y = %2.4f,q_z = %2.4f \n",q_w,q_x,q_y,q_z);
//			printf("Sonar Measurement: %2.6f \n", _sonar_msg.sonar_down);
//			printf("Last Sonar Measurement: %2.6f \n", last_sonar);
//			printf("Height %2.2f and Last Height %2.2f \n",h_W,last_height);


		if (_sonar_msg.sonar_down > _faultSonar.get())
		{
			last_sonar = 0;
			Q(8,8) = 0;
		}
		else
		{
			if  (abs_float(abs_float(h_W - last_height) - abs_float((pow(q_w,2) - pow(q_x,2) - pow(q_y,2) + pow(q_z,2))*(_sonar_msg.sonar_down - last_sonar))) > _thresSonar.get())
			{
				Q(8,8) = 0.0;
				b_s = h_W - (pow(q_w,2) - pow(q_x,2) - pow(q_y,2) + pow(q_z,2))*_sonar_msg.sonar_down;
//				printf("b_s = %4.3f\n",b_s);
			}
			else
			{
				Q(8,8) = 0.1;
				correctSonar();
				// Actually state updated so update:
				_predictTimeStamp = _sonar_msg.timestamp;
			}
			last_sonar = _sonar_msg.sonar_down;
		}
		}
	}

	last_height = h_W;

	// publication to Topic
	if (newTimeStamp - _pubTimeStamp > 1e6 / 50) { // 50 Hz
		_pubTimeStamp = newTimeStamp;
		updatePublications();
	}

	// output to Console
	if (newTimeStamp - _outTimeStamp > 10e6) { // 0.1 Hz
		_outTimeStamp = newTimeStamp;
		// Show Timestamps here:
//		printf("dt: %15.10f\n", double(dt));
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

	_emergency_ekf.timestamp = _pubTimeStamp;
	_emergency_ekf.h_W = h_W;
	_emergency_ekf.u_B = u_B;
	_emergency_ekf.v_B = v_B;
	_emergency_ekf.w_B = w_B;
	_emergency_ekf.q_w = q_w;
	_emergency_ekf.q_x = q_x;
	_emergency_ekf.q_y = q_y;
	_emergency_ekf.q_z = q_z;
	_emergency_ekf.p_0 = p_0;
	_emergency_ekf.phi = phi;
	_emergency_ekf.theta = theta;
	_emergency_ekf.psi = psi;
	_emergency_ekf.h_0 = h_0;
	_emergency_ekf.b_s = b_s;

	// Selectively update - interresting :-)
	// selectively update publications,
	// do NOT call superblock do-all method
		_emergency_ekf.update();

}

int EKFFunction::predictState(float dt)
{
	using namespace math;

	double state[9] = {h_W, u_B, v_B, w_B, q_w, q_x, q_y, q_z, p_0};
	double inputs[6] = {_imu_msg.gyro_x,_imu_msg.gyro_y,_imu_msg.gyro_z,
			_imu_msg.acc_x,_imu_msg.acc_y,_imu_msg.acc_z};

//	double inputs[6] = {0,0,0,0,0,0};
	// Last input, ground bias b_s - TODO implement
	double parameters[8] = {0.5, g_0, R_0, T_0, mu_N, dt, h_0, 0};

	// Matrices.. will be changed in the function:
	double A_matrix_temp[81];
	double U_matrix_temp[54];

//	double norm_gyro = sqrt(pow(_imu_msg.gyro_x,2) + pow(_imu_msg.gyro_y,2) + pow(_imu_msg.gyro_z,2));
	// Gyro Integration yields failures...
//	if (norm_gyro < 1.0e-7)
//		return ret_ok;
	// Somehow this should work.. check if only value or adress is forwarded ^^ - Array/Pointer zusammenhang
	composeCPPPrediction(state,inputs, parameters,f_vec,A_matrix_temp,U_matrix_temp);

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

//	for (int i = 0; i < 9; i++)
//		printf("f_vec[] - %3.4f\n",f_vec[i]);

	// XXX Look at this allocation!!!
	for (int j = 0; j < 9; j++) for (int i = 0; i < 9; i++)
	A(i, j) = A_matrix_temp[i + j * 9];

	for (int j = 0; j < 6; j++) for (int i = 0; i < 9; i++)
    U(i, j) = U_matrix_temp[i + j * 6];

	// Predict State Covariance
	P = A * P * A.transposed() + U * Sigma_u * U.transposed() + Q;

	return ret_ok;
}

int EKFFunction::correctIMU()
{
	using namespace math;

	Vector<9> s_predict;

	// Tydisome:
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

	// Tydisome:
	s_predict(HW) = h_W;
	s_predict(UB) = u_B;
	s_predict(VB) = v_B;
	s_predict(WB) = w_B;
	s_predict(QW) = q_w;
	s_predict(QX) = q_x;
	s_predict(QY) = q_y;
	s_predict(QZ) = q_z;
	s_predict(P0) = p_0;

//	s_predict.print();

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

	// compute correction
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	Matrix<1,1> S = HPress * P * HPress.transposed() + RPress; // residual covariance
	Matrix<9, 1> K = P * HPress.transposed() * S.inversed();
	Vector<9> sCorrect = s_predict +  K * y;

	// check correciton is sane
	for (size_t i = 0; i < sCorrect.get_size(); i++) {
		float val = sCorrect(i);

		if (isnan(val) || isinf(val)) {
			// abort correction and return
			// TODO Analyze numerical failure for drag correction! (and make usleep...)
			warnx("numerical failure in pressure correction"); // Yeah this somehow happens ...
			// reset P matrix to P0
			P = P_0;
			return ret_error;
		}
	}

	// update state covariance
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	P = P - K * HPress * P;

	//Return State:
	// TODO could all be done better with some state vector over all...
	h_W = sCorrect(HW);
	u_B = sCorrect(UB);
	v_B = sCorrect(VB);
	w_B = sCorrect(WB);
	q_w = sCorrect(QW);
	q_x = sCorrect(QX);
	q_y = sCorrect(QY);
	q_z = sCorrect(QZ);
	p_0 = sCorrect(P0);

//	warnx("Baro Stuff corrected");
	return ret_ok;
}

int EKFFunction::correctSonar()
{
	using namespace math;

	Vector<9> s_predict;

//	// Tydisome:
	s_predict(HW) = h_W;
	s_predict(UB) = u_B;
	s_predict(VB) = v_B;
	s_predict(WB) = w_B;
	s_predict(QW) = q_w;
	s_predict(QX) = q_x;
	s_predict(QY) = q_y;
	s_predict(QZ) = q_z;
	s_predict(P0) = p_0;

//	s_predict.print();

	Vector<1> zSonar;
	zSonar(0) = (pow(q_w,2) - pow(q_x,2) - pow(q_y,2) + pow(q_z,2))*_sonar_msg.sonar_down;
	// accel predicted measurement
	Vector<1> zSonarHat;
	zSonarHat(0) = h_W - b_s;
	// calculate residual
	Vector<1> y;
	y(0) = zSonar(0) - zSonarHat(0);

	// compute correction
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	Matrix<1,1> S = HSonar * P * HSonar.transposed() + RSonar; // residual covariance
	Matrix<9, 1> K = P * HSonar.transposed() * S.inversed();
	Vector<9> sCorrect = s_predict +  K * y;

	// check correciton is sane
	for (size_t i = 0; i < sCorrect.get_size(); i++) {
		float val = sCorrect(i);

		if (isnan(val) || isinf(val)) {
			// abort correction and return
			// TODO Analyze numerical failure for drag correction! (and make usleep...)
			warnx("numerical failure in sonar correction"); // Yeah this somehow happens ...
			// reset P matrix to P0
			P = P_0;
			return ret_error;
		}
	}

	// update state covariance
	// http://en.wikipedia.org/wiki/Extended_Kalman_filter
	P = P - K * HSonar * P;

//	P.print();
//	K.print();
	//Return State:
	// TODO could all be done better with some state vector over all... maybe...
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

float EKFFunction::abs_float(float input_abs)
{
	if (input_abs < 0)
	    {
		input_abs = -input_abs;
	    }
	  return input_abs;
}

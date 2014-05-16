/****************************************************************************/
#include <systemlib/param/param.h>
#include <math.h>

//Temporary calculations:

// Definition of global variables ... Tuning Parameters in form of variances!

/*PARAM_DEFINE_FLOAT(NAME,0.0f);*/
// Covariances:
PARAM_DEFINE_FLOAT(EKFE_U_GYRO, 0.0000076154f); // [rad/s^2] Gyroscope
PARAM_DEFINE_FLOAT(EKFE_U_ACCEL, 0.0392f); // [m/s^2] Accelerometer
PARAM_DEFINE_FLOAT(EKFE_R_DRAG, 0.03f); // [m/s^2] Drag formula
PARAM_DEFINE_FLOAT(EKFE_R_PRESS, 0.1175f); //[mbar/hPa] Barometer noise + Temperature/Equation variation

// Thresholds:
PARAM_DEFINE_FLOAT(EKFE_FAULT_SONAR, 5.0f); //[m] Sonar noise
PARAM_DEFINE_FLOAT(EKFE_THRES_SONAR, 0.03f); //default: 0.03 -> 3 m /s Terrain Change (/dt)

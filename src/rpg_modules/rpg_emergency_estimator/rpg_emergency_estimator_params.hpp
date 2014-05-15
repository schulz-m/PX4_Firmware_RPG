#ifndef RPG_EMERGENCY_ESTIMATOR_H_
#define RPG_EMERGENCY_ESTIMATOR_H_

//This header file/macro defines the constant parameters and their handles & the main file for the estimation.

#include <math.h>
#include <systemlib/param/param.h>

// additional:
#include <systemlib/systemlib.h>

// I use this additional source file completely different than others ~~

struct rpg_emergency_estimator_params
{ \
// Put comments in rpg_emergency_estimator.c --- declaration
/* PHYSICAL PROPERTIES */
  float g; //[m/s^2] Gravitational acceleration
  float R; //[J/kgK] Physical Gas constant of Air
  float T; //Ambient Temperature in Kelvin ~ Assumed to be constant!
  float mu_N;//[1/s] - Normalized Drag coefficient

 /* THRESHOLDS */
  float terrain_threshold; //default: 0.03 -> 3 m /s Terrain Change (/dt)
  float b_s; //[m] Ground Bias
  float sonar_max; //[m] Maximum Distance measurable by sonar

 /* VARIANCES */
  // PREDICTION
  float p_0_cov; //[mbar/hPa] Bias Pressure

  float gyro_cov; // [rad/s^2] Gyroscope
  float acc_cov; // [m/s^2] Accelerometer

  //CORRECTION
  float drag_cov; // [m/s^2] Drag formula
  float press_cov; //[mbar/hPa] Barometer noise + Temperature/Equation variation
  float son_cov; //[m] Sonar noise

  float h_0; //[m] Initial height that will be set
};

//void runAttitudeController(const float roll_des, const float pitch_des, const float yaw_rate_des, const float attitude[],
//                           const struct rpg_emergency_estimator_params params, float desired_body_rates[]);

int parametersInit(struct rpg_emergency_estimator_params *h);

#endif /* RPG_EMERGENCY_ESTIMATOR_H_ */

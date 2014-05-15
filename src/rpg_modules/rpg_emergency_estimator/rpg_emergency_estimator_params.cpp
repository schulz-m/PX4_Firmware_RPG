#include "rpg_emergency_estimator_params.hpp"

int parametersInit(struct rpg_emergency_estimator_params *h)
{

 /*PHYSICAL PROPERTIES */
  h->g  = 9.81; //[m/s^2] Gravitational acceleration
  h->R = 287; //[J/kgK] Physical Gas constant of Air
  h->T = 273.15 + 15; //Ambient Temperature in Kelvin ~ Assumed to be constant!
  h->mu_N = 0.4; //[1/s] - Normalized Drag coefficient

 /* THRESHOLDS */
  h->terrain_threshold = 0.03; //default: 0.03 -> 3 m /s Terrain Change (/dt)
  h->b_s = 0; //[m] Ground Bias
  h->sonar_max = 5; //[m] Maximum Distance measurable by sonar

 /* VARIANCES */
  // PREDICTION
  h->p_0_cov = 0.1; //[mbar/hPa] Bias Pressure
  h->gyro_cov = pow(0.05/180*M_PI,2); // [rad/s^2] Gyroscope
  h->acc_cov = 400*pow(10,-6)*9.81*sqrt(100); // [m/s^2] Accelerometer

  //CORRECTION
  h->drag_cov = 0.03; // [m/s^2] Drag formula
  h->press_cov = 0.1175; //[mbar/hPa] Barometer noise + Temperature/Equation variation
  h->son_cov = 0.05; //[m] Sonar noise

  return 0;
}

// Like this, I can put additional functions in... Could also be defined not in main file ;-)
void PX4EulerAnglesToRPGQuaternion(float q[], const float roll, const float pitch, const float yaw)
{
  // Conversion to RPG Euler angles
  float R_3_2 = sin(roll);
  float R_3_3 = cos(pitch) * cos(roll);
  float R_3_1 = cos(roll) * sin(pitch);
  float R_2_1 = -(cos(pitch) * sin(yaw) + cos(yaw) * sin(pitch) * sin(roll));
  float R_1_1 = cos(pitch) * cos(yaw) - sin(pitch) * sin(roll) * sin(yaw);

  float roll_rpg = atan2(R_3_2, R_3_3);
  float pitch_rpg = -asin(R_3_1);
  float yaw_rpg = atan2(R_2_1, R_1_1);

  // Conversion from RPG Euler angles to Quaternion
  float r = roll_rpg / 2.0f;
  float p = pitch_rpg / 2.0f;
  float y = yaw_rpg / 2.0f;

  q[0] = cos(r) * cos(p) * cos(y) + sin(r) * sin(p) * sin(y);
  q[1] = sin(r) * cos(p) * cos(y) - cos(r) * sin(p) * sin(y);
  q[2] = cos(r) * sin(p) * cos(y) + sin(r) * cos(p) * sin(y);
  q[3] = cos(r) * cos(p) * sin(y) - sin(r) * sin(p) * cos(y);
}

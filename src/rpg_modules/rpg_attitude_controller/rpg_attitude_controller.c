#include <math.h>
#include <systemlib/param/param.h>

#include "rpg_attitude_controller.h"

PARAM_DEFINE_FLOAT(RPG_ATTCONT_TAU, 0.15f);

void runAttitudeController(const float roll_des, const float pitch_des, const float yaw_rate_des, const float attitude[],
                           const struct rpg_attitude_controller_params params, float desired_body_rates[])
{
  // Compute desired body z-axis
  float e_z_des[3];
  e_z_des[0] = cos(roll_des)*sin(pitch_des);
  e_z_des[1] = -sin(roll_des);
  e_z_des[2] = cos(pitch_des)*cos(roll_des);

  // Compute current body z-axis
  float e_z[3];
  e_z[0] = 2.0f*attitude[0]*attitude[2] + 2.0f*attitude[1]*attitude[3];
  e_z[1] = 2.0f*attitude[2]*attitude[3] - 2.0f*attitude[0]*attitude[1];
  e_z[2] = attitude[0]*attitude[0] - attitude[1]*attitude[1] - attitude[2]*attitude[2] + attitude[3]*attitude[3];

  // Compute angle between e_z_des and e_z
  float alpha = acos(e_z_des[0]*e_z[0] + e_z_des[1]*e_z[1] + e_z_des[2]*e_z[2]);

  // Compute rotation axis of error quaternion
  float k[3];
  k[0] = e_z[1]*e_z_des[2] - e_z[2]*e_z_des[1];
  k[1] = e_z[2]*e_z_des[0] - e_z[0]*e_z_des[2];
  k[2] = e_z[0]*e_z_des[1] - e_z[1]*e_z_des[0];
  float k_norm = sqrt(k[0]*k[0] + k[1]*k[1] + k[2]*k[2]);
  k[0] = k[0]/k_norm;
  k[1] = k[1]/k_norm;
  k[2] = k[2]/k_norm;

  // Compute error quaternion
  float q_e[4];
  q_e[0] = cos(alpha/2.0f);
  q_e[1] = sin(alpha/2.0f) * k[0];
  q_e[2] = sin(alpha/2.0f) * k[1];
  q_e[3] = sin(alpha/2.0f) * k[2];

  float attitude_inv[4];

  if (q_e[0] >= 0)
  {
    desired_body_rates[0] = 2.0f / params.tau * q_e[1];
    desired_body_rates[1] = 2.0f / params.tau * q_e[2];
  }
  else
  {
    desired_body_rates[0] = -2.0f / params.tau * q_e[1];
    desired_body_rates[1] = -2.0f / params.tau * q_e[2];
  }

  desired_body_rates[2] = yaw_rate_des;
}

int parametersInit(struct rpg_attitude_controller_params_handles *h)
{
  h->tau = param_find("RPG_ATTCONT_TAU");

  return 0;
}

int parametersUpdate(const struct rpg_attitude_controller_params_handles *h, struct rpg_attitude_controller_params *p)
{
  param_get(h->tau, &(p->tau));

  return 0;
}

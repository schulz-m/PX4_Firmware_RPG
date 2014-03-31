#include <math.h>
#include <systemlib/param/param.h>

#include "rpg_attitude_controller.h"

PARAM_DEFINE_FLOAT(RPG_ATTCONT_TAU, 0.15f);
PARAM_DEFINE_FLOAT(RPG_ATTCONT_P, 0.3f);

void runAttitudeController(const float attitude_sp[], const float attitude[],
                           const struct rpg_attitude_controller_params params, float desired_body_rates[])
{
  // Compute Desired Body Rates
  float q_e[4];
  float attitude_inv[4];

  quatInverse(attitude, attitude_inv);
  quatMultiplication(attitude_inv, attitude_sp, q_e);

  if (q_e[0] >= 0)
  {
    desired_body_rates[0] = 2.0f / params.tau * q_e[1];
    desired_body_rates[1] = 2.0f / params.tau * q_e[2];
    desired_body_rates[2] = 2.0f / params.tau * q_e[3];
  }
  else
  {
    desired_body_rates[0] = -2.0f / params.tau * q_e[1];
    desired_body_rates[1] = -2.0f / params.tau * q_e[2];
    desired_body_rates[2] = -2.0f / params.tau * q_e[3];
  }

  desired_body_rates[2] = params.p * desired_body_rates[2];
}

void quatInverse(const float quaternion[], float quaternion_inv[])
{
  quaternion_inv[0] = quaternion[0];
  quaternion_inv[1] = -quaternion[1];
  quaternion_inv[2] = -quaternion[2];
  quaternion_inv[3] = -quaternion[3];
}

void quatMultiplication(const float q1[], const float q2[], float q_product[])
{
  q_product[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
  q_product[1] = q1[1] * q2[0] + q1[0] * q2[1] - q1[3] * q2[2] + q1[2] * q2[3];
  q_product[2] = q1[2] * q2[0] + q1[3] * q2[1] + q1[0] * q2[2] - q1[1] * q2[3];
  q_product[3] = q1[3] * q2[0] - q1[2] * q2[1] + q1[1] * q2[2] + q1[0] * q2[3];
}

int parametersInit(struct rpg_attitude_controller_params_handles *h)
{
  h->tau = param_find("RPG_ATTCONT_TAU");
  h->p = param_find("RPG_ATTCONT_P");

  return 0;
}

int parametersUpdate(const struct rpg_attitude_controller_params_handles *h, struct rpg_attitude_controller_params *p)
{
  param_get(h->tau, &(p->tau));
  param_get(h->p, &(p->p));

  return 0;
}

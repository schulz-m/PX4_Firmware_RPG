#include "rpg_rate_controller.h"

PARAM_DEFINE_FLOAT(RPG_RCONT_IXX, 0.001f);
PARAM_DEFINE_FLOAT(RPG_RCONT_IYY, 0.001f);
PARAM_DEFINE_FLOAT(RPG_RCONT_IZZ, 0.002f);

PARAM_DEFINE_FLOAT(RPG_RCONT_TAU_PQ, 0.03f);
PARAM_DEFINE_FLOAT(RPG_RCONT_TAU_R, 0.15f);

void runRateController(const float rates_thrust_sp[], const float rates[],
                         const struct rpg_rate_controller_params params, float torques_and_thrust[])
{
  // Compute desired torques
  torques_and_thrust[0] = params.moment_of_inertia_x / params.tau_pq * (rates_thrust_sp[0] - rates[0])
      + rates[1] * rates[2] * (params.moment_of_inertia_z - params.moment_of_inertia_y);
  torques_and_thrust[1] = params.moment_of_inertia_y / params.tau_pq * (rates_thrust_sp[1] - rates[1])
      + rates[0] * rates[2] * (params.moment_of_inertia_x - params.moment_of_inertia_z);
  torques_and_thrust[2] = params.moment_of_inertia_z / params.tau_r * (rates_thrust_sp[2] - rates[2])
      + rates[0] * rates[1] * (params.moment_of_inertia_y - params.moment_of_inertia_x);

  torques_and_thrust[3] = rates_thrust_sp[3];
}

int parametersInit(struct rpg_rate_controller_params_handles *h)
{
  h->moment_of_inertia_x = param_find("RPG_RCONT_IXX");
  h->moment_of_inertia_y = param_find("RPG_RCONT_IYY");
  h->moment_of_inertia_z = param_find("RPG_RCONT_IZZ");

  h->tau_pq = param_find("RPG_RCONT_TAU_PQ");
  h->tau_r = param_find("RPG_RCONT_TAU_R");

  return 0;
}

int parametersUpdate(const struct rpg_rate_controller_params_handles *h, struct rpg_rate_controller_params *p)
{
  param_get(h->moment_of_inertia_x, &(p->moment_of_inertia_x));
  param_get(h->moment_of_inertia_y, &(p->moment_of_inertia_y));
  param_get(h->moment_of_inertia_z, &(p->moment_of_inertia_z));

  param_get(h->tau_pq, &(p->tau_pq));
  param_get(h->tau_r, &(p->tau_r));

  return 0;
}

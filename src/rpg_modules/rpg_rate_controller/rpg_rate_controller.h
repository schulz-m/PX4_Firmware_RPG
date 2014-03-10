#ifndef RPG_RATE_CONTROLLER_H_
#define RPG_RATE_CONTROLLER_H_

struct rpg_rate_controller_params
{
  float moment_of_inertia_x;
  float moment_of_inertia_y;
  float moment_of_inertia_z;

  float tau_pq;
  float tau_r;
};

struct rpg_rate_controller_params_handles
{
  param_t moment_of_inertia_x;
  param_t moment_of_inertia_y;
  param_t moment_of_inertia_z;

  param_t tau_pq;
  param_t tau_r;
};

void run_rate_controller(const float rates_thrust_sp[], const float rates[],
                         const struct rpg_rate_controller_params params, float torques_and_thrust[]);

int parameters_init(struct rpg_rate_controller_params_handles *h);
int parameters_update(const struct rpg_rate_controller_params_handles *h, struct rpg_rate_controller_params *p);

#endif /* RPG_RATE_CONTROLLER_H_ */

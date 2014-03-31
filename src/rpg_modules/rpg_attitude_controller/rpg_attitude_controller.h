#ifndef RPG_ATTITUDE_CONTROLLER_H_
#define RPG_ATTITUDE_CONTROLLER_H_

struct rpg_attitude_controller_params
{
  float tau;
  float p;
};

struct rpg_attitude_controller_params_handles
{
  param_t tau;
  param_t p;
};

void runAttitudeController(const float attitude_sp[], const float attitude[],
                           const struct rpg_attitude_controller_params params, float desired_body_rates[]);

void quatInverse(const float quaternion[], float quaternion_inv[]);
void quatMultiplication(const float q1[], const float q2[], float q_product[]);

int parametersInit(struct rpg_attitude_controller_params_handles *h);
int parametersUpdate(const struct rpg_attitude_controller_params_handles *h, struct rpg_attitude_controller_params *p);

#endif /* RPG_ATTITUDE_CONTROLLER_H_ */

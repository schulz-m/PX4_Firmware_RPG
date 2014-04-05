#ifndef RPG_ATTITUDE_CONTROLLER_H_
#define RPG_ATTITUDE_CONTROLLER_H_

#include <math.h>
#include <systemlib/param/param.h>

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

void runAttitudeController(const float roll_des, const float pitch_des, const float yaw_rate_des, const float attitude[],
                           const struct rpg_attitude_controller_params params, float desired_body_rates[]);

int parametersInit(struct rpg_attitude_controller_params_handles *h);
int parametersUpdate(const struct rpg_attitude_controller_params_handles *h, struct rpg_attitude_controller_params *p);

#endif /* RPG_ATTITUDE_CONTROLLER_H_ */

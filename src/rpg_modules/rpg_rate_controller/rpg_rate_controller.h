#ifndef RPG_RATE_CONTROLLER_H_
#define RPG_RATE_CONTROLLER_H_

struct rpg_rate_controller_params
{

  float mass;
  float arm_length;

  float moment_of_inertia_x;
  float moment_of_inertia_y;
  float moment_of_inertia_z;

  float rotor_drag_coeff;

  float tau_pq;
  float tau_r;

  float gamma_1;
  float gamma_2;
  float gamma_3;
  float gamma_4;
};

struct rpg_rate_controller_params_handles
{

  param_t mass;
  param_t arm_length;

  param_t moment_of_inertia_x;
  param_t moment_of_inertia_y;
  param_t moment_of_inertia_z;

  param_t rotor_drag_coeff;

  param_t tau_pq;
  param_t tau_r;

  param_t gamma_1;
  param_t gamma_2;
  param_t gamma_3;
  param_t gamma_4;
};

void run_rate_controller(const float rate_sp[], const float rates[], const struct rpg_rate_controller_params params,
                         bool use_x_configuration, uint16_t motor_commands[]);

uint16_t convert_thrust_to_motor_command(float thrust);
float convert_motor_command_to_thrust(uint16_t motor_command);

void compute_single_rotor_thrusts(float* rotor_thrusts, float roll_torque, float pitch_torque, float yaw_torque,
                                  float normalized_thrust, bool use_x_configuration,
                                  const struct rpg_rate_controller_params params);

uint16_t saturate_motor_command(uint16_t value, uint16_t min, uint16_t max);

int parameters_init(struct rpg_rate_controller_params_handles *h);
int parameters_update(const struct rpg_rate_controller_params_handles *h, struct rpg_rate_controller_params *p);

#endif /* RPG_RATE_CONTROLLER_H_ */

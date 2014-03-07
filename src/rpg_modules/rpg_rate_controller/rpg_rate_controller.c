#include <stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <float.h>
#include <math.h>
#include <systemlib/pid/pid.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>

#include "rpg_rate_controller.h"

PARAM_DEFINE_FLOAT(RPG_MASS, 0.45f);
PARAM_DEFINE_FLOAT(RPG_L, 0.178f);

PARAM_DEFINE_FLOAT(RPG_IXX, 0.001f);
PARAM_DEFINE_FLOAT(RPG_IYY, 0.001f);
PARAM_DEFINE_FLOAT(RPG_IZZ, 0.002f);

PARAM_DEFINE_FLOAT(RPG_KAPPA, 0.1f);

PARAM_DEFINE_FLOAT(RPG_TAU_PQ, 0.03f);
PARAM_DEFINE_FLOAT(RPG_TAU_R, 0.05f);

PARAM_DEFINE_FLOAT(RPG_GAMMA1, 1.0f);
PARAM_DEFINE_FLOAT(RPG_GAMMA2, 1.0f);
PARAM_DEFINE_FLOAT(RPG_GAMMA3, 1.0f);
PARAM_DEFINE_FLOAT(RPG_GAMMA4, 1.0f);

const int MAX_MOTOR_CMD = 510; // TODO: Where do we have these values? Just make these to parameters which we can set?
const int MIN_SPINNING_MOTOR_CMD = 10;
const float THRUST_MAPPING_A = 4.4854e-06;
const float THRUST_MAPPING_B = 0.0013;
const float THRUST_MAPPING_C = 0.1088;

void run_rate_controller(const float rate_sp[], const float rates[], const struct rpg_rate_controller_params params,
                         bool use_x_configuration, uint16_t motor_commands[])
{
  float rotor_thrusts[4] = {0.0f};
  float desired_torques[3] = {0.0f};

  // "+-configuration" is considered standard, so parameters (L, moments of inertia) are defined with respect to that
  if (use_x_configuration)
  {
    // This assumes that the IMU is aligned to the "x-configuration" where rotor 1 is spinning clockwise as seen from above
    // 1    2    x
    //  \  /     ^
    //   \/      |__> y
    //   /\
    //  /  \
    // 4    3

    // Convert moments of inertia into x-configuration
    float moment_of_inertia_x = (params.moment_of_inertia_x + params.moment_of_inertia_y) / 2.0f;
    float moment_of_inertia_y = (params.moment_of_inertia_x + params.moment_of_inertia_y) / 2.0f;
    float moment_of_inertia_z = params.moment_of_inertia_z;

    // Compute desired torques
    desired_torques[0] = moment_of_inertia_x / params.tau_pq * (rate_sp[0] - rates[0])
        + rates[1] * rates[2] * (moment_of_inertia_z - moment_of_inertia_y);
    desired_torques[1] = moment_of_inertia_y / params.tau_pq * (rate_sp[1] - rates[1])
        + rates[0] * rates[2] * (moment_of_inertia_x - moment_of_inertia_z);
    desired_torques[2] = moment_of_inertia_z / params.tau_r * (rate_sp[2] - rates[2])
        + rates[0] * rates[1] * (moment_of_inertia_y - moment_of_inertia_x);

    // Compute single rotor thrusts for given torques and normalized thrust
    compute_single_rotor_thrusts(rotor_thrusts, desired_torques[0], desired_torques[1], desired_torques[2], rate_sp[3],
                                 use_x_configuration, params);

  }
  else
  {
    // This assumes that the IMU is aligned to the "+-configuration" where rotor 1 is spinning clockwise as seen from above
    //     1        x
    //     |        ^
    // 4-------2    |__> y
    //     |
    //     3

    // Compute desired torques
    desired_torques[0] = params.moment_of_inertia_x / params.tau_pq * (rate_sp[0] - rates[0])
        + rates[1] * rates[2] * (params.moment_of_inertia_z - params.moment_of_inertia_y);
    desired_torques[1] = params.moment_of_inertia_y / params.tau_pq * (rate_sp[1] - rates[1])
        + rates[0] * rates[2] * (params.moment_of_inertia_x - params.moment_of_inertia_z);
    desired_torques[2] = params.moment_of_inertia_z / params.tau_r * (rate_sp[2] - rates[2])
        + rates[0] * rates[1] * (params.moment_of_inertia_y - params.moment_of_inertia_x);

    // Compute single rotor thrusts for given torques and normalized thrust
    compute_single_rotor_thrusts(rotor_thrusts, desired_torques[0], desired_torques[1], desired_torques[2], rate_sp[3],
                                 use_x_configuration, params);
  }

  // Lower collective thrust if one or more of the rotors is saturated
  float max_nom_rotor_thrust = convert_motor_command_to_thrust(MAX_MOTOR_CMD);
  float collective_thrust_above_saturation = 0.0f;

  if (rotor_thrusts[0] > params.gamma_1 * max_nom_rotor_thrust)
  {
    collective_thrust_above_saturation = fmaxf(4.0f * params.gamma_1 * (rotor_thrusts[0] - max_nom_rotor_thrust),
                                               collective_thrust_above_saturation);
  }
  if (rotor_thrusts[1] > params.gamma_2 * max_nom_rotor_thrust)
  {
    collective_thrust_above_saturation = fmaxf(4.0f * params.gamma_1 * (rotor_thrusts[0] - max_nom_rotor_thrust),
                                               collective_thrust_above_saturation);
  }
  if (rotor_thrusts[2] > params.gamma_3 * max_nom_rotor_thrust)
  {
    collective_thrust_above_saturation = fmaxf(4.0f * params.gamma_1 * (rotor_thrusts[0] - max_nom_rotor_thrust),
                                               collective_thrust_above_saturation);
  }
  if (rotor_thrusts[3] > params.gamma_4 * max_nom_rotor_thrust)
  {
    collective_thrust_above_saturation = fmaxf(4.0f * params.gamma_1 * (rotor_thrusts[0] - max_nom_rotor_thrust),
                                               collective_thrust_above_saturation);
  }

  if (collective_thrust_above_saturation > 0.0f)
  {
    // Recompute rotor thrusts with saturated collective thrust (rate_sp[3] - collective_thrust_above_saturation/params.mass)
    compute_single_rotor_thrusts(rotor_thrusts, desired_torques[0], desired_torques[1], desired_torques[2],
                                 rate_sp[3] - collective_thrust_above_saturation / params.mass, use_x_configuration,
                                 params);
  }

  // Convert forces into motor commands
  uint16_t motor_1_cmd = convert_thrust_to_motor_command(rotor_thrusts[0]);
  uint16_t motor_2_cmd = convert_thrust_to_motor_command(rotor_thrusts[1]);
  uint16_t motor_3_cmd = convert_thrust_to_motor_command(rotor_thrusts[2]);
  uint16_t motor_4_cmd = convert_thrust_to_motor_command(rotor_thrusts[3]);

  // Saturate motor commands to ensure valid range
  if (rate_sp[3] < 2.5f) // this is an acceleration in [m/s^2], so 9.81 would be hover.
  {
    // This is a small collective thrust so we can allow 0 motor commands. This is important since with 0 thrust commands we want to have the motors not spinning!!!
    motor_commands[0] = saturate_motor_command(motor_1_cmd, 0, MAX_MOTOR_CMD);
    motor_commands[1] = saturate_motor_command(motor_2_cmd, 0, MAX_MOTOR_CMD);
    motor_commands[2] = saturate_motor_command(motor_3_cmd, 0, MAX_MOTOR_CMD);
    motor_commands[3] = saturate_motor_command(motor_4_cmd, 0, MAX_MOTOR_CMD);
  }
  else
  {
    // This is a decent collective thrust so we don't allow the rotors to stop entirely
    motor_commands[0] = saturate_motor_command(motor_1_cmd, MIN_SPINNING_MOTOR_CMD, MAX_MOTOR_CMD);
    motor_commands[1] = saturate_motor_command(motor_2_cmd, MIN_SPINNING_MOTOR_CMD, MAX_MOTOR_CMD);
    motor_commands[2] = saturate_motor_command(motor_3_cmd, MIN_SPINNING_MOTOR_CMD, MAX_MOTOR_CMD);
    motor_commands[3] = saturate_motor_command(motor_4_cmd, MIN_SPINNING_MOTOR_CMD, MAX_MOTOR_CMD);
  }
}

void compute_single_rotor_thrusts(float* rotor_thrusts, float roll_torque, float pitch_torque, float yaw_torque,
                                  float normalized_thrust, bool use_x_configuration,
                                  const struct rpg_rate_controller_params params)
{
  float K = params.rotor_drag_coeff;
  float L = params.arm_length;

  if (use_x_configuration)
  {
    // Compute the desired thrust for each rotor for x-configuration
    rotor_thrusts[0] = 1.0f / params.gamma_1
        * ((K * L * params.mass * normalized_thrust - L * yaw_torque + sqrt(2) * K * roll_torque
            + sqrt(2) * K * pitch_torque) / (4 * K * L));
    rotor_thrusts[1] = 1.0f / params.gamma_2
        * ((L * yaw_torque + K * L * params.mass * normalized_thrust - sqrt(2) * K * roll_torque
            + sqrt(2) * K * pitch_torque) / (4 * K * L));
    rotor_thrusts[2] = 1.0f / params.gamma_3
        * (-(sqrt(2)
            * (2 * K * roll_torque + 2 * K * pitch_torque + sqrt(2) * L * yaw_torque
                - sqrt(2) * K * L * params.mass * normalized_thrust)) / (8 * K * L));
    rotor_thrusts[3] = 1.0f / params.gamma_4
        * ((sqrt(2)
            * (2 * K * roll_torque - 2 * K * pitch_torque + sqrt(2) * L * yaw_torque
                + sqrt(2) * K * L * params.mass * normalized_thrust)) / (8 * K * L));
  }
  else
  {
    // Compute the desired thrust for each rotor for +-configuration
    rotor_thrusts[0] = 1.0f / params.gamma_1
        * ((2 * K * pitch_torque - L * yaw_torque + K * L * params.mass * normalized_thrust) / (4 * K * L));
    rotor_thrusts[1] = 1.0f / params.gamma_2
        * ((L * yaw_torque - 2 * K * roll_torque + K * L * params.mass * normalized_thrust) / (4 * K * L));
    rotor_thrusts[2] = 1.0f / params.gamma_3
        * (-(2 * K * pitch_torque + L * yaw_torque - K * L * params.mass * normalized_thrust) / (4 * K * L));
    rotor_thrusts[3] = 1.0f / params.gamma_4
        * ((2 * K * roll_torque + L * yaw_torque + K * L * params.mass * normalized_thrust) / (4 * K * L));
  }
}

uint16_t convert_thrust_to_motor_command(float thrust)
{
  // compute thrust for one rotor with second order polynomial according to
  // force = a*mot_cmd^2 + b*mot_cmd + c
  // take the inverse of this function
  uint16_t motor_command = round(
      (-THRUST_MAPPING_B
          + sqrt(THRUST_MAPPING_B * THRUST_MAPPING_B - 4.0 * THRUST_MAPPING_A * (THRUST_MAPPING_C - thrust)))
          / (2 * THRUST_MAPPING_A));

  return motor_command;
}

float convert_motor_command_to_thrust(uint16_t motor_command)
{
  // compute thrust for one rotor with second order polynomial according to
  // force = a*mot_cmd^2 + b*mot_cmd + c
  float thrust = THRUST_MAPPING_A * motor_command * motor_command + THRUST_MAPPING_B * motor_command + THRUST_MAPPING_C;

  return thrust;
}

uint16_t saturate_motor_command(uint16_t value, uint16_t min, uint16_t max)
{
  if (value < min)
    value = min;
  if (value > max)
    value = max;

  return value;
}

int parameters_init(struct rpg_rate_controller_params_handles *h)
{
  h->mass = param_find("RPG_MASS");
  h->arm_length = param_find("RPG_L");

  h->moment_of_inertia_x = param_find("RPG_IXX");
  h->moment_of_inertia_y = param_find("RPG_IYY");
  h->moment_of_inertia_z = param_find("RPG_IZZ");

  h->rotor_drag_coeff = param_find("RPG_KAPPA");

  h->tau_pq = param_find("RPG_TAU_PQ");
  h->tau_r = param_find("RPG_TAU_R");

  h->gamma_1 = param_find("RPG_GAMMA1");
  h->gamma_2 = param_find("RPG_GAMMA2");
  h->gamma_3 = param_find("RPG_GAMMA3");
  h->gamma_4 = param_find("RPG_GAMMA4");

  return 0;
}

int parameters_update(const struct rpg_rate_controller_params_handles *h, struct rpg_rate_controller_params *p)
{
  param_get(h->mass, &(p->mass));
  param_get(h->arm_length, &(p->arm_length));

  param_get(h->moment_of_inertia_x, &(p->moment_of_inertia_x));
  param_get(h->moment_of_inertia_y, &(p->moment_of_inertia_y));
  param_get(h->moment_of_inertia_z, &(p->moment_of_inertia_z));

  param_get(h->rotor_drag_coeff, &(p->rotor_drag_coeff));

  param_get(h->tau_pq, &(p->tau_pq));
  param_get(h->tau_r, &(p->tau_r));

  param_get(h->gamma_1, &(p->gamma_1));
  param_get(h->gamma_2, &(p->gamma_2));
  param_get(h->gamma_3, &(p->gamma_3));
  param_get(h->gamma_4, &(p->gamma_4));

  return 0;
}

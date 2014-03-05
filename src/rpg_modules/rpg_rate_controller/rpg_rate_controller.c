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

PARAM_DEFINE_FLOAT(RPG_KAPPA, 1.0f);

PARAM_DEFINE_FLOAT(RPG_TAU_PQ, 0.1f);
PARAM_DEFINE_FLOAT(RPG_TAU_R, 0.005f);

PARAM_DEFINE_FLOAT(RPG_GAMMA1, 1.0f);
PARAM_DEFINE_FLOAT(RPG_GAMMA2, 1.0f);
PARAM_DEFINE_FLOAT(RPG_GAMMA3, 1.0f);
PARAM_DEFINE_FLOAT(RPG_GAMMA4, 1.0f);

void run_rate_controller(const float rate_sp[], const float rates[], const struct rpg_rate_controller_params params,
bool use_x_configuration,
                         uint16_t motor_commands[])
{
  float K = params.rotor_drag_coeff;
  float L = params.arm_length;

  float rotor_thrusts[4] = {0.0f};

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
    float roll_torque = moment_of_inertia_x / params.tau_pq * (rate_sp[0] - rates[0])
        + rates[1] * rates[2] * (moment_of_inertia_z - moment_of_inertia_y);
    float pitch_torque = moment_of_inertia_y / params.tau_pq * (rate_sp[1] - rates[1])
        + rates[0] * rates[2] * (moment_of_inertia_x - moment_of_inertia_z);
    float yaw_torque = moment_of_inertia_z / params.tau_r * (rate_sp[2] - rates[2])
        + rates[0] * rates[1] * (moment_of_inertia_y - moment_of_inertia_x);

    // Compute the desired thrust for each rotor
    rotor_thrusts[0] = 1.0f / params.gamma_1
        * ((K * L * params.mass * rate_sp[3] - L * yaw_torque + sqrt(2) * K * roll_torque + sqrt(2) * K * pitch_torque)
            / (4 * K * L));
    rotor_thrusts[1] = 1.0f / params.gamma_2
        * ((L * yaw_torque + K * L * params.mass * rate_sp[3] - sqrt(2) * K * roll_torque + sqrt(2) * K * pitch_torque)
            / (4 * K * L));
    rotor_thrusts[2] = 1.0f / params.gamma_3
        * (-(sqrt(2)
            * (2 * K * roll_torque + 2 * K * pitch_torque + sqrt(2) * L * yaw_torque
                - sqrt(2) * K * L * params.mass * rate_sp[3])) / (8 * K * L));
    rotor_thrusts[3] = 1.0f / params.gamma_4
        * ((sqrt(2)
            * (2 * K * roll_torque - 2 * K * pitch_torque + sqrt(2) * L * yaw_torque
                + sqrt(2) * K * L * params.mass * rate_sp[3])) / (8 * K * L));
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
    float roll_torque = params.moment_of_inertia_x / params.tau_pq * (rate_sp[0] - rates[0])
        + rates[1] * rates[2] * (params.moment_of_inertia_z - params.moment_of_inertia_y);
    float pitch_torque = params.moment_of_inertia_y / params.tau_pq * (rate_sp[1] - rates[1])
        + rates[0] * rates[2] * (params.moment_of_inertia_x - params.moment_of_inertia_z);
    float yaw_torque = params.moment_of_inertia_z / params.tau_r * (rate_sp[2] - rates[2])
        + rates[0] * rates[1] * (params.moment_of_inertia_y - params.moment_of_inertia_x);

    // Compute the desired thrust for each rotor
    rotor_thrusts[0] = 1.0f / params.gamma_1
        * ((2 * K * pitch_torque - L * yaw_torque + K * L * params.mass * rate_sp[3]) / (4 * K * L));
    rotor_thrusts[1] = 1.0f / params.gamma_2
        * ((L * yaw_torque - 2 * K * roll_torque + K * L * params.mass * rate_sp[3]) / (4 * K * L));
    rotor_thrusts[2] = 1.0f / params.gamma_3
        * (-(2 * K * pitch_torque + L * yaw_torque - K * L * params.mass * rate_sp[3]) / (4 * K * L));
    rotor_thrusts[3] = 1.0f / params.gamma_4
        * ((2 * K * roll_torque + L * yaw_torque + K * L * params.mass * rate_sp[3]) / (4 * K * L));
  }

  // Convert forces into motor commands
  uint16_t motor_1_cmd = convert_thrust_to_motor_command(rotor_thrusts[0]);
  uint16_t motor_2_cmd = convert_thrust_to_motor_command(rotor_thrusts[1]);
  uint16_t motor_3_cmd = convert_thrust_to_motor_command(rotor_thrusts[2]);
  uint16_t motor_4_cmd = convert_thrust_to_motor_command(rotor_thrusts[3]);

  // Saturate motor commands to ensure valid range
  // TODO: Does it make sense to scale all motors if one is saturated? If so then do it before the conversion of thrusts to motor commands
  motor_1_cmd = saturate_motor_command(motor_1_cmd, 0, 510);
  motor_2_cmd = saturate_motor_command(motor_2_cmd, 0, 510);
  motor_3_cmd = saturate_motor_command(motor_3_cmd, 0, 510);
  motor_4_cmd = saturate_motor_command(motor_4_cmd, 0, 510);

  motor_commands[0] = motor_1_cmd;
  motor_commands[1] = motor_2_cmd;
  motor_commands[2] = motor_3_cmd;
  motor_commands[3] = motor_4_cmd;
}

uint16_t convert_thrust_to_motor_command(float thrust)
{
  // Convert forces into motor commands
  float a = 4.4854e-06;
  float b = 0.0013;
  float c = 0.1088;

  // compute thrust for one rotor with second order polynomial according to
  // force = a*mot_cmd^2 + b*mot_cmd + c
  // take the inverse of this function
  uint16_t motor_command = round((-b + sqrt(b * b - 4.0 * a * (c - thrust))) / (2 * a));

  return motor_command;
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

  h->gyro_bias_x = param_find("SENS_GYRO_XOFF");
  h->gyro_bias_y = param_find("SENS_GYRO_YOFF");
  h->gyro_bias_z = param_find("SENS_GYRO_ZOFF");

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

  param_get(h->gyro_bias_x, &(p->gyro_bias_x));
  param_get(h->gyro_bias_y, &(p->gyro_bias_y));
  param_get(h->gyro_bias_z, &(p->gyro_bias_z));

  return 0;
}

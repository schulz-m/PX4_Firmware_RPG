#include "rpg_rate_controller.h"
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

PARAM_DEFINE_FLOAT(RPG_MASS, 0.45f);
PARAM_DEFINE_FLOAT(RPG_L, 0.2f);

PARAM_DEFINE_FLOAT(RPG_IXX, 0.001f);
PARAM_DEFINE_FLOAT(RPG_IYY, 0.001f);
PARAM_DEFINE_FLOAT(RPG_IZZ, 0.002f);

PARAM_DEFINE_FLOAT(RPG_KAPPA, 1.0f);

PARAM_DEFINE_FLOAT(RPG_TAU_PQ, 0.2f);
PARAM_DEFINE_FLOAT(RPG_TAU_R, 0.2f);

PARAM_DEFINE_FLOAT(RPG_GAMMA1, 1.0f);
PARAM_DEFINE_FLOAT(RPG_GAMMA2, 1.0f);
PARAM_DEFINE_FLOAT(RPG_GAMMA3, 1.0f);
PARAM_DEFINE_FLOAT(RPG_GAMMA4, 1.0f);

static int parameters_init(struct rpg_rate_controller_params_handles *h)
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

  return OK;
}

static int parameters_update(const struct rpg_rate_controller_params_handles *h, struct rpg_rate_controller_params *p)
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

  return OK;
}

void run_rate_controller(const struct vehicle_rates_setpoint_s *rate_sp, const float rates[], const struct rpg_rate_controller_params params, struct actuator_controls_s *actuators)
{
  // TODO: consider x or + configuration

  // Compute torques to be commanded
  float roll_torque = 1/params.tau_pq * (rate_sp[0] - (rates[0] - gyro_bias_x))
                         + rates[1]*rates[2]*(params.moment_of_inertia_z - params.moment_of_inertia_y);
  float pitch_torque = 1/params.tau_pq * (rate_sp[1] - (rates[1] - gyro_bias_y))
                          + rates[0]*rates[2]*(params.moment_of_inertia_x - params.moment_of_inertia_z);
  float yaw_torque = 1/params.tau_r * (rate_sp[2] - (rates[2] - gyro_bias_z))
                        + rates[0]*rates[1]*(params.moment_of_inertia_y - params.moment_of_inertia_x);

  actuators->control[0] = roll_torque;
  actuators->control[1] = pitch_torque;
  actuators->control[2] = yaw_torque;
  actuators->control[3] = rate_sp->thrust;

  /*
  // Compute the desired forces for each rotor
  float f1 = 1/params.gamma_1*((params.arm_length*yaw_torque - 2*params.rotor_drag_coeff*pitch_torque + params.rotor_drag_coeff*params.arm_length*params.mass*rate_sp->thrust)/(4*params.rotor_drag_coeff*params.arm_length));
  float f2 = 1/params.gamma_2*((2*params.rotor_drag_coeff*roll_torque - params.arm_length*yaw_torque + params.rotor_drag_coeff*params.arm_length*params.mass*rate_sp->thrust)/(4*params.rotor_drag_coeff*params.arm_length));
  float f3 = 1/params.gamma_3*((2*params.rotor_drag_coeff*pitch_torque + params.arm_length*yaw_torque + params.rotor_drag_coeff*params.arm_length*params.mass*rate_sp->thrust)/(4*params.rotor_drag_coeff*params.arm_length));
  float f4 = 1/params.gamma_4*(-(2*params.rotor_drag_coeff*roll_torque + params.arm_length*yaw_torque - params.rotor_drag_coeff*params.arm_length*params.mass*rate_sp->thrust)/(4*params.rotor_drag_coeff*params.arm_length));

  // Convert forces into single motor commands
  float a = 4.4854e-06;
  float b = 0.0013;
  float c = 0.1088;

  // compute force for one rotor with second order polynomial according to
  // force = a*mot_cmd^2 + b*mot_cmd + c
  // take the inverse of this function
  uint16_t motor_1_cmd = round((-b + sqrt(b*b - 4.0*a*(c - f1)))/(2*a));
  uint16_t motor_2_cmd = round((-b + sqrt(b*b - 4.0*a*(c - f2)))/(2*a));
  uint16_t motor_3_cmd = round((-b + sqrt(b*b - 4.0*a*(c - f3)))/(2*a));
  uint16_t motor_4_cmd = round((-b + sqrt(b*b - 4.0*a*(c - f4)))/(2*a));

  // Ensure lower limit of commands
  motor_1_cmd = (motor_1_cmd > 0) ? motor_1_cmd : 0;
  motor_2_cmd = (motor_2_cmd > 0) ? motor_2_cmd : 0;
  motor_3_cmd = (motor_3_cmd > 0) ? motor_3_cmd : 0;
  motor_4_cmd = (motor_4_cmd > 0) ? motor_4_cmd : 0;

  // Ensure upper limit of commands
  motor_1_cmd = (motor_1_cmd <= 510) ? motor_1_cmd : 510;
  motor_2_cmd = (motor_2_cmd <= 510) ? motor_2_cmd : 510;
  motor_3_cmd = (motor_3_cmd <= 510) ? motor_3_cmd : 510;
  motor_4_cmd = (motor_4_cmd <= 510) ? motor_4_cmd : 510;

  // send motor commands via UART
  ardrone_write_motor_commands(ardrone_write, motor_1_cmd, motor_2_cmd, motor_3_cmd, motor_4_cmd);

  */
}

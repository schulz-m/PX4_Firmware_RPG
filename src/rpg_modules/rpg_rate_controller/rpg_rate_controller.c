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

void run_rate_controller(const float rate_sp[], const float rates[], const struct rpg_rate_controller_params params, uint16_t motor_commands[])
{
  // TODO: consider x or + configuration

  // Compute torques to be commanded
  float roll_torque = 1/params.tau_pq * (rate_sp[0] - (rates[0] - params.gyro_bias_x))
                         + rates[1]*rates[2]*(params.moment_of_inertia_z - params.moment_of_inertia_y);
  float pitch_torque = 1/params.tau_pq * (rate_sp[1] - (rates[1] - params.gyro_bias_y))
                          + rates[0]*rates[2]*(params.moment_of_inertia_x - params.moment_of_inertia_z);
  float yaw_torque = 1/params.tau_r * (rate_sp[2] - (rates[2] - params.gyro_bias_z))
                        + rates[0]*rates[1]*(params.moment_of_inertia_y - params.moment_of_inertia_x);

  // Compute the desired forces for each rotor
  float f1 = 1/params.gamma_1*((params.arm_length*yaw_torque - 2*params.rotor_drag_coeff*pitch_torque + params.rotor_drag_coeff*params.arm_length*params.mass*rate_sp[3])/(4*params.rotor_drag_coeff*params.arm_length));
  float f2 = 1/params.gamma_2*((2*params.rotor_drag_coeff*roll_torque - params.arm_length*yaw_torque + params.rotor_drag_coeff*params.arm_length*params.mass*rate_sp[3])/(4*params.rotor_drag_coeff*params.arm_length));
  float f3 = 1/params.gamma_3*((2*params.rotor_drag_coeff*pitch_torque + params.arm_length*yaw_torque + params.rotor_drag_coeff*params.arm_length*params.mass*rate_sp[3])/(4*params.rotor_drag_coeff*params.arm_length));
  float f4 = 1/params.gamma_4*(-(2*params.rotor_drag_coeff*roll_torque + params.arm_length*yaw_torque - params.rotor_drag_coeff*params.arm_length*params.mass*rate_sp[3])/(4*params.rotor_drag_coeff*params.arm_length));

  // Convert forces into motor commands
  uint16_t motor_1_cmd = convert_force_to_motor_command(f1);
  uint16_t motor_2_cmd = convert_force_to_motor_command(f2);
  uint16_t motor_3_cmd = convert_force_to_motor_command(f3);
  uint16_t motor_4_cmd = convert_force_to_motor_command(f4);

  // Saturate motor commands to ensure valid range
  // TODO: Does it makes sense to scale all motors if one is saturated?
  motor_1_cmd = saturate_motor_command(motor_1_cmd, 0, 510);
  motor_2_cmd = saturate_motor_command(motor_2_cmd, 0, 510);
  motor_3_cmd = saturate_motor_command(motor_3_cmd, 0, 510);
  motor_4_cmd = saturate_motor_command(motor_4_cmd, 0, 510);

  motor_commands[0] = motor_1_cmd;
  motor_commands[1] = motor_2_cmd;
  motor_commands[2] = motor_3_cmd;
  motor_commands[3] = motor_4_cmd;
}

uint16_t convert_force_to_motor_command(float force)
{
  // Convert forces into motor commands
  float a = 4.4854e-06;
  float b = 0.0013;
  float c = 0.1088;

  // compute force for one rotor with second order polynomial according to
  // force = a*mot_cmd^2 + b*mot_cmd + c
  // take the inverse of this function
  uint16_t motor_command = round((-b + sqrt(b*b - 4.0*a*(c - force)))/(2*a));

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

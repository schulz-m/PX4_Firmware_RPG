#ifndef EMERGENCY_EKF_H
#define EMERGENCY_EKF_H

#include <stdint.h>
#include "../../uORB.h"

struct emergency_ekf_msg_s {
        uint64_t timestamp;
        float h_W; /**< height in world frame */
        float u_B, v_B, w_B; /**< navigation velocity in body frame, m/s */
        float q_w, q_x, q_y, q_z; /**< quaternion, [-] */
        double p_0; /**< reference pressure */

        // parameters to publish and helper:
        float phi, theta, psi;
        float h_0; /**< refeerence altitude (ground height) */
        float b_s;      /**< ground bias for sonar estimation */

};

ORB_DECLARE(emergency_ekf_msg);

#endif

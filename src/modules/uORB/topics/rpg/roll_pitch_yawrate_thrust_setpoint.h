#ifndef TOPIC_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_H
#define TOPIC_ROLL_PITCH_YAWRATE_THRUST_SETPOINT_H

#include <stdint.h>
#include "../../uORB.h"

struct roll_pitch_yawrate_thrust_setpoint_s {
        uint64_t timestamp;
        float roll;
        float pitch;
        float yaw_rate;
        float normalized_thrust;
};

ORB_DECLARE(roll_pitch_yawrate_thrust_setpoint);

#endif

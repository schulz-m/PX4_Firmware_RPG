#ifndef TOPIC_TORQUES_AND_THRUST_H
#define TOPIC_TORQUES_AND_THRUST_H

#include <stdint.h>
#include "../../uORB.h"

struct torques_and_thrust_s {
        uint64_t timestamp;
        float roll_torque;
        float pitch_torque;
        float yaw_torque;
        float normalized_thrust;
};

ORB_DECLARE(torques_and_thrust);

#endif

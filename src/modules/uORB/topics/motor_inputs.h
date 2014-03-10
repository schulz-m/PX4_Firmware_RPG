#ifndef TOPIC_MOTOR_INPUTS_H
#define TOPIC_MOTOR_INPUTS_H

#include <stdint.h>
#include "../uORB.h"

struct motor_inputs_s {
        uint64_t timestamp;
        uint16_t motor_inputs[];
};

ORB_DECLARE(motor_inputs);

#endif

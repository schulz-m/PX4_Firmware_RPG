#ifndef TOPIC_MOTOR_INPUTS_H
#define TOPIC_MOTOR_INPUTS_H

#include <stdint.h>
#include "../uORB.h"

#define NUM_MOTORS           4

struct motor_inputs_s {
        uint64_t timestamp;
        uint16_t motor_inputs[NUM_MOTORS];
};

ORB_DECLARE(motor_inputs);

#endif

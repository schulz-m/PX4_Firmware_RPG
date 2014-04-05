#ifndef TOPIC_THRUST_INPUTS_H
#define TOPIC_THRUST_INPUTS_H

#include <stdint.h>
#include "../../uORB.h"

#define NUM_ROTORS           4

struct thrust_inputs_s {
        uint64_t timestamp;
        float thrust_inputs[NUM_ROTORS]; // [N]
};

ORB_DECLARE(thrust_inputs);

#endif

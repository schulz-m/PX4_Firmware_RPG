#ifndef TOPIC_THRUST_INPUTS_H
#define TOPIC_THRUST_INPUTS_H

#include <stdint.h>
#include "../uORB.h"

#define NUM_ROTORS           4

struct thrust_inputs_s {
        uint64_t timestamp;
        uint16_t thrust_inputs[NUM_ROTORS];
};

ORB_DECLARE(thrust_inputs);

#endif

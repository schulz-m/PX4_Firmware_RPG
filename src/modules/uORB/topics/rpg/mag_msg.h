#ifndef MAG_MSG_H
#define MAG_MSG_H

#include <stdint.h>
#include "../../uORB.h"

struct mag_msg_s {
        uint64_t timestamp;
        float x;
        float y;
        float z;
};

ORB_DECLARE(mag_msg);

#endif

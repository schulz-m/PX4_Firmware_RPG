#ifndef SONAR_MSG_H
#define SONAR_MSG_H

#include <stdint.h>
#include "../../uORB.h"

struct sonar_msg_s {
        uint64_t timestamp;
        float sonar_down;
};

ORB_DECLARE(sonar_msg);

#endif

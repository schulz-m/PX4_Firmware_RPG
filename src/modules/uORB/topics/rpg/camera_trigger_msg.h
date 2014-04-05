#ifndef CAMERA_TRIGGE_MSG_H
#define CAMERA_TRIGGE_MSG_H

#include <stdint.h>
#include <string.h>
#include "../../uORB.h"

struct camera_trigger_msg_s {
        uint64_t timestamp;
        char* camera_name;
        unsigned long frame_number;
};

ORB_DECLARE(camera_trigger_msg);

#endif

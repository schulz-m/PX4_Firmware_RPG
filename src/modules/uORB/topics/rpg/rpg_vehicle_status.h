#ifndef RPG_VEHICLE_STATUS_H
#define RPG_VEHICLE_STATUS_H

#include <stdint.h>
#include "../../uORB.h"

struct rpg_vehicle_status_s {
        uint64_t timestamp;
        int commander_state;
        int battery_state;
};

ORB_DECLARE(rpg_vehicle_status);

#endif

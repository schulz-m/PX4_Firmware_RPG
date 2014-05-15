#ifndef IMU_MSG_H
#define IMU_MSG_H

#include <stdint.h>
#include "../../uORB.h"

struct imu_msg_s {
        uint64_t timestamp;
        float acc_x;
        float acc_y;
        float acc_z;

        float gyro_x;
        float gyro_y;
        float gyro_z;
};

ORB_DECLARE(imu_msg);

#endif

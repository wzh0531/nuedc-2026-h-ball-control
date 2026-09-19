#ifndef _HOST_STUB_ZF_DEVICE_IMU660RA_H_
#define _HOST_STUB_ZF_DEVICE_IMU660RA_H_

#include "zf_common_typedef.h"

extern int16 imu660ra_gyro_z;
extern float imu660ra_transition_factor[2];

#define imu660ra_gyro_transition(gyro_value) \
    ((float)(gyro_value) / imu660ra_transition_factor[1])

uint8 imu660ra_init(void);
void imu660ra_get_gyro(void);

#endif

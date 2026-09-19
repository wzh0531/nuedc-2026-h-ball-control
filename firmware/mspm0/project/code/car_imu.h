#ifndef _CAR_IMU_H_
#define _CAR_IMU_H_

/* L3 Driver adapter - 逐飞 IMU660RA 软件 I2C 驱动适配。 */
#include "zf_common_typedef.h"

uint8 car_imu_init(void);
/*
 * 上电标定由底盘 CALIBRATE 状态每 5 ms 推进一步，避免约 0.9 s 的阻塞等待。
 * calibration_step 返回 1 表示本轮标定已经结束，结果再由 is_ready 判断。
 */
uint8 car_imu_is_calibrating(void);
uint8 car_imu_calibration_step(void);
uint8 car_imu_calibrate(void);
void car_imu_update(float dt_s);
uint8 car_imu_is_ready(void);
float car_imu_get_rate(void);
float car_imu_get_bias(void);
float car_imu_get_residual(void);

#endif

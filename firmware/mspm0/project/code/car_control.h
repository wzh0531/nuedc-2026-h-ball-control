#ifndef _CAR_CONTROL_H_
#define _CAR_CONTROL_H_

/* L5 Service - 灰度外环、IMU 角速度内环和左右轮速度 PI。 */
#include "zf_common_typedef.h"

typedef struct
{
    float left_speed_mps;
    float right_speed_mps;
    float left_target_mps;
    float right_target_mps;
    float yaw_rate_target_dps;
    float yaw_rate_feedback_dps;
    float line_error;
    float speed_delta_mps;
    int16 left_output;
    int16 right_output;
}car_control_state_struct;

void car_control_init(void);
void car_control_reset(void);
void car_control_update(float base_speed_mps, float line_error,
    uint8 line_valid, float dt_s);
void car_control_update_yaw_target(float base_speed_mps,
    float yaw_target_dps, float dt_s);
void car_control_update_wheel_targets(float left_target_mps,
    float right_target_mps, float dt_s);
const car_control_state_struct *car_control_get_state(void);
void car_control_set_speed_pid(float kp, float ki, float kd);
void car_control_set_line_pd(float kp, float kd);
void car_control_set_yaw_pid(float kp, float ki, float kd);

#endif

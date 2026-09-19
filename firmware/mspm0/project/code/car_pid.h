#ifndef _CAR_PID_H_
#define _CAR_PID_H_

/* L4 Middleware - 带条件积分抗饱和的通用 PID。 */
#include "zf_common_typedef.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float integral_limit;
    float output_limit;
    uint8 initialized;
}car_pid_struct;

void car_pid_init(car_pid_struct *pid, float kp, float ki, float kd,
    float integral_limit, float output_limit);
void car_pid_set_gains(car_pid_struct *pid, float kp, float ki, float kd);
void car_pid_reset(car_pid_struct *pid);
float car_pid_update(car_pid_struct *pid, float target, float measurement,
    float dt_s, float feedforward);

#endif

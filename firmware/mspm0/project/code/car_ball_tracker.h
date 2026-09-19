#ifndef _CAR_BALL_TRACKER_H_
#define _CAR_BALL_TRACKER_H_

/* L5 Service - MaixCAM 小球偏差解析与无刷云台位置闭环。 */
#include "zf_common_typedef.h"

typedef enum
{
    CAR_BALL_TASK3_IDLE = 0,
    CAR_BALL_TASK3_TO_POSITIVE,
    CAR_BALL_TASK3_TO_NEGATIVE,
    CAR_BALL_TASK3_COMPLETE,
}car_ball_task3_phase_enum;

void car_ball_tracker_init(void);
void car_ball_tracker_update_5ms(uint8 task2_hold_zero,
    float vehicle_accel_ref_mps2);
void car_ball_tracker_parse_vision_byte(uint8 data);
void car_ball_tracker_set_pid(float kp, float ki, float kd);
void car_ball_tracker_set_task3_pid(float kp, float ki, float kd);
void car_ball_tracker_set_task3_left_pid(float kp, float ki, float kd);
void car_ball_tracker_set_task3_right_pid(float kp, float ki, float kd);
void car_ball_tracker_set_target(float target_pixels);
void car_ball_tracker_task3_start(void);
void car_ball_tracker_task3_cancel(void);
uint8 car_ball_tracker_task3_take_complete(void);

uint8 car_ball_tracker_is_tracking(void);
int16 car_ball_tracker_get_dx(void);
float car_ball_tracker_get_target(void);
int32 car_ball_tracker_get_command_angle_x10(void);
uint32 car_ball_tracker_get_frame_count(void);
car_ball_task3_phase_enum car_ball_tracker_get_task3_phase(void);

#endif

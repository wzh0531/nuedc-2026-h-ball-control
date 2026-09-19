#ifndef _CAR_CHASSIS_H_
#define _CAR_CHASSIS_H_

/* L5 Service - H 题底盘状态机与固定对外接口。 */
#include "zf_common_typedef.h"

typedef enum
{
    CHASSIS_MODE_INIT = 0,
    CHASSIS_MODE_CALIBRATE,
    CHASSIS_MODE_READY,
    CHASSIS_MODE_RUNNING,
    CHASSIS_MODE_PRE_BRAKE,
    CHASSIS_MODE_FINISHED,
    CHASSIS_MODE_FAULT,
}ChassisMode;

typedef enum
{
    CHASSIS_TASK_2_LAP_STOP_A = 2,
    CHASSIS_TASK_3_STATIONARY = 3,
    CHASSIS_TASK_4_A_TO_B = 4,
    CHASSIS_TASK_5_LAP_CENTER = 5,
    CHASSIS_TASK_6_LAP_POSITION = 6,
}ChassisTask;

typedef struct
{
    float left_speed_mps;
    float right_speed_mps;
    float body_speed_mps;
    float distance_m;
    uint8 gray_mask;
    uint8 gray_black_count;
    uint8 line_valid;
    float line_error;
    float yaw_rate_dps;
    float base_speed_ref_mps;
    float vehicle_accel_ref_mps2;
    float left_speed_ref_mps;
    float right_speed_ref_mps;
    int16 left_pwm;
    int16 right_pwm;
    uint32 run_time_ms;
    /*
     * 赛题有效时间：任务 2/3 为完成时刻，任务 4 为测试点通过 B 的
     * 时刻，任务 5/6 为测试点通过 A 的时刻。字段名保留以兼容既有接口。
     */
    uint32 lap_time_ms;
    uint32 fault_bits;
    ChassisMode mode;
    ChassisTask task;
}ChassisState;

void chassis_init(void);
uint8 chassis_start(void);
void chassis_stop(void);
void chassis_update_5ms(void);
void chassis_set_base_speed(float speed_mps);
const ChassisState *chassis_get_state(void);
void chassis_emergency_stop(uint32 fault_bits);
uint8 chassis_reset(void);
uint8 chassis_select_task(ChassisTask task);
uint8 chassis_start_motor_test(int16 left, int16 right, uint32 duration_ms);
uint8 chassis_start_velocity_test(float left_mps, float right_mps,
    uint32 duration_ms);
uint8 chassis_start_yaw_test(float base_speed_mps, float yaw_target_dps,
    uint32 duration_ms);

#endif

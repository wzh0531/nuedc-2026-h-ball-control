#ifndef _CAR_TELEMETRY_H_
#define _CAR_TELEMETRY_H_

/*
 * L4 Middleware - 非阻塞运行统计。
 *
 * 控制周期内只做常数时间的 RAM 累计，不使用串口。停车后由 STATS 命令读取，
 * 用于速度环、循迹和左右轮一致性调参。
 */
#include "zf_common_typedef.h"

typedef struct
{
    uint32 sample_count;
    float left_speed_error_mean_abs_mps;
    float right_speed_error_mean_abs_mps;
    float left_speed_error_max_abs_mps;
    float right_speed_error_max_abs_mps;
    float wheel_speed_difference_mean_abs_mps;
    float wheel_speed_difference_max_abs_mps;
    float line_error_max_abs;
    float yaw_rate_error_mean_abs_dps;
    float yaw_rate_error_max_abs_dps;
    float yaw_rate_max_abs_dps;
    int16 pwm_max_abs;
    uint32 pwm_saturation_samples;
    uint32 line_lost_samples;
}car_telemetry_stats_struct;

typedef struct
{
    uint32 time_ms;
    float distance_m;
    float line_error;
    float yaw_rate_dps;
    uint8 gray_mask;
    uint8 black_count;
    uint8 line_valid;
}car_a_log_sample_struct;

typedef struct
{
    uint16 sample_count;
    uint8 triggered;
    uint8 frozen;
}car_a_log_info_struct;

void car_telemetry_init(void);
void car_telemetry_reset(void);
void car_telemetry_update(float left_speed_mps, float right_speed_mps,
    float left_target_mps, float right_target_mps, float line_error,
    uint8 line_valid, float yaw_rate_target_dps,
    float yaw_rate_feedback_dps, int16 left_pwm, int16 right_pwm);
const car_telemetry_stats_struct *car_telemetry_get_stats(void);
void car_telemetry_update_a_log(uint32 time_ms, float distance_m,
    uint8 gray_mask, uint8 black_count, uint8 line_valid,
    float line_error, float yaw_rate_dps);
void car_telemetry_freeze_a_log(void);
car_a_log_info_struct car_telemetry_get_a_log_info(void);
uint8 car_telemetry_get_a_log_sample(uint16 chronological_index,
    car_a_log_sample_struct *sample);

#endif

#include "car_telemetry.h"

#include <string.h>

#include "car_config.h"

static car_telemetry_stats_struct telemetry_stats;
static car_a_log_sample_struct a_log_samples[CAR_A_LOG_CAPACITY];
static uint16 a_log_write_index;
static uint16 a_log_sample_count;
static uint8 a_log_triggered;
static uint8 a_log_frozen;

static float car_telemetry_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16 car_telemetry_pwm_abs(int16 value)
{
    int32 extended = value;

    if(extended < 0)
    {
        extended = -extended;
    }
    return (int16)extended;
}

static void car_telemetry_update_mean(float *mean, float sample,
    uint32 sample_count)
{
    *mean += (sample - *mean) / (float)sample_count;
}

void car_telemetry_init(void)
{
    car_telemetry_reset();
}

void car_telemetry_reset(void)
{
    memset(&telemetry_stats, 0, sizeof(telemetry_stats));
    memset(a_log_samples, 0, sizeof(a_log_samples));
    a_log_write_index = 0U;
    a_log_sample_count = 0U;
    a_log_triggered = 0U;
    a_log_frozen = 0U;
}

void car_telemetry_update(float left_speed_mps, float right_speed_mps,
    float left_target_mps, float right_target_mps, float line_error,
    uint8 line_valid, float yaw_rate_target_dps,
    float yaw_rate_feedback_dps, int16 left_pwm, int16 right_pwm)
{
    float left_error = car_telemetry_abs(
        left_target_mps - left_speed_mps);
    float right_error = car_telemetry_abs(
        right_target_mps - right_speed_mps);
    float wheel_difference = car_telemetry_abs(
        left_speed_mps - right_speed_mps);
    float line_abs = car_telemetry_abs(line_error);
    float yaw_error = car_telemetry_abs(
        yaw_rate_target_dps - yaw_rate_feedback_dps);
    float yaw_abs = car_telemetry_abs(yaw_rate_feedback_dps);
    int16 left_pwm_abs = car_telemetry_pwm_abs(left_pwm);
    int16 right_pwm_abs = car_telemetry_pwm_abs(right_pwm);
    int16 pwm_abs = (left_pwm_abs > right_pwm_abs) ?
        left_pwm_abs : right_pwm_abs;

    if(telemetry_stats.sample_count < 0xFFFFFFFFU)
    {
        telemetry_stats.sample_count++;
    }
    car_telemetry_update_mean(
        &telemetry_stats.left_speed_error_mean_abs_mps,
        left_error, telemetry_stats.sample_count);
    car_telemetry_update_mean(
        &telemetry_stats.right_speed_error_mean_abs_mps,
        right_error, telemetry_stats.sample_count);
    car_telemetry_update_mean(
        &telemetry_stats.wheel_speed_difference_mean_abs_mps,
        wheel_difference, telemetry_stats.sample_count);
    car_telemetry_update_mean(
        &telemetry_stats.yaw_rate_error_mean_abs_dps,
        yaw_error, telemetry_stats.sample_count);

    if(left_error > telemetry_stats.left_speed_error_max_abs_mps)
    {
        telemetry_stats.left_speed_error_max_abs_mps = left_error;
    }
    if(right_error > telemetry_stats.right_speed_error_max_abs_mps)
    {
        telemetry_stats.right_speed_error_max_abs_mps = right_error;
    }
    if(wheel_difference >
        telemetry_stats.wheel_speed_difference_max_abs_mps)
    {
        telemetry_stats.wheel_speed_difference_max_abs_mps =
            wheel_difference;
    }
    /* 丢线时误差可能仍是上一帧缓存，只统计有效灰度测量。 */
    if(line_valid && (line_abs > telemetry_stats.line_error_max_abs))
    {
        telemetry_stats.line_error_max_abs = line_abs;
    }
    if(yaw_abs > telemetry_stats.yaw_rate_max_abs_dps)
    {
        telemetry_stats.yaw_rate_max_abs_dps = yaw_abs;
    }
    if(yaw_error > telemetry_stats.yaw_rate_error_max_abs_dps)
    {
        telemetry_stats.yaw_rate_error_max_abs_dps = yaw_error;
    }
    if(pwm_abs > telemetry_stats.pwm_max_abs)
    {
        telemetry_stats.pwm_max_abs = pwm_abs;
    }
    if((left_pwm_abs >= CAR_MOTOR_OUTPUT_LIMIT) ||
        (right_pwm_abs >= CAR_MOTOR_OUTPUT_LIMIT))
    {
        telemetry_stats.pwm_saturation_samples++;
    }
    if(!line_valid)
    {
        telemetry_stats.line_lost_samples++;
    }
}

const car_telemetry_stats_struct *car_telemetry_get_stats(void)
{
    return &telemetry_stats;
}

void car_telemetry_update_a_log(uint32 time_ms, float distance_m,
    uint8 gray_mask, uint8 black_count, uint8 line_valid,
    float line_error, float yaw_rate_dps)
{
    car_a_log_sample_struct *sample;

    if(a_log_frozen)
    {
        return;
    }

    sample = &a_log_samples[a_log_write_index];
    sample->time_ms = time_ms;
    sample->distance_m = distance_m;
    sample->line_error = line_error;
    sample->yaw_rate_dps = yaw_rate_dps;
    sample->gray_mask = gray_mask;
    sample->black_count = black_count;
    sample->line_valid = line_valid;

    a_log_write_index++;
    if(a_log_write_index >= CAR_A_LOG_CAPACITY)
    {
        a_log_write_index = 0U;
    }
    if(a_log_sample_count < CAR_A_LOG_CAPACITY)
    {
        a_log_sample_count++;
    }

    if(!a_log_triggered &&
        (black_count >= CAR_A_LOG_TRIGGER_MIN_BLACK))
    {
        a_log_triggered = 1U;
    }
}

void car_telemetry_freeze_a_log(void)
{
    a_log_frozen = 1U;
}

car_a_log_info_struct car_telemetry_get_a_log_info(void)
{
    car_a_log_info_struct info;

    info.sample_count = a_log_sample_count;
    info.triggered = a_log_triggered;
    info.frozen = a_log_frozen;
    return info;
}

uint8 car_telemetry_get_a_log_sample(uint16 chronological_index,
    car_a_log_sample_struct *sample)
{
    uint16 oldest_index;
    uint16 physical_index;

    if((NULL == sample) || (chronological_index >= a_log_sample_count))
    {
        return 0U;
    }

    if(a_log_sample_count < CAR_A_LOG_CAPACITY)
    {
        oldest_index = 0U;
    }
    else
    {
        oldest_index = a_log_write_index;
    }
    physical_index = (uint16)(oldest_index + chronological_index);
    if(physical_index >= CAR_A_LOG_CAPACITY)
    {
        physical_index =
            (uint16)(physical_index - CAR_A_LOG_CAPACITY);
    }
    *sample = a_log_samples[physical_index];
    return 1U;
}

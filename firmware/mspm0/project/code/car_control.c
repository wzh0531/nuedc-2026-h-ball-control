#include "car_control.h"

#include <string.h>

#include "car_config.h"
#include "car_encoder.h"
#include "car_imu.h"
#include "car_motor.h"
#include "car_pid.h"

static car_pid_struct speed_pid_left;
static car_pid_struct speed_pid_right;
static car_pid_struct yaw_rate_pid;
static car_control_state_struct control_state;
static float speed_history_left[CAR_SPEED_SAMPLE_COUNT];
static float speed_history_right[CAR_SPEED_SAMPLE_COUNT];
static uint8 speed_history_index;
static float line_kp;
static float line_kd;
static float previous_line_error;
static float filtered_line_rate;
static int8 last_turn_sign;

static float car_control_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float car_control_limit(float value, float minimum, float maximum)
{
    if(value > maximum)
    {
        value = maximum;
    }
    else if(value < minimum)
    {
        value = minimum;
    }
    return value;
}

static int16 car_control_to_output(float value)
{
    value = car_control_limit(value, -(float)CAR_MOTOR_OUTPUT_LIMIT,
        (float)CAR_MOTOR_OUTPUT_LIMIT);
    return (int16)value;
}

static float car_control_average(const float *history)
{
    uint8 index;
    float sum = 0.0f;

    for(index = 0U; index < CAR_SPEED_SAMPLE_COUNT; index++)
    {
        sum += history[index];
    }
    return sum / (float)CAR_SPEED_SAMPLE_COUNT;
}

static void car_control_sample_wheel_speed(float dt_s)
{
    float left_raw;
    float right_raw;

    left_raw = (float)car_encoder_get_delta(CAR_ENCODER_LEFT) *
        CAR_METERS_PER_COUNT / dt_s;
    right_raw = (float)car_encoder_get_delta(CAR_ENCODER_RIGHT) *
        CAR_METERS_PER_COUNT / dt_s;

    speed_history_left[speed_history_index] = left_raw;
    speed_history_right[speed_history_index] = right_raw;
    speed_history_index++;
    if(speed_history_index >= CAR_SPEED_SAMPLE_COUNT)
    {
        speed_history_index = 0U;
    }

    control_state.left_speed_mps = car_control_average(speed_history_left);
    control_state.right_speed_mps = car_control_average(speed_history_right);
}

static float car_control_line_to_yaw(float line_error, uint8 line_valid,
    float dt_s)
{
    float error_rate;
    float target;

    if(!line_valid)
    {
        return (float)last_turn_sign * CAR_LINE_SEARCH_YAW_RATE_DPS;
    }

    error_rate = (line_error - previous_line_error) / dt_s;
    filtered_line_rate += CAR_LINE_RATE_FILTER_ALPHA *
        (error_rate - filtered_line_rate);
    previous_line_error = line_error;
    target = CAR_LINE_CONTROL_SIGN *
        (line_kp * line_error + line_kd * filtered_line_rate);
    target = car_control_limit(target, -CAR_LINE_YAW_RATE_LIMIT_DPS,
        CAR_LINE_YAW_RATE_LIMIT_DPS);

    if(car_control_abs(target) > 1.0f)
    {
        last_turn_sign = (target > 0.0f) ? 1 : -1;
    }
    return target;
}

static void car_control_apply_wheel_targets(float left_target_mps,
    float right_target_mps, float dt_s)
{
    float left_feedforward;
    float right_feedforward;
    float left_output;
    float right_output;

    control_state.left_target_mps = car_control_limit(left_target_mps,
        -CAR_WHEEL_SPEED_LIMIT_MPS, CAR_WHEEL_SPEED_LIMIT_MPS);
    control_state.right_target_mps = car_control_limit(right_target_mps,
        -CAR_WHEEL_SPEED_LIMIT_MPS, CAR_WHEEL_SPEED_LIMIT_MPS);
    left_feedforward = control_state.left_target_mps * CAR_SPEED_FF_GAIN;
    right_feedforward = control_state.right_target_mps * CAR_SPEED_FF_GAIN;
    left_output = car_pid_update(&speed_pid_left,
        control_state.left_target_mps, control_state.left_speed_mps,
        dt_s, left_feedforward);
    right_output = car_pid_update(&speed_pid_right,
        control_state.right_target_mps, control_state.right_speed_mps,
        dt_s, right_feedforward);
    control_state.left_output = car_control_to_output(left_output);
    control_state.right_output = car_control_to_output(right_output);
    car_motor_set_pair(control_state.left_output, control_state.right_output);
}

static void car_control_apply_yaw_target(float base_speed_mps,
    float yaw_target_dps, float yaw_feedback_dps, float line_error,
    float dt_s)
{
    float yaw_feedforward;
    float yaw_feedback_delta;
    float speed_delta;

    /* 差速运动学前馈：delta_v = B * omega / 2。 */
    yaw_feedforward = CAR_WHEEL_TRACK_M * yaw_target_dps *
        0.0174532925f * 0.5f;
    yaw_feedback_delta = car_pid_update(&yaw_rate_pid, yaw_target_dps,
        yaw_feedback_dps, dt_s, 0.0f);
    speed_delta = car_control_limit(yaw_feedforward + yaw_feedback_delta,
        -CAR_YAW_DELTA_SPEED_LIMIT_MPS,
        CAR_YAW_DELTA_SPEED_LIMIT_MPS);

    control_state.yaw_rate_target_dps = yaw_target_dps;
    control_state.yaw_rate_feedback_dps = yaw_feedback_dps;
    control_state.line_error = line_error;
    control_state.speed_delta_mps = speed_delta;
    car_control_apply_wheel_targets(base_speed_mps - speed_delta,
        base_speed_mps + speed_delta, dt_s);
}

void car_control_init(void)
{
    car_pid_init(&speed_pid_left, CAR_SPEED_KP_DEFAULT, CAR_SPEED_KI_DEFAULT,
        CAR_SPEED_KD_DEFAULT, CAR_SPEED_INTEGRAL_LIMIT,
        CAR_SPEED_OUTPUT_LIMIT);
    car_pid_init(&speed_pid_right, CAR_SPEED_KP_DEFAULT, CAR_SPEED_KI_DEFAULT,
        CAR_SPEED_KD_DEFAULT, CAR_SPEED_INTEGRAL_LIMIT,
        CAR_SPEED_OUTPUT_LIMIT);
    car_pid_init(&yaw_rate_pid, CAR_YAW_KP_DEFAULT, CAR_YAW_KI_DEFAULT,
        CAR_YAW_KD_DEFAULT, CAR_YAW_INTEGRAL_LIMIT,
        CAR_YAW_DELTA_SPEED_LIMIT_MPS);
    line_kp = CAR_LINE_KP_DEFAULT;
    line_kd = CAR_LINE_KD_DEFAULT;
    car_control_reset();
}

void car_control_reset(void)
{
    uint8 index;

    car_pid_reset(&speed_pid_left);
    car_pid_reset(&speed_pid_right);
    car_pid_reset(&yaw_rate_pid);
    for(index = 0U; index < CAR_SPEED_SAMPLE_COUNT; index++)
    {
        speed_history_left[index] = 0.0f;
        speed_history_right[index] = 0.0f;
    }
    speed_history_index = 0U;
    previous_line_error = 0.0f;
    filtered_line_rate = 0.0f;
    last_turn_sign = 1;
    memset(&control_state, 0, sizeof(control_state));
}

void car_control_update(float base_speed_mps, float line_error,
    uint8 line_valid, float dt_s)
{
    float yaw_target;
    float yaw_feedback;

    car_control_sample_wheel_speed(dt_s);
    yaw_target = car_control_line_to_yaw(line_error, line_valid, dt_s);
    yaw_feedback = car_imu_get_rate();
    car_control_apply_yaw_target(base_speed_mps, yaw_target, yaw_feedback,
        line_error, dt_s);
}

void car_control_update_yaw_target(float base_speed_mps,
    float yaw_target_dps, float dt_s)
{
    car_control_sample_wheel_speed(dt_s);
    yaw_target_dps = car_control_limit(yaw_target_dps,
        -CAR_LINE_YAW_RATE_LIMIT_DPS, CAR_LINE_YAW_RATE_LIMIT_DPS);
    car_control_apply_yaw_target(base_speed_mps, yaw_target_dps,
        car_imu_get_rate(), 0.0f, dt_s);
}

void car_control_update_wheel_targets(float left_target_mps,
    float right_target_mps, float dt_s)
{
    car_control_sample_wheel_speed(dt_s);
    control_state.yaw_rate_target_dps = 0.0f;
    control_state.yaw_rate_feedback_dps = 0.0f;
    control_state.line_error = 0.0f;
    control_state.speed_delta_mps = 0.0f;
    car_control_apply_wheel_targets(left_target_mps, right_target_mps, dt_s);
}

const car_control_state_struct *car_control_get_state(void)
{
    return &control_state;
}

void car_control_set_speed_pid(float kp, float ki, float kd)
{
    car_pid_set_gains(&speed_pid_left, kp, ki, kd);
    car_pid_set_gains(&speed_pid_right, kp, ki, kd);
}

void car_control_set_line_pd(float kp, float kd)
{
    line_kp = kp;
    line_kd = kd;
    previous_line_error = 0.0f;
    filtered_line_rate = 0.0f;
}

void car_control_set_yaw_pid(float kp, float ki, float kd)
{
    car_pid_set_gains(&yaw_rate_pid, kp, ki, kd);
}

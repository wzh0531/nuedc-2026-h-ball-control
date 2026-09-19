#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "car_app.h"
#include "car_ball_tracker.h"
#include "car_bldc.h"
#include "car_chassis.h"
#include "car_command.h"
#include "car_config.h"
#include "car_control.h"
#include "car_encoder.h"
#include "car_gray.h"
#include "car_imu.h"
#include "car_telemetry.h"

/*
 * 直接链接正式 car_command.c，通过调试串口环形缓冲桩输入文本命令。
 * 重点覆盖调参有限值校验、运行态命令门禁和开环电机测试边界。
 */
static ChassisState stub_state;
static car_telemetry_stats_struct stub_stats;
static uint8 input_buffer[256];
static uint32 input_length;
static uint32 input_offset;
static uint32 speed_pid_calls;
static uint32 line_pid_calls;
static uint32 yaw_pid_calls;
static uint32 ball_pid_calls;
static uint32 ball3_pid_calls;
static uint32 ball3_left_pid_calls;
static uint32 ball3_right_pid_calls;
static uint32 motor_test_calls;
static uint32 velocity_test_calls;
static uint32 yaw_test_calls;
static uint32 stop_calls;
static uint32 speed_set_calls;
static uint32 a_log_info_calls;
static float last_first;
static float last_second;
static float last_third;
static float last_speed;
static int16 last_motor_left;
static int16 last_motor_right;
static uint32 last_motor_ms;
static float last_velocity_left;
static float last_velocity_right;
static uint32 last_velocity_ms;
static float last_yaw_base;
static float last_yaw_target;
static uint32 last_yaw_ms;

static void test_fail(const char *expression, int line)
{
    fprintf(stderr, "[FAIL] command line %d: %s\n", line, expression);
    exit(1);
}

#define TEST_ASSERT(expression) \
    do \
    { \
        if(!(expression)) \
        { \
            test_fail(#expression, __LINE__); \
        } \
    } while(0)

static uint8 test_near(float actual, float expected)
{
    return (uint8)(fabsf(actual - expected) < 0.0001f);
}

static void test_send(const char *command)
{
    size_t length = strlen(command);

    TEST_ASSERT((length + 1U) <= sizeof(input_buffer));
    memcpy(input_buffer, command, length);
    input_buffer[length] = '\n';
    input_length = (uint32)length + 1U;
    input_offset = 0U;
    while(input_offset < input_length)
    {
        car_command_process();
    }
}

static void test_pid_and_numeric_validation(void)
{
    TEST_ASSERT(test_near(CAR_SPEED_KP_DEFAULT, 7000.0f));
    TEST_ASSERT(test_near(CAR_SPEED_KI_DEFAULT, 5000.0f));
    TEST_ASSERT(test_near(CAR_SPEED_KD_DEFAULT, 0.0f));
    TEST_ASSERT(test_near(CAR_YAW_KP_DEFAULT, 0.0024f));
    TEST_ASSERT(test_near(CAR_YAW_KI_DEFAULT, 0.0f));
    TEST_ASSERT(test_near(CAR_YAW_KD_DEFAULT, 0.0f));
    TEST_ASSERT(test_near(CAR_LINE_KP_DEFAULT, 65.0f));
    TEST_ASSERT(test_near(CAR_LINE_KD_DEFAULT, 0.3f));
    TEST_ASSERT(test_near(CAR_BALL_TASK3_LEFT_KP, 0.17f));
    TEST_ASSERT(test_near(CAR_BALL_TASK3_LEFT_KI, 0.20f));
    TEST_ASSERT(test_near(CAR_BALL_TASK3_LEFT_KD, 0.012f));
    TEST_ASSERT(test_near(CAR_BALL_TASK3_RIGHT_KP, 0.14f));
    TEST_ASSERT(test_near(CAR_BALL_TASK3_RIGHT_KI, 0.21f));
    TEST_ASSERT(test_near(CAR_BALL_TASK3_RIGHT_KD, 0.012f));
    TEST_ASSERT(5U == CAR_BALL_TASK3_LEFT_STABLE_FRAMES);
    TEST_ASSERT(5U == CAR_BALL_TASK3_RIGHT_STABLE_FRAMES);

    test_send("pid speed inf 1 0");
    test_send("pid speed nan 1 0");
    test_send("pid speed 1e100 1 0");
    TEST_ASSERT(0U == speed_pid_calls);

    test_send("pid speed 7000 5000 0");
    TEST_ASSERT(1U == speed_pid_calls);
    TEST_ASSERT(test_near(last_first, 7000.0f));
    TEST_ASSERT(test_near(last_second, 5000.0f));
    TEST_ASSERT(test_near(last_third, 0.0f));

    test_send("pid line 65 0.3");
    TEST_ASSERT(1U == line_pid_calls);
    TEST_ASSERT(test_near(last_first, 65.0f));
    TEST_ASSERT(test_near(last_second, 0.3f));
    test_send("pid yaw 0.0024 0 0");
    TEST_ASSERT(1U == yaw_pid_calls);
    TEST_ASSERT(test_near(last_first, 0.0024f));
    TEST_ASSERT(test_near(last_second, 0.0f));
    TEST_ASSERT(test_near(last_third, 0.0f));

    test_send("pid ball 0.11 0.21 0.012");
    TEST_ASSERT(1U == ball_pid_calls);
    TEST_ASSERT(0U == ball3_pid_calls);
    TEST_ASSERT(test_near(last_first, 0.11f));
    TEST_ASSERT(test_near(last_second, 0.21f));
    TEST_ASSERT(test_near(last_third, 0.012f));

    test_send("pid ball3 0.15 0.10 0.005");
    TEST_ASSERT(1U == ball_pid_calls);
    TEST_ASSERT(1U == ball3_pid_calls);
    TEST_ASSERT(test_near(last_first, 0.15f));
    TEST_ASSERT(test_near(last_second, 0.10f));
    TEST_ASSERT(test_near(last_third, 0.005f));

    test_send("pid ball3l 0.17 0.20 0.012");
    TEST_ASSERT(1U == ball3_left_pid_calls);
    TEST_ASSERT(0U == ball3_right_pid_calls);
    TEST_ASSERT(test_near(last_first, 0.17f));
    TEST_ASSERT(test_near(last_second, 0.20f));
    TEST_ASSERT(test_near(last_third, 0.012f));

    test_send("pid ball3r 0.14 0.21 0.012");
    TEST_ASSERT(1U == ball3_left_pid_calls);
    TEST_ASSERT(1U == ball3_right_pid_calls);
    TEST_ASSERT(test_near(last_first, 0.14f));
    TEST_ASSERT(test_near(last_second, 0.21f));
    TEST_ASSERT(test_near(last_third, 0.012f));

    test_send("speed inf");
    TEST_ASSERT(0U == speed_set_calls);
    test_send("speed 0.25");
    TEST_ASSERT(1U == speed_set_calls);
    TEST_ASSERT(test_near(last_speed, 0.25f));
}

static void test_task_motor_and_runtime_guard(void)
{
    test_send("alog");
    TEST_ASSERT(1U == a_log_info_calls);

    test_send("task 4");
    TEST_ASSERT(CHASSIS_TASK_4_A_TO_B == stub_state.task);

    test_send("motor 1200 -900 300");
    TEST_ASSERT(1U == motor_test_calls);
    TEST_ASSERT(1200 == last_motor_left);
    TEST_ASSERT(-900 == last_motor_right);
    TEST_ASSERT(300U == last_motor_ms);

    test_send("motor 6501 0 300");
    test_send("motor 1000 0 1501");
    TEST_ASSERT(1U == motor_test_calls);

    test_send("vel -0.1 0.1 500");
    test_send("vel 0.36 0.1 500");
    test_send("vel 0.2 0.1 199");
    test_send("vel 0.2 0.1 3001");
    TEST_ASSERT(0U == velocity_test_calls);
    test_send("vel 0.20 0.10 1000");
    TEST_ASSERT(1U == velocity_test_calls);
    TEST_ASSERT(test_near(last_velocity_left, 0.20f));
    TEST_ASSERT(test_near(last_velocity_right, 0.10f));
    TEST_ASSERT(1000U == last_velocity_ms);

    test_send("yaw 0.17 20 500");
    test_send("yaw 0.20 31 500");
    test_send("yaw 0.20 20 199");
    test_send("yaw 0.20 20 3001");
    TEST_ASSERT(0U == yaw_test_calls);
    test_send("yaw 0.20 20 1000");
    TEST_ASSERT(1U == yaw_test_calls);
    TEST_ASSERT(test_near(last_yaw_base, 0.20f));
    TEST_ASSERT(test_near(last_yaw_target, 20.0f));
    TEST_ASSERT(1000U == last_yaw_ms);

    stub_state.mode = CHASSIS_MODE_RUNNING;
    test_send("pid speed 1 1 0");
    TEST_ASSERT(1U == speed_pid_calls);
    test_send("status");
    TEST_ASSERT(CHASSIS_MODE_RUNNING == stub_state.mode);
    test_send("alog");
    TEST_ASSERT(1U == a_log_info_calls);
    test_send("stop");
    TEST_ASSERT(1U == stop_calls);
}

int main(void)
{
    memset(&stub_state, 0, sizeof(stub_state));
    stub_state.mode = CHASSIS_MODE_READY;
    stub_state.task = CHASSIS_TASK_2_LAP_STOP_A;
    car_command_init();
    test_pid_and_numeric_validation();
    test_task_motor_and_runtime_guard();
    printf("[PASS] UART command finite values, limits and runtime guard\n");
    return 0;
}

/* --------------------------- command dependency stubs --------------------------- */

uint32 debug_read_ring_buffer(uint8 *buffer, uint32 length)
{
    uint32 remaining = input_length - input_offset;

    if(length > remaining)
    {
        length = remaining;
    }
    memcpy(buffer, &input_buffer[input_offset], length);
    input_offset += length;
    return length;
}

const ChassisState *chassis_get_state(void)
{
    return &stub_state;
}

uint8 chassis_start(void)
{
    if(CHASSIS_MODE_READY != stub_state.mode)
    {
        return 0U;
    }
    stub_state.mode = CHASSIS_MODE_RUNNING;
    return 1U;
}

void chassis_stop(void)
{
    stop_calls++;
    stub_state.mode = CHASSIS_MODE_FINISHED;
}

uint8 chassis_reset(void)
{
    stub_state.mode = CHASSIS_MODE_READY;
    stub_state.fault_bits = 0U;
    return 1U;
}

uint8 chassis_select_task(ChassisTask task)
{
    if((task < CHASSIS_TASK_2_LAP_STOP_A) ||
        (task > CHASSIS_TASK_6_LAP_POSITION))
    {
        return 0U;
    }
    stub_state.task = task;
    return 1U;
}

void chassis_set_base_speed(float speed_mps)
{
    speed_set_calls++;
    last_speed = speed_mps;
}

uint8 chassis_start_motor_test(int16 left, int16 right, uint32 duration_ms)
{
    if((CHASSIS_MODE_READY != stub_state.mode) ||
        (0U == duration_ms) ||
        (duration_ms > CAR_MOTOR_TEST_MAX_MS))
    {
        return 0U;
    }
    motor_test_calls++;
    last_motor_left = left;
    last_motor_right = right;
    last_motor_ms = duration_ms;
    return 1U;
}

uint8 chassis_start_velocity_test(float left_mps, float right_mps,
    uint32 duration_ms)
{
    if((CHASSIS_MODE_READY != stub_state.mode) ||
        (duration_ms < CAR_VELOCITY_TEST_MIN_MS) ||
        (duration_ms > CAR_VELOCITY_TEST_MAX_MS) ||
        (left_mps < 0.0f) ||
        (left_mps > CAR_VELOCITY_TEST_MAX_SPEED_MPS) ||
        (right_mps < 0.0f) ||
        (right_mps > CAR_VELOCITY_TEST_MAX_SPEED_MPS))
    {
        return 0U;
    }
    velocity_test_calls++;
    last_velocity_left = left_mps;
    last_velocity_right = right_mps;
    last_velocity_ms = duration_ms;
    return 1U;
}

uint8 chassis_start_yaw_test(float base_speed_mps, float yaw_target_dps,
    uint32 duration_ms)
{
    if((CHASSIS_MODE_READY != stub_state.mode) ||
        (base_speed_mps < CAR_YAW_TEST_BASE_MIN_MPS) ||
        (base_speed_mps > CAR_YAW_TEST_BASE_MAX_MPS) ||
        (fabsf(yaw_target_dps) > CAR_YAW_TEST_MAX_RATE_DPS) ||
        (duration_ms < CAR_YAW_TEST_MIN_MS) ||
        (duration_ms > CAR_YAW_TEST_MAX_MS))
    {
        return 0U;
    }
    yaw_test_calls++;
    last_yaw_base = base_speed_mps;
    last_yaw_target = yaw_target_dps;
    last_yaw_ms = duration_ms;
    return 1U;
}

void car_control_set_speed_pid(float kp, float ki, float kd)
{
    speed_pid_calls++;
    last_first = kp;
    last_second = ki;
    last_third = kd;
}

void car_control_set_line_pd(float kp, float kd)
{
    line_pid_calls++;
    last_first = kp;
    last_second = kd;
}

void car_control_set_yaw_pid(float kp, float ki, float kd)
{
    yaw_pid_calls++;
    last_first = kp;
    last_second = ki;
    last_third = kd;
}

int32 car_encoder_get_total(car_encoder_id_enum encoder)
{
    (void)encoder;
    return 0;
}

uint32 car_encoder_get_invalid(car_encoder_id_enum encoder)
{
    (void)encoder;
    return 0U;
}

void car_encoder_reset(void)
{
}

uint8 car_gray_read(void)
{
    return 0x18U;
}

uint8 car_gray_get_mask(void)
{
    return 0x18U;
}

uint8 car_gray_get_black_count(void)
{
    return 2U;
}

uint8 car_imu_calibrate(void)
{
    return 1U;
}

float car_imu_get_bias(void)
{
    return 0.0f;
}

float car_imu_get_residual(void)
{
    return 0.0f;
}

const car_telemetry_stats_struct *car_telemetry_get_stats(void)
{
    return &stub_stats;
}

car_a_log_info_struct car_telemetry_get_a_log_info(void)
{
    car_a_log_info_struct info;

    a_log_info_calls++;
    info.sample_count = 0U;
    info.triggered = 0U;
    info.frozen = 0U;
    return info;
}

uint8 car_telemetry_get_a_log_sample(uint16 chronological_index,
    car_a_log_sample_struct *sample)
{
    (void)chronological_index;
    (void)sample;
    return 0U;
}

void car_app_clear_pending_ticks(void)
{
}

uint32 car_app_get_overrun_count(void)
{
    return 0U;
}

uint8 car_app_get_max_pending_ticks(void)
{
    return 0U;
}

volatile car_bldc_motor_data_struct car_bldc_motor_2;

void car_ball_tracker_set_pid(float kp, float ki, float kd)
{
    ball_pid_calls++;
    last_first = kp;
    last_second = ki;
    last_third = kd;
}

void car_ball_tracker_set_task3_pid(float kp, float ki, float kd)
{
    ball3_pid_calls++;
    last_first = kp;
    last_second = ki;
    last_third = kd;
}

void car_ball_tracker_set_task3_left_pid(float kp, float ki, float kd)
{
    ball3_left_pid_calls++;
    last_first = kp;
    last_second = ki;
    last_third = kd;
}

void car_ball_tracker_set_task3_right_pid(float kp, float ki, float kd)
{
    ball3_right_pid_calls++;
    last_first = kp;
    last_second = ki;
    last_third = kd;
}

void car_ball_tracker_set_target(float target_pixels)
{
    (void)target_pixels;
}

uint8 car_ball_tracker_is_tracking(void)
{
    return 0U;
}

int16 car_ball_tracker_get_dx(void)
{
    return 0;
}

float car_ball_tracker_get_target(void)
{
    return 0.0f;
}

int32 car_ball_tracker_get_command_angle_x10(void)
{
    return 0;
}

uint32 car_ball_tracker_get_frame_count(void)
{
    return 0U;
}

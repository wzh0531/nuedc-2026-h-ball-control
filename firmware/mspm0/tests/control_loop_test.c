#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "car_config.h"
#include "car_control.h"
#include "car_encoder.h"
#include "car_imu.h"
#include "car_motor.h"
#include "car_pid.h"

static int16 stub_encoder_delta[CAR_ENCODER_COUNT];
static float stub_yaw_rate_dps;
static int16 stub_motor_left;
static int16 stub_motor_right;
static unsigned int test_count;

static void test_fail(const char *name, const char *expression, int line)
{
    fprintf(stderr, "[FAIL] %s line %d: %s\n", name, line, expression);
    exit(1);
}

#define TEST_ASSERT(name, expression) \
    do \
    { \
        if(!(expression)) \
        { \
            test_fail((name), #expression, __LINE__); \
        } \
    } while(0)

static void test_pass(const char *name)
{
    test_count++;
    printf("[PASS] %s\n", name);
}

static void test_three_sample_speed_filter(void)
{
    const car_control_state_struct *state;
    float raw_speed = 11.0f * CAR_METERS_PER_COUNT / CAR_CONTROL_DT_S;
    uint8 index;

    car_control_init();
    stub_encoder_delta[CAR_ENCODER_LEFT] = 11;
    stub_encoder_delta[CAR_ENCODER_RIGHT] = 11;
    stub_yaw_rate_dps = 0.0f;

    for(index = 0U; index < CAR_SPEED_SAMPLE_COUNT; index++)
    {
        car_control_update(0.20f, 0.0f, 1U, CAR_CONTROL_DT_S);
    }
    state = car_control_get_state();
    TEST_ASSERT("left three-sample speed",
        fabsf(state->left_speed_mps - raw_speed) < 0.00001f);
    TEST_ASSERT("right three-sample speed",
        fabsf(state->right_speed_mps - raw_speed) < 0.00001f);
    TEST_ASSERT("straight targets equal",
        fabsf(state->left_target_mps - state->right_target_mps) <
        0.000001f);
    TEST_ASSERT("straight outputs equal",
        state->left_output == state->right_output);
    TEST_ASSERT("motor receives left output",
        state->left_output == stub_motor_left);
    TEST_ASSERT("motor receives right output",
        state->right_output == stub_motor_right);
    test_pass("three-sample wheel-speed filter and straight symmetry");
}

static void test_line_and_yaw_sign(void)
{
    const car_control_state_struct *state;

    car_control_reset();
    stub_encoder_delta[CAR_ENCODER_LEFT] = 0;
    stub_encoder_delta[CAR_ENCODER_RIGHT] = 0;
    stub_yaw_rate_dps = 0.0f;

    /*
     * The line appears on the physical left (negative weighted error), so the
     * chassis must command positive yaw: right wheel faster than left.
     */
    car_control_update(0.20f, -0.50f, 1U, CAR_CONTROL_DT_S);
    state = car_control_get_state();
    TEST_ASSERT("negative line error commands positive yaw",
        state->yaw_rate_target_dps > 0.0f);
    TEST_ASSERT("positive yaw makes right target faster",
        state->right_target_mps > state->left_target_mps);
    TEST_ASSERT("speed delta within limit",
        fabsf(state->speed_delta_mps) <=
        CAR_YAW_DELTA_SPEED_LIMIT_MPS + 0.000001f);

    car_control_update(0.10f, 0.0f, 0U, CAR_CONTROL_DT_S);
    state = car_control_get_state();
    TEST_ASSERT("lost line remembers last turn direction",
        state->yaw_rate_target_dps ==
        CAR_LINE_SEARCH_YAW_RATE_DPS);
    test_pass("line outer-loop sign and lost-line turn memory");
}

static void test_output_saturation(void)
{
    const car_control_state_struct *state;

    car_control_reset();
    stub_encoder_delta[CAR_ENCODER_LEFT] = -300;
    stub_encoder_delta[CAR_ENCODER_RIGHT] = -300;
    stub_yaw_rate_dps = -500.0f;
    car_control_update(CAR_WHEEL_SPEED_LIMIT_MPS, -1.0f, 1U,
        CAR_CONTROL_DT_S);
    state = car_control_get_state();

    TEST_ASSERT("left output limited",
        (state->left_output <= CAR_MOTOR_OUTPUT_LIMIT) &&
        (state->left_output >= -CAR_MOTOR_OUTPUT_LIMIT));
    TEST_ASSERT("right output limited",
        (state->right_output <= CAR_MOTOR_OUTPUT_LIMIT) &&
        (state->right_output >= -CAR_MOTOR_OUTPUT_LIMIT));
    TEST_ASSERT("left target limited",
        fabsf(state->left_target_mps) <=
        CAR_WHEEL_SPEED_LIMIT_MPS + 0.000001f);
    TEST_ASSERT("right target limited",
        fabsf(state->right_target_mps) <=
        CAR_WHEEL_SPEED_LIMIT_MPS + 0.000001f);
    test_pass("wheel target, yaw delta and PWM saturation");
}

static void test_direct_wheel_speed_targets(void)
{
    const car_control_state_struct *state;

    car_control_reset();
    stub_encoder_delta[CAR_ENCODER_LEFT] = 0;
    stub_encoder_delta[CAR_ENCODER_RIGHT] = 0;
    stub_yaw_rate_dps = 500.0f;
    car_control_update_wheel_targets(0.20f, 0.10f, CAR_CONTROL_DT_S);
    state = car_control_get_state();

    TEST_ASSERT("direct left target",
        fabsf(state->left_target_mps - 0.20f) < 0.000001f);
    TEST_ASSERT("direct right target",
        fabsf(state->right_target_mps - 0.10f) < 0.000001f);
    TEST_ASSERT("direct mode bypasses yaw target",
        fabsf(state->yaw_rate_target_dps) < 0.000001f);
    TEST_ASSERT("direct mode bypasses yaw feedback",
        fabsf(state->yaw_rate_feedback_dps) < 0.000001f);
    TEST_ASSERT("larger target produces larger initial output",
        state->left_output > state->right_output);
    test_pass("direct wheel targets bypass line and yaw loops");
}

static void test_direct_yaw_rate_target(void)
{
    const car_control_state_struct *state;

    car_control_reset();
    stub_encoder_delta[CAR_ENCODER_LEFT] = 0;
    stub_encoder_delta[CAR_ENCODER_RIGHT] = 0;
    stub_yaw_rate_dps = 0.0f;
    car_control_update_yaw_target(0.20f, 20.0f, CAR_CONTROL_DT_S);
    state = car_control_get_state();

    TEST_ASSERT("direct yaw target retained",
        fabsf(state->yaw_rate_target_dps - 20.0f) < 0.000001f);
    TEST_ASSERT("direct yaw reads IMU feedback",
        fabsf(state->yaw_rate_feedback_dps) < 0.000001f);
    TEST_ASSERT("direct yaw bypasses line error",
        fabsf(state->line_error) < 0.000001f);
    TEST_ASSERT("positive yaw makes right wheel faster",
        state->right_target_mps > state->left_target_mps);
    TEST_ASSERT("direct yaw delta remains bounded",
        fabsf(state->speed_delta_mps) <=
        CAR_YAW_DELTA_SPEED_LIMIT_MPS + 0.000001f);
    test_pass("direct IMU yaw-rate target bypasses line loop");
}

static void test_pid_conditional_integration(void)
{
    car_pid_struct pid;
    uint8 index;

    car_pid_init(&pid, 0.0f, 10.0f, 0.0f, 1.0f, 1.0f);
    for(index = 0U; index < 50U; index++)
    {
        (void)car_pid_update(&pid, 1.0f, 0.0f, CAR_CONTROL_DT_S, 2.0f);
    }
    TEST_ASSERT("saturated PID blocks further integration",
        fabsf(pid.integral) < 0.000001f);

    (void)car_pid_update(&pid, -1.0f, 0.0f, CAR_CONTROL_DT_S, 2.0f);
    TEST_ASSERT("opposite error can unwind saturation",
        pid.integral < 0.0f);
    TEST_ASSERT("integral remains bounded",
        fabsf(pid.integral) <= pid.integral_limit);
    test_pass("PID conditional integration anti-windup");
}

int main(void)
{
    test_three_sample_speed_filter();
    test_line_and_yaw_sign();
    test_output_saturation();
    test_direct_wheel_speed_targets();
    test_direct_yaw_rate_target();
    test_pid_conditional_integration();
    printf("[PASS] all %u control-loop tests\n", test_count);
    return 0;
}

/* Hardware-facing dependencies used by the production control module. */

int16 car_encoder_get_delta(car_encoder_id_enum encoder)
{
    return stub_encoder_delta[encoder];
}

float car_imu_get_rate(void)
{
    return stub_yaw_rate_dps;
}

void car_motor_set_pair(int16 left, int16 right)
{
    stub_motor_left = left;
    stub_motor_right = right;
}

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "car_config.h"
#include "car_telemetry.h"

static void test_fail(const char *expression, int line)
{
    fprintf(stderr, "[FAIL] telemetry line %d: %s\n", line, expression);
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

#define TEST_NEAR(actual, expected, tolerance) \
    TEST_ASSERT(fabsf((actual) - (expected)) <= (tolerance))

int main(void)
{
    const car_telemetry_stats_struct *stats;
    car_a_log_info_struct a_log_info;
    car_a_log_sample_struct a_log_sample;
    uint16 index;

    car_telemetry_init();
    car_telemetry_update(0.10f, 0.15f, 0.20f, 0.20f,
        -0.70f, 1U, 10.0f, -20.0f, 3000, -4000);
    car_telemetry_update(0.25f, 0.10f, 0.20f, 0.20f,
        0.90f, 0U, 20.0f, 30.0f, CAR_MOTOR_OUTPUT_LIMIT, 1200);

    stats = car_telemetry_get_stats();
    TEST_ASSERT(2U == stats->sample_count);
    TEST_NEAR(stats->left_speed_error_mean_abs_mps, 0.075f, 0.00001f);
    TEST_NEAR(stats->right_speed_error_mean_abs_mps, 0.075f, 0.00001f);
    TEST_NEAR(stats->left_speed_error_max_abs_mps, 0.10f, 0.00001f);
    TEST_NEAR(stats->right_speed_error_max_abs_mps, 0.10f, 0.00001f);
    TEST_NEAR(stats->wheel_speed_difference_mean_abs_mps,
        0.10f, 0.00001f);
    TEST_NEAR(stats->wheel_speed_difference_max_abs_mps,
        0.15f, 0.00001f);
    TEST_NEAR(stats->line_error_max_abs, 0.70f, 0.00001f);
    TEST_NEAR(stats->yaw_rate_error_mean_abs_dps, 20.0f, 0.00001f);
    TEST_NEAR(stats->yaw_rate_error_max_abs_dps, 30.0f, 0.00001f);
    TEST_NEAR(stats->yaw_rate_max_abs_dps, 30.0f, 0.00001f);
    TEST_ASSERT(CAR_MOTOR_OUTPUT_LIMIT == stats->pwm_max_abs);
    TEST_ASSERT(1U == stats->pwm_saturation_samples);
    TEST_ASSERT(1U == stats->line_lost_samples);

    car_telemetry_reset();
    stats = car_telemetry_get_stats();
    TEST_ASSERT(0U == stats->sample_count);
    TEST_NEAR(stats->left_speed_error_mean_abs_mps, 0.0f, 0.00001f);
    TEST_ASSERT(0 == stats->pwm_max_abs);

    /* 先写满滚动历史，再以 black=3 标记候选，但不得提前冻结。 */
    for(index = 0U; index < 200U; index++)
    {
        car_telemetry_update_a_log((uint32)index * 5U,
            5.50f + (float)index * 0.001f, 0x18U, 2U, 1U,
            0.0f, 5.0f);
    }
    car_telemetry_update_a_log(1000U, 5.700f, 0x1CU, 3U, 1U,
        0.10f, 2.0f);
    for(index = 0U; index < 100U; index++)
    {
        car_telemetry_update_a_log(1005U + (uint32)index * 5U,
            5.701f + (float)index * 0.001f, 0x1CU, 3U, 1U,
            0.10f, 2.0f);
    }
    a_log_info = car_telemetry_get_a_log_info();
    TEST_ASSERT(CAR_A_LOG_CAPACITY == a_log_info.sample_count);
    TEST_ASSERT(1U == a_log_info.triggered);
    TEST_ASSERT(0U == a_log_info.frozen);
    TEST_ASSERT(car_telemetry_get_a_log_sample(0U, &a_log_sample));
    TEST_ASSERT(225U == a_log_sample.time_ms);
    TEST_ASSERT(car_telemetry_get_a_log_sample(
        (uint16)(a_log_info.sample_count - 1U), &a_log_sample));
    TEST_ASSERT(3U == a_log_sample.black_count);
    TEST_ASSERT(!car_telemetry_get_a_log_sample(
        a_log_info.sample_count, &a_log_sample));
    car_telemetry_freeze_a_log();
    a_log_info = car_telemetry_get_a_log_info();
    TEST_ASSERT(1U == a_log_info.frozen);
    car_telemetry_update_a_log(9999U, 9.999f, 0xFFU, 8U, 1U,
        1.0f, 90.0f);
    TEST_ASSERT(car_telemetry_get_a_log_sample(
        (uint16)(a_log_info.sample_count - 1U), &a_log_sample));
    TEST_ASSERT(9999U != a_log_sample.time_ms);

    car_telemetry_reset();
    a_log_info = car_telemetry_get_a_log_info();
    TEST_ASSERT(0U == a_log_info.sample_count);
    TEST_ASSERT(0U == a_log_info.triggered);
    TEST_ASSERT(0U == a_log_info.frozen);

    printf("[PASS] non-blocking telemetry statistics and reset\n");
    return 0;
}

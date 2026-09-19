#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "car_config.h"
#include "car_planner.h"

static void test_fail(const char *expression, int line)
{
    fprintf(stderr, "[FAIL] planner line %d: %s\n", line, expression);
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

int main(void)
{
    car_planner_struct planner;
    float previous_acceleration;
    uint32 index;

    TEST_ASSERT(CAR_ACCEL_LIMIT_MPS2 <= 0.3001f);
    TEST_ASSERT(fabsf(CAR_BALANCE_ACCEL_LIMIT_MPS2 - 0.15f) < 0.0001f);
    TEST_ASSERT(CAR_DECEL_LIMIT_MPS2 <= 0.3501f);
    TEST_ASSERT(CAR_JERK_LIMIT_MPS3 <= 1.2001f);

    car_planner_init(&planner);
    TEST_ASSERT(fabsf(planner.acceleration_limit_mps2 -
        CAR_ACCEL_LIMIT_MPS2) < 0.0001f);
    car_planner_set_target(&planner, CAR_RACE_TASK2_SPEED_MPS);
    previous_acceleration = planner.acceleration_mps2;
    for(index = 0U; index < 2000U; index++)
    {
        (void)car_planner_update(&planner, CAR_CONTROL_DT_S);
        TEST_ASSERT(fabsf(planner.acceleration_mps2) <=
            CAR_ACCEL_LIMIT_MPS2 + 0.0001f);
        TEST_ASSERT(fabsf(planner.acceleration_mps2 -
            previous_acceleration) <=
            CAR_JERK_LIMIT_MPS3 * CAR_CONTROL_DT_S + 0.0001f);
        previous_acceleration = planner.acceleration_mps2;
        if((fabsf(planner.current_mps -
            CAR_RACE_TASK2_SPEED_MPS) < 0.000001f) &&
            (fabsf(planner.acceleration_mps2) < 0.000001f))
        {
            break;
        }
    }
    TEST_ASSERT(index < 2000U);
    TEST_ASSERT(fabsf(planner.acceleration_mps2) < 0.000001f);

    car_planner_force_zero(&planner);
    car_planner_set_acceleration_limit(&planner,
        CAR_BALANCE_ACCEL_LIMIT_MPS2);
    car_planner_set_target(&planner, CAR_RACE_TASK56_SPEED_MPS);
    for(index = 0U; index < 2000U; index++)
    {
        (void)car_planner_update(&planner, CAR_CONTROL_DT_S);
        TEST_ASSERT(fabsf(planner.acceleration_mps2) <=
            CAR_BALANCE_ACCEL_LIMIT_MPS2 + 0.0001f);
        if((fabsf(planner.current_mps -
            CAR_RACE_TASK56_SPEED_MPS) < 0.000001f) &&
            (fabsf(planner.acceleration_mps2) < 0.000001f))
        {
            break;
        }
    }
    TEST_ASSERT(index < 2000U);

    car_planner_set_target(&planner, 0.0f);
    previous_acceleration = planner.acceleration_mps2;
    for(index = 0U; index < 2000U; index++)
    {
        (void)car_planner_update(&planner, CAR_CONTROL_DT_S);
        TEST_ASSERT(fabsf(planner.acceleration_mps2) <=
            CAR_DECEL_LIMIT_MPS2 + 0.0001f);
        TEST_ASSERT(fabsf(planner.acceleration_mps2 -
            previous_acceleration) <=
            CAR_JERK_LIMIT_MPS3 * CAR_CONTROL_DT_S + 0.0001f);
        TEST_ASSERT(planner.current_mps >= -0.000001f);
        previous_acceleration = planner.acceleration_mps2;
        if((fabsf(planner.current_mps) < 0.000001f) &&
            (fabsf(planner.acceleration_mps2) < 0.000001f))
        {
            break;
        }
    }
    TEST_ASSERT(index < 2000U);
    TEST_ASSERT(fabsf(planner.acceleration_mps2) < 0.000001f);

    printf("[PASS] jerk-limited S-curve speed planner\n");
    return 0;
}

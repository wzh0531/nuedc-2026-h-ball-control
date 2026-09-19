#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "car_chassis.h"
#include "car_config.h"
#include "car_control.h"
#include "car_encoder.h"
#include "car_gray.h"
#include "car_imu.h"
#include "car_motor.h"
#include "car_safety.h"
#include "car_telemetry.h"
/*
 * 该测试直接链接正式的 car_chassis.c、car_safety.c 和 car_planner.c。
 * 这里只替换硬件相关模块，让任务状态机和安全条件能在 PC 上确定性运行。
 */
static car_control_state_struct stub_control_state;
static int32 stub_encoder_total[CAR_ENCODER_COUNT];
static int16 stub_encoder_delta[CAR_ENCODER_COUNT];
static uint32 stub_encoder_invalid[CAR_ENCODER_COUNT];
static uint8 stub_gray_valid;
static uint8 stub_gray_mask;
static uint8 stub_gray_black_count;
static float stub_gray_error;
static uint8 stub_imu_ready;
static uint8 stub_imu_calibrating;
static uint8 stub_imu_calibration_steps;
static uint8 stub_imu_calibration_success;
static float stub_imu_rate;
static int16 stub_motor_output[CAR_MOTOR_COUNT];
static int16 stub_control_pwm_left;
static int16 stub_control_pwm_right;
static uint32 stub_velocity_update_calls;
static uint32 stub_yaw_update_calls;
static uint8 stub_simulation_enabled;
static float stub_simulation_distance_m;
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

static int32 test_distance_to_count(float distance_m)
{
    return (int32)(distance_m / CAR_METERS_PER_COUNT + 0.5f);
}

static void test_set_distance(float distance_m)
{
    int32 count = test_distance_to_count(distance_m);

    stub_encoder_total[CAR_ENCODER_LEFT] = count;
    stub_encoder_total[CAR_ENCODER_RIGHT] = count;
}

static void test_tick(uint32 count)
{
    while(count > 0U)
    {
        chassis_update_5ms();
        count--;
    }
}

static void test_reset_stubs(void)
{
    memset(&stub_control_state, 0, sizeof(stub_control_state));
    memset(stub_encoder_total, 0, sizeof(stub_encoder_total));
    memset(stub_encoder_delta, 0, sizeof(stub_encoder_delta));
    memset(stub_encoder_invalid, 0, sizeof(stub_encoder_invalid));
    memset(stub_motor_output, 0, sizeof(stub_motor_output));
    stub_gray_valid = 1U;
    stub_gray_mask = 0x18U;
    stub_gray_black_count = 2U;
    stub_gray_error = 0.0f;
    stub_imu_ready = 1U;
    stub_imu_calibrating = 0U;
    stub_imu_calibration_steps = 0U;
    stub_imu_calibration_success = 1U;
    stub_imu_rate = 0.0f;
    stub_control_pwm_left = 0;
    stub_control_pwm_right = 0;
    stub_velocity_update_calls = 0U;
    stub_yaw_update_calls = 0U;
    stub_simulation_enabled = 0U;
    stub_simulation_distance_m = 0.0f;
}

static void test_new_chassis(void)
{
    test_reset_stubs();
    chassis_init();
}

static void test_initial_state_and_selection(void)
{
    ChassisTask task;

    test_new_chassis();
    TEST_ASSERT("initial state", CHASSIS_MODE_READY ==
        chassis_get_state()->mode);
    TEST_ASSERT("default task", CHASSIS_TASK_2_LAP_STOP_A ==
        chassis_get_state()->task);

    for(task = CHASSIS_TASK_2_LAP_STOP_A;
        task <= CHASSIS_TASK_6_LAP_POSITION;
        task = (ChassisTask)((int)task + 1))
    {
        TEST_ASSERT("select task", chassis_select_task(task));
        TEST_ASSERT("selected task visible", task ==
            chassis_get_state()->task);
    }
    TEST_ASSERT("reject task below range",
        !chassis_select_task((ChassisTask)1));
    TEST_ASSERT("reject task above range",
        !chassis_select_task((ChassisTask)7));
    test_pass("initial state and task selection");
}

static void test_imu_readiness_gate_and_recovery(void)
{
    test_reset_stubs();
    stub_imu_ready = 0U;
    chassis_init();
    TEST_ASSERT("imu not ready enters fault",
        CHASSIS_MODE_FAULT == chassis_get_state()->mode);
    TEST_ASSERT("imu fault bit latched", 0U !=
        (chassis_get_state()->fault_bits & CHASSIS_FAULT_IMU));
    TEST_ASSERT("imu not ready rejects start", !chassis_start());
    TEST_ASSERT("imu not ready rejects reset", !chassis_reset());
    /* 模拟停车后重新标定成功，再由人工复位回到 READY。 */
    stub_imu_ready = 1U;
    TEST_ASSERT("imu recovery accepts reset", chassis_reset());
    TEST_ASSERT("imu recovery returns ready",
        CHASSIS_MODE_READY == chassis_get_state()->mode);
    TEST_ASSERT("imu recovery clears latched fault",
        CHASSIS_FAULT_NONE == chassis_get_state()->fault_bits);
    test_pass("IMU readiness gate and manual recovery");

    test_reset_stubs();
    stub_imu_ready = 0U;
    stub_imu_calibrating = 1U;
    stub_imu_calibration_steps = 2U;
    chassis_init();
    TEST_ASSERT("startup enters calibration",
        CHASSIS_MODE_CALIBRATE == chassis_get_state()->mode);
    TEST_ASSERT("calibration rejects start", !chassis_start());
    test_tick(1U);
    TEST_ASSERT("calibration remains active",
        CHASSIS_MODE_CALIBRATE == chassis_get_state()->mode);
    test_tick(1U);
    TEST_ASSERT("calibration completes ready",
        CHASSIS_MODE_READY == chassis_get_state()->mode);
    TEST_ASSERT("calibration does not start timer",
        0U == chassis_get_state()->run_time_ms);
    TEST_ASSERT("calibration keeps motors stopped",
        (0 == stub_motor_output[CAR_MOTOR_LEFT]) &&
        (0 == stub_motor_output[CAR_MOTOR_RIGHT]));
    test_pass("non-blocking IMU startup calibration state");

    test_reset_stubs();
    stub_imu_ready = 0U;
    stub_imu_calibrating = 1U;
    stub_imu_calibration_steps = 1U;
    stub_imu_calibration_success = 0U;
    chassis_init();
    test_tick(1U);
    TEST_ASSERT("failed calibration enters fault",
        CHASSIS_MODE_FAULT == chassis_get_state()->mode);
    TEST_ASSERT("failed calibration latches imu", 0U !=
        (chassis_get_state()->fault_bits & CHASSIS_FAULT_IMU));
    TEST_ASSERT("failed calibration keeps timer zero",
        0U == chassis_get_state()->run_time_ms);
    test_pass("IMU startup calibration failure latch");
}

/*
 * 理想执行器任务仿真：
 * - 正常黑线始终位于灰度中心；
 * - 行驶到一圈名义里程后给出 5 cm 宽的 A 点黑线；
 * - 轮速完全跟随规划速度，并积分成编码器里程。
 *
 * 它验证的是完整任务的状态/里程/计时闭环，不代表实车动力学或 PID 已调好。
 */
static void test_simulation_tick(uint8 enable_a_line)
{
    const ChassisState *state;
    float sensor_reaches_a_distance;
    float lap_position;
    float body_speed;
    int32 count;

    /*
     * 起点是车体测试点位于 A，前置灰度在其前方。绕行返回时，灰度先到 A，
     * 此时车体里程比完整一圈少 CAR_SENSOR_TO_TEST_POINT_M。
     */
    sensor_reaches_a_distance = CAR_LAP_NOMINAL_DISTANCE_M -
        CAR_SENSOR_TO_TEST_POINT_M;
    if(enable_a_line &&
        (stub_simulation_distance_m >= sensor_reaches_a_distance) &&
        (stub_simulation_distance_m <=
            (sensor_reaches_a_distance + 0.050f)))
    {
        stub_gray_mask = 0xFFU;
        stub_gray_black_count = 8U;
        stub_gray_error = 0.0f;
    }
    else
    {
        stub_gray_mask = 0x18U;
        stub_gray_black_count = 2U;
        /*
         * 赛道从 A 出发依次为 1.5 m 直线、半径 0.5 m 半圆、
         * 1.5 m 直线和第二个半圆。左右半圆仍给相反误差符号，
         * 用来验证循迹误差不会改变任务的恒定巡航速度。
         */
        lap_position = stub_simulation_distance_m;
        while(lap_position >= CAR_LAP_NOMINAL_DISTANCE_M)
        {
            lap_position -= CAR_LAP_NOMINAL_DISTANCE_M;
        }
        if((lap_position >= 1.500f) &&
            (lap_position < (1.500f + 1.5708f)))
        {
            stub_gray_error = 0.50f;
        }
        else if(lap_position >= (3.000f + 1.5708f))
        {
            stub_gray_error = -0.50f;
        }
        else
        {
            stub_gray_error = 0.0f;
        }
    }
    stub_gray_valid = 1U;

    chassis_update_5ms();
    state = chassis_get_state();
    if((CHASSIS_MODE_RUNNING != state->mode) &&
        (CHASSIS_MODE_PRE_BRAKE != state->mode))
    {
        return;
    }

    body_speed = (stub_control_state.left_speed_mps +
        stub_control_state.right_speed_mps) * 0.5f;
    if(body_speed < 0.0f)
    {
        body_speed = 0.0f;
    }
    stub_simulation_distance_m += body_speed * CAR_CONTROL_DT_S;
    count = test_distance_to_count(stub_simulation_distance_m);
    stub_encoder_total[CAR_ENCODER_LEFT] = count;
    stub_encoder_total[CAR_ENCODER_RIGHT] = count;
}

static uint32 test_run_simulated_mission(uint8 enable_a_line,
    uint32 maximum_ticks)
{
    uint32 tick;

    stub_simulation_enabled = 1U;
    stub_simulation_distance_m = 0.0f;
    for(tick = 0U; tick < maximum_ticks; tick++)
    {
        test_simulation_tick(enable_a_line);
        if(CHASSIS_MODE_FINISHED == chassis_get_state()->mode)
        {
            break;
        }
        TEST_ASSERT("mission simulation no fault",
            CHASSIS_MODE_FAULT != chassis_get_state()->mode);
    }
    return tick;
}

static void test_task2_complete_mission_simulation(void)
{
    uint32 tick;

    test_new_chassis();
    TEST_ASSERT("task2 simulation start", chassis_start());
    tick = test_run_simulated_mission(1U,
        (20000U / CAR_CONTROL_PERIOD_MS));
    TEST_ASSERT("task2 simulation finishes", tick <
        (20000U / CAR_CONTROL_PERIOD_MS));
    TEST_ASSERT("task2 result within 20 seconds",
        chassis_get_state()->lap_time_ms <= 20000U);
    TEST_ASSERT("task2 stops at A within internal tolerance",
        fabsf(chassis_get_state()->distance_m -
            CAR_LAP_NOMINAL_DISTANCE_M) <=
            CAR_FINISH_DISTANCE_TOLERANCE_M);
    printf("[INFO] task2 ideal result=%lums stop=%.3fm\n",
        (unsigned long)chassis_get_state()->lap_time_ms,
        chassis_get_state()->distance_m);
    test_pass("task 2 complete ideal mission within 20 seconds");
}

static void test_task4_complete_mission_simulation(void)
{
    uint32 tick;

    test_new_chassis();
    TEST_ASSERT("task4 simulation select",
        chassis_select_task(CHASSIS_TASK_4_A_TO_B));
    TEST_ASSERT("task4 simulation start", chassis_start());
    tick = test_run_simulated_mission(0U,
        (8000U / CAR_CONTROL_PERIOD_MS) + 1000U);
    if(tick >= ((8000U / CAR_CONTROL_PERIOD_MS) + 1000U))
    {
        fprintf(stderr,
            "[DIAG] task4 mode=%d distance=%.6f speed=%.6f ref=%.6f "
            "result=%lu\n",
            (int)chassis_get_state()->mode,
            chassis_get_state()->distance_m,
            chassis_get_state()->body_speed_mps,
            chassis_get_state()->base_speed_ref_mps,
            (unsigned long)chassis_get_state()->lap_time_ms);
    }
    TEST_ASSERT("task4 simulation finishes", tick <
        ((8000U / CAR_CONTROL_PERIOD_MS) + 1000U));
    TEST_ASSERT("task4 result within 8 seconds",
        chassis_get_state()->lap_time_ms <= 8000U);
    TEST_ASSERT("task4 configured for one-meter post-B travel",
        fabsf(CAR_TASK4_STOP_DISTANCE_M -
            (CAR_TASK4_B_DISTANCE_M +
                CAR_POST_RESULT_DISTANCE_M)) < 0.000001f);
    TEST_ASSERT("task4 reaches post-B stop target",
        chassis_get_state()->distance_m >= CAR_TASK4_STOP_DISTANCE_M);
    TEST_ASSERT("task4 post-B overshoot stays within internal tolerance",
        chassis_get_state()->distance_m <=
            (CAR_TASK4_STOP_DISTANCE_M +
                CAR_FINISH_DISTANCE_TOLERANCE_M));
    printf("[INFO] task4 ideal result=%lums stop=%.3fm\n",
        (unsigned long)chassis_get_state()->lap_time_ms,
        chassis_get_state()->distance_m);
    test_pass("task 4 complete ideal mission passes B within 8 seconds");
}

static void test_lap_complete_mission_simulation(ChassisTask task,
    const char *name)
{
    uint32 tick;

    test_new_chassis();
    TEST_ASSERT(name, chassis_select_task(task));
    TEST_ASSERT(name, chassis_start());
    tick = test_run_simulated_mission(1U,
        (30000U / CAR_CONTROL_PERIOD_MS) + 1000U);
    TEST_ASSERT(name, tick <
        ((30000U / CAR_CONTROL_PERIOD_MS) + 1000U));
    TEST_ASSERT(name, chassis_get_state()->lap_time_ms <= 30000U);
    TEST_ASSERT(name, chassis_get_state()->distance_m >=
        (CAR_LAP_NOMINAL_DISTANCE_M + CAR_PASS_A_EXTRA_DISTANCE_M -
        CAR_FINISH_DISTANCE_TOLERANCE_M));
    printf("[INFO] task%u ideal result=%lums stop=%.3fm\n",
        (unsigned int)task,
        (unsigned long)chassis_get_state()->lap_time_ms,
        chassis_get_state()->distance_m);
    test_pass(name);
}

static void test_task3_stationary_stop(void)
{
    test_new_chassis();
    TEST_ASSERT("task3 select",
        chassis_select_task(CHASSIS_TASK_3_STATIONARY));
    TEST_ASSERT("task3 start", chassis_start());
    test_tick(20U);
    TEST_ASSERT("task3 remains running", CHASSIS_MODE_RUNNING ==
        chassis_get_state()->mode);
    TEST_ASSERT("task3 left output zero", 0 ==
        stub_motor_output[CAR_MOTOR_LEFT]);
    TEST_ASSERT("task3 right output zero", 0 ==
        stub_motor_output[CAR_MOTOR_RIGHT]);

    chassis_stop();
    test_tick(1U);
    TEST_ASSERT("task3 finishes on stop", CHASSIS_MODE_FINISHED ==
        chassis_get_state()->mode);
    TEST_ASSERT("task3 captures finish time",
        chassis_get_state()->lap_time_ms ==
        chassis_get_state()->run_time_ms);
    test_pass("task 3 stationary and explicit finish");
}

static void test_velocity_closed_loop_mode(void)
{
    test_new_chassis();
    stub_gray_valid = 0U;
    stub_control_pwm_left = 1000;
    stub_control_pwm_right = 800;

    TEST_ASSERT("velocity rejects short duration",
        !chassis_start_velocity_test(0.20f, 0.10f,
            CAR_VELOCITY_TEST_MIN_MS - 1U));
    TEST_ASSERT("velocity rejects excessive target",
        !chassis_start_velocity_test(
            CAR_VELOCITY_TEST_MAX_SPEED_MPS + 0.01f, 0.10f, 500U));
    TEST_ASSERT("velocity test starts",
        chassis_start_velocity_test(0.20f, 0.10f,
            CAR_VELOCITY_TEST_MIN_MS));

    test_tick(1U);
    TEST_ASSERT("velocity test remains running",
        CHASSIS_MODE_RUNNING == chassis_get_state()->mode);
    TEST_ASSERT("velocity path called once", 1U == stub_velocity_update_calls);
    TEST_ASSERT("velocity left reference direct",
        fabsf(chassis_get_state()->left_speed_ref_mps - 0.20f) < 0.000001f);
    TEST_ASSERT("velocity right reference direct",
        fabsf(chassis_get_state()->right_speed_ref_mps - 0.10f) < 0.000001f);
    TEST_ASSERT("velocity ignores missing line",
        CHASSIS_FAULT_NONE == chassis_get_state()->fault_bits);

    test_tick((CAR_VELOCITY_TEST_MIN_MS / CAR_CONTROL_PERIOD_MS) - 1U);
    TEST_ASSERT("velocity duration finishes automatically",
        CHASSIS_MODE_FINISHED == chassis_get_state()->mode);
    TEST_ASSERT("velocity test does not capture race result",
        0U == chassis_get_state()->lap_time_ms);
    TEST_ASSERT("velocity finish stops both motors",
        (0 == stub_motor_output[CAR_MOTOR_LEFT]) &&
        (0 == stub_motor_output[CAR_MOTOR_RIGHT]));

    TEST_ASSERT("velocity test reset", chassis_reset());
    stub_control_pwm_left = CAR_MOTOR_STALL_PWM_THRESHOLD + 100;
    stub_control_pwm_right = CAR_MOTOR_STALL_PWM_THRESHOLD + 100;
    TEST_ASSERT("velocity high-output start",
        chassis_start_velocity_test(0.20f, 0.20f,
            CAR_VELOCITY_TEST_MIN_MS));
    /*
     * 首帧才施加 PWM；之后实际高输出连续三个完整周期无边沿，
     * 下一次安全采样才应锁定，不能在 VEL 的第 15 ms 提前误报。
     */
    test_tick(CAR_STALL_NO_EDGE_TICKS);
    TEST_ASSERT("velocity stall not counted before PWM applies",
        CHASSIS_MODE_FAULT != chassis_get_state()->mode);
    test_tick(1U);
    TEST_ASSERT("velocity stall after actual no-edge interval",
        CHASSIS_MODE_FAULT == chassis_get_state()->mode);
    TEST_ASSERT("velocity dual stall bits",
        (0U != (chassis_get_state()->fault_bits &
            CHASSIS_FAULT_LEFT_STALL)) &&
        (0U != (chassis_get_state()->fault_bits &
            CHASSIS_FAULT_RIGHT_STALL)));
    test_pass("velocity-only closed loop bypasses line control");
}

static void test_yaw_closed_loop_mode(void)
{
    test_new_chassis();
    stub_gray_valid = 0U;
    stub_imu_rate = 8.0f;
    stub_control_pwm_left = 900;
    stub_control_pwm_right = 1100;

    TEST_ASSERT("yaw rejects low base speed",
        !chassis_start_yaw_test(CAR_YAW_TEST_BASE_MIN_MPS - 0.01f,
            20.0f, 500U));
    TEST_ASSERT("yaw rejects excessive rate",
        !chassis_start_yaw_test(0.20f,
            CAR_YAW_TEST_MAX_RATE_DPS + 1.0f, 500U));
    TEST_ASSERT("yaw rejects short duration",
        !chassis_start_yaw_test(0.20f, 20.0f,
            CAR_YAW_TEST_MIN_MS - 1U));
    TEST_ASSERT("yaw test starts",
        chassis_start_yaw_test(0.20f, 20.0f, CAR_YAW_TEST_MIN_MS));

    test_tick(1U);
    TEST_ASSERT("yaw test remains running",
        CHASSIS_MODE_RUNNING == chassis_get_state()->mode);
    TEST_ASSERT("yaw path called once", 1U == stub_yaw_update_calls);
    TEST_ASSERT("yaw positive target makes right reference faster",
        chassis_get_state()->right_speed_ref_mps >
        chassis_get_state()->left_speed_ref_mps);
    TEST_ASSERT("yaw ignores missing line",
        CHASSIS_FAULT_NONE == chassis_get_state()->fault_bits);

    test_tick((CAR_YAW_TEST_MIN_MS / CAR_CONTROL_PERIOD_MS) - 1U);
    TEST_ASSERT("yaw duration finishes automatically",
        CHASSIS_MODE_FINISHED == chassis_get_state()->mode);
    TEST_ASSERT("yaw test does not capture race result",
        0U == chassis_get_state()->lap_time_ms);
    TEST_ASSERT("yaw finish stops both motors",
        (0 == stub_motor_output[CAR_MOTOR_LEFT]) &&
        (0 == stub_motor_output[CAR_MOTOR_RIGHT]));
    test_pass("yaw-rate closed loop bypasses line control");
}

static void test_task4_pass_b_then_stop(void)
{
    uint32 time_at_b;

    test_new_chassis();
    TEST_ASSERT("task4 select",
        chassis_select_task(CHASSIS_TASK_4_A_TO_B));
    TEST_ASSERT("task4 start", chassis_start());

    test_set_distance(CAR_TASK4_PRE_BRAKE_DISTANCE_M + 0.01f);
    test_tick(1U);
    TEST_ASSERT("task4 enters pre-brake", CHASSIS_MODE_PRE_BRAKE ==
        chassis_get_state()->mode);

    /* 加两个编码器计数，跨过浮点换算后的 B 点量化边界。 */
    test_set_distance(CAR_TASK4_B_DISTANCE_M +
        (2.0f * CAR_METERS_PER_COUNT));
    test_tick(1U);
    time_at_b = chassis_get_state()->lap_time_ms;
    TEST_ASSERT("task4 captures time at B", time_at_b > 0U);
    TEST_ASSERT("task4 keeps moving after B",
        CHASSIS_MODE_PRE_BRAKE ==
        chassis_get_state()->mode);
    test_tick(9U);
    TEST_ASSERT("task4 result time stays at B", time_at_b ==
        chassis_get_state()->lap_time_ms);
    TEST_ASSERT("task4 stop target is one meter after B",
        fabsf(CAR_TASK4_STOP_DISTANCE_M -
            (CAR_TASK4_B_DISTANCE_M +
                CAR_POST_RESULT_DISTANCE_M)) < 0.000001f);

    test_set_distance(CAR_TASK4_STOP_DISTANCE_M +
        (2.0f * CAR_METERS_PER_COUNT));
    test_tick(1U);
    TEST_ASSERT("task4 finishes one meter after B",
        CHASSIS_MODE_FINISHED == chassis_get_state()->mode);
    TEST_ASSERT("task4 result remains frozen after post-B travel",
        time_at_b == chassis_get_state()->lap_time_ms);
    test_pass("task 4 freezes result at B and stops one meter later");
}

static void test_prepare_a_line_detection(ChassisTask task)
{
    car_a_log_info_struct a_log_info;

    test_new_chassis();
    TEST_ASSERT("lap task select", chassis_select_task(task));
    TEST_ASSERT("lap task start", chassis_start());
    test_set_distance(CAR_A_GATE_DISTANCE_M + 0.10f);

    /*
     * 先跑满 10 s，但仅保持普通双点黑线，避免提前触发宽停车线。
     */
    stub_gray_black_count = 2U;
    stub_gray_mask = 0x18U;
    test_tick((CAR_A_GATE_TIME_MS / CAR_CONTROL_PERIOD_MS) - 1U);
    a_log_info = car_telemetry_get_a_log_info();
    TEST_ASSERT("A log stays empty before time gate",
        0U == a_log_info.sample_count);
    test_tick(1U);
    a_log_info = car_telemetry_get_a_log_info();
    TEST_ASSERT("A log starts when both gates open",
        1U == a_log_info.sample_count);

    /*
     * 实车普通弯道曾连续出现 black=3；即使时间和里程门已打开，
     * 也不能把三探头压线误判为 A 点。
     */
    stub_gray_black_count = CAR_GRAY_STOP_MIN_BLACK - 1U;
    stub_gray_mask = 0x70U;
    test_tick(CAR_GRAY_STOP_CONFIRM_FRAMES + 2U);
    test_set_distance(chassis_get_state()->distance_m +
        CAR_SENSOR_TO_TEST_POINT_M + CAR_FINISH_DISTANCE_TOLERANCE_M);
    test_tick(1U);
    TEST_ASSERT("black below A threshold does not stop",
        CHASSIS_MODE_FINISHED != chassis_get_state()->mode);

    stub_gray_black_count = CAR_GRAY_STOP_MIN_BLACK;
    stub_gray_mask = 0x78U;
    test_tick(CAR_GRAY_STOP_CONFIRM_FRAMES);
    a_log_info = car_telemetry_get_a_log_info();
    TEST_ASSERT("A log black candidate triggers capture",
        1U == a_log_info.triggered);
}

static void test_task2_a_line_latch_and_stop(void)
{
    float detected_distance;
    car_a_log_info_struct a_log_info;

    test_prepare_a_line_detection(CHASSIS_TASK_2_LAP_STOP_A);
    TEST_ASSERT("task2 pre-brake after A", CHASSIS_MODE_PRE_BRAKE ==
        chassis_get_state()->mode);
    detected_distance = chassis_get_state()->distance_m;

    /* 宽黑线持续存在，停车目标必须只锁存一次，不能每三帧向前漂移。 */
    test_tick(9U);
    test_set_distance(detected_distance + CAR_SENSOR_TO_TEST_POINT_M +
        CAR_FINISH_DISTANCE_TOLERANCE_M);
    test_tick(1U);
    TEST_ASSERT("task2 finishes at latched target",
        CHASSIS_MODE_FINISHED == chassis_get_state()->mode);
    TEST_ASSERT("task2 captures stop time",
        chassis_get_state()->lap_time_ms ==
        chassis_get_state()->run_time_ms);
    a_log_info = car_telemetry_get_a_log_info();
    TEST_ASSERT("task2 final stop freezes A log", 1U ==
        a_log_info.frozen);
    test_pass("task 2 A-line one-shot latch and stop");
}

static void test_lap_task(ChassisTask task, const char *name)
{
    float detected_distance;
    uint32 captured_lap_time;

    test_prepare_a_line_detection(task);
    TEST_ASSERT(name, CHASSIS_MODE_PRE_BRAKE ==
        chassis_get_state()->mode);
    detected_distance = chassis_get_state()->distance_m;
    TEST_ASSERT(name, 0U == chassis_get_state()->lap_time_ms);

    /* 前置灰度过线不等于车体测试点过线，计时不得提前锁存。 */
    test_set_distance(detected_distance + CAR_SENSOR_TO_TEST_POINT_M -
        (2.0f * CAR_METERS_PER_COUNT));
    test_tick(1U);
    TEST_ASSERT(name, 0U == chassis_get_state()->lap_time_ms);

    test_set_distance(detected_distance + CAR_SENSOR_TO_TEST_POINT_M);
    test_tick(1U);
    captured_lap_time = chassis_get_state()->lap_time_ms;
    TEST_ASSERT(name, captured_lap_time >= CAR_A_GATE_TIME_MS);

    test_set_distance(detected_distance + CAR_SENSOR_TO_TEST_POINT_M +
        (CAR_POST_RESULT_DISTANCE_M * 0.5f));
    test_tick(1U);
    TEST_ASSERT(name, CHASSIS_MODE_FINISHED !=
        chassis_get_state()->mode);
    TEST_ASSERT(name, captured_lap_time ==
        chassis_get_state()->lap_time_ms);
    test_set_distance(detected_distance + CAR_SENSOR_TO_TEST_POINT_M +
        CAR_PASS_A_EXTRA_DISTANCE_M +
        CAR_FINISH_DISTANCE_TOLERANCE_M);
    test_tick(1U);
    TEST_ASSERT(name, CHASSIS_MODE_FINISHED ==
        chassis_get_state()->mode);
    test_pass(name);
}

static void test_line_lost_fault(void)
{
    test_new_chassis();
    TEST_ASSERT("line fault start", chassis_start());
    stub_gray_valid = 0U;
    stub_gray_mask = 0U;
    stub_gray_black_count = 0U;

    test_tick(CAR_LINE_FAULT_TICKS - 1U);
    TEST_ASSERT("line fault not early", CHASSIS_MODE_FAULT !=
        chassis_get_state()->mode);
    test_tick(1U);
    TEST_ASSERT("line fault at threshold", CHASSIS_MODE_FAULT ==
        chassis_get_state()->mode);
    TEST_ASSERT("line fault bit", 0U !=
        (chassis_get_state()->fault_bits & CHASSIS_FAULT_LINE_LOST));
    test_pass("line loss threshold and fault latch");
}

static void test_stall_fault(void)
{
    test_new_chassis();
    stub_control_pwm_left = CAR_MOTOR_STALL_PWM_THRESHOLD + 100;
    stub_control_pwm_right = 0;
    TEST_ASSERT("stall start", chassis_start());

    /* 首帧才写入 PWM，随后连续三个周期无编码器边沿。 */
    test_tick(CAR_STALL_NO_EDGE_TICKS);
    TEST_ASSERT("stall not early", CHASSIS_MODE_FAULT !=
        chassis_get_state()->mode);
    test_tick(1U);
    TEST_ASSERT("left stall fault", CHASSIS_MODE_FAULT ==
        chassis_get_state()->mode);
    TEST_ASSERT("left stall bit", 0U !=
        (chassis_get_state()->fault_bits & CHASSIS_FAULT_LEFT_STALL));
    TEST_ASSERT("stall stops both motors",
        (0 == stub_motor_output[CAR_MOTOR_LEFT]) &&
        (0 == stub_motor_output[CAR_MOTOR_RIGHT]));

    TEST_ASSERT("motor stall reset", chassis_reset());
    TEST_ASSERT("high-output motor test starts",
        chassis_start_motor_test(
            CAR_MOTOR_STALL_PWM_THRESHOLD + 100, 0, 500U));
    test_tick(CAR_STALL_NO_EDGE_TICKS);
    TEST_ASSERT("motor test does not count before PWM applies",
        CHASSIS_MODE_FAULT != chassis_get_state()->mode);
    test_tick(1U);
    TEST_ASSERT("motor test stall after actual no-edge interval",
        CHASSIS_MODE_FAULT == chassis_get_state()->mode);
    TEST_ASSERT("motor test left stall bit", 0U !=
        (chassis_get_state()->fault_bits & CHASSIS_FAULT_LEFT_STALL));
    test_pass("15 ms no-edge stall protection");
}

static void test_encoder_signal_fault(void)
{
    test_new_chassis();
    TEST_ASSERT("encoder fault start", chassis_start());
    stub_encoder_invalid[CAR_ENCODER_RIGHT] =
        CAR_ENCODER_INVALID_LIMIT + 1U;
    test_tick(1U);
    TEST_ASSERT("encoder fault mode", CHASSIS_MODE_FAULT ==
        chassis_get_state()->mode);
    TEST_ASSERT("encoder fault bit", 0U !=
        (chassis_get_state()->fault_bits &
        CHASSIS_FAULT_ENCODER_SIGNAL));
    test_pass("encoder invalid-transition protection");
}

static void test_emergency_and_manual_reset(void)
{
    car_a_log_info_struct a_log_info;

    test_new_chassis();
    TEST_ASSERT("emergency start", chassis_start());
    chassis_emergency_stop(CHASSIS_FAULT_EMERGENCY_KEY);
    TEST_ASSERT("emergency fault mode", CHASSIS_MODE_FAULT ==
        chassis_get_state()->mode);
    TEST_ASSERT("emergency fault bit", 0U !=
        (chassis_get_state()->fault_bits &
        CHASSIS_FAULT_EMERGENCY_KEY));
    a_log_info = car_telemetry_get_a_log_info();
    TEST_ASSERT("emergency stop freezes A log", 1U ==
        a_log_info.frozen);
    TEST_ASSERT("manual reset accepted", chassis_reset());
    TEST_ASSERT("manual reset ready", CHASSIS_MODE_READY ==
        chassis_get_state()->mode);
    TEST_ASSERT("manual reset clears fault", CHASSIS_FAULT_NONE ==
        chassis_get_state()->fault_bits);
    a_log_info = car_telemetry_get_a_log_info();
    TEST_ASSERT("manual reset preserves frozen A log", 1U ==
        a_log_info.frozen);
    test_pass("emergency stop and manual reset");
}

static void test_constant_cruise_across_curve(void)
{
    test_new_chassis();
    TEST_ASSERT("constant cruise start", chassis_start());
    test_tick(400U);
    TEST_ASSERT("constant cruise reaches task2 speed",
        chassis_get_state()->base_speed_ref_mps >=
        (CAR_RACE_TASK2_SPEED_MPS - 0.001f));

    stub_gray_error = 0.50f;
    stub_imu_rate = 30.0f;
    test_tick(400U);
    TEST_ASSERT("curve does not change cruise speed",
        fabsf(chassis_get_state()->base_speed_ref_mps -
        CAR_RACE_TASK2_SPEED_MPS) < 0.001f);
    test_pass("constant cruise speed across straight and curve");
}

static float test_start_speed_for_task(ChassisTask task)
{
    test_new_chassis();
    TEST_ASSERT("select acceleration-profile task",
        chassis_select_task(task));
    TEST_ASSERT("start acceleration-profile task", chassis_start());
    test_tick(100U);
    return chassis_get_state()->base_speed_ref_mps;
}

static void test_task_specific_acceleration_profiles(void)
{
    float task2_speed =
        test_start_speed_for_task(CHASSIS_TASK_2_LAP_STOP_A);
    float task4_speed =
        test_start_speed_for_task(CHASSIS_TASK_4_A_TO_B);
    float task5_speed =
        test_start_speed_for_task(CHASSIS_TASK_5_LAP_CENTER);
    float task6_speed =
        test_start_speed_for_task(CHASSIS_TASK_6_LAP_POSITION);

    TEST_ASSERT("task 2 acceleration remains 0.30 profile",
        task2_speed > task4_speed + 0.040f);
    TEST_ASSERT("task 4 uses 0.15 acceleration profile",
        (task4_speed > 0.064f) && (task4_speed < 0.067f));
    TEST_ASSERT("task 5 matches task 4 acceleration profile",
        fabsf(task5_speed - task4_speed) < 0.0001f);
    TEST_ASSERT("task 6 matches task 4 acceleration profile",
        fabsf(task6_speed - task4_speed) < 0.0001f);
    TEST_ASSERT("task 4 rated speed is 0.26 mps",
        fabsf(CAR_RACE_TASK4_SPEED_MPS - 0.26f) < 0.0001f);
    test_pass("task-specific acceleration profiles");
}

int main(void)
{
    test_initial_state_and_selection();
    test_imu_readiness_gate_and_recovery();
    test_task2_complete_mission_simulation();
    test_task4_complete_mission_simulation();
    test_lap_complete_mission_simulation(CHASSIS_TASK_5_LAP_CENTER,
        "task 5 complete ideal lap within 30 seconds");
    test_lap_complete_mission_simulation(CHASSIS_TASK_6_LAP_POSITION,
        "task 6 complete ideal lap within 30 seconds");
    test_task3_stationary_stop();
    test_constant_cruise_across_curve();
    test_task_specific_acceleration_profiles();
    test_velocity_closed_loop_mode();
    test_yaw_closed_loop_mode();
    test_task4_pass_b_then_stop();
    test_task2_a_line_latch_and_stop();
    test_lap_task(CHASSIS_TASK_5_LAP_CENTER,
        "task 5 lap timing and pass-A stop");
    test_lap_task(CHASSIS_TASK_6_LAP_POSITION,
        "task 6 lap timing and pass-A stop");
    test_line_lost_fault();
    test_stall_fault();
    test_encoder_signal_fault();
    test_emergency_and_manual_reset();
    printf("[PASS] all %u chassis state-machine tests\n", test_count);
    return 0;
}

/* ------------------------- 正式硬件模块的测试桩 ------------------------- */

void car_control_init(void)
{
    memset(&stub_control_state, 0, sizeof(stub_control_state));
}

void car_control_reset(void)
{
    memset(&stub_control_state, 0, sizeof(stub_control_state));
    stub_motor_output[CAR_MOTOR_LEFT] = 0;
    stub_motor_output[CAR_MOTOR_RIGHT] = 0;
}

void car_control_update(float base_speed_mps, float line_error,
    uint8 line_valid, float dt_s)
{
    (void)line_error;
    (void)line_valid;
    (void)dt_s;
    stub_control_state.left_target_mps = base_speed_mps;
    stub_control_state.right_target_mps = base_speed_mps;
    if(stub_simulation_enabled)
    {
        stub_control_state.left_speed_mps = base_speed_mps;
        stub_control_state.right_speed_mps = base_speed_mps;
    }
    stub_control_state.left_output = stub_control_pwm_left;
    stub_control_state.right_output = stub_control_pwm_right;
    car_motor_set_pair(stub_control_pwm_left, stub_control_pwm_right);
}

void car_control_update_wheel_targets(float left_target_mps,
    float right_target_mps, float dt_s)
{
    (void)dt_s;
    stub_velocity_update_calls++;
    stub_control_state.left_target_mps = left_target_mps;
    stub_control_state.right_target_mps = right_target_mps;
    stub_control_state.left_output = stub_control_pwm_left;
    stub_control_state.right_output = stub_control_pwm_right;
    car_motor_set_pair(stub_control_pwm_left, stub_control_pwm_right);
}

void car_control_update_yaw_target(float base_speed_mps,
    float yaw_target_dps, float dt_s)
{
    float speed_delta = 0.02f;

    (void)dt_s;
    stub_yaw_update_calls++;
    if(yaw_target_dps < 0.0f)
    {
        speed_delta = -speed_delta;
    }
    stub_control_state.left_target_mps = base_speed_mps - speed_delta;
    stub_control_state.right_target_mps = base_speed_mps + speed_delta;
    stub_control_state.yaw_rate_target_dps = yaw_target_dps;
    stub_control_state.yaw_rate_feedback_dps = stub_imu_rate;
    stub_control_state.left_output = stub_control_pwm_left;
    stub_control_state.right_output = stub_control_pwm_right;
    car_motor_set_pair(stub_control_pwm_left, stub_control_pwm_right);
}

const car_control_state_struct *car_control_get_state(void)
{
    return &stub_control_state;
}

void car_control_set_speed_pid(float kp, float ki, float kd)
{
    (void)kp;
    (void)ki;
    (void)kd;
}

void car_control_set_line_pd(float kp, float kd)
{
    (void)kp;
    (void)kd;
}

void car_control_set_yaw_pid(float kp, float ki, float kd)
{
    (void)kp;
    (void)ki;
    (void)kd;
}

void car_encoder_init(void)
{
}

void car_encoder_sample(void)
{
}

int16 car_encoder_get_delta(car_encoder_id_enum encoder)
{
    return stub_encoder_delta[encoder];
}

int32 car_encoder_get_total(car_encoder_id_enum encoder)
{
    return stub_encoder_total[encoder];
}

uint32 car_encoder_get_invalid(car_encoder_id_enum encoder)
{
    return stub_encoder_invalid[encoder];
}

void car_encoder_reset(void)
{
    memset(stub_encoder_total, 0, sizeof(stub_encoder_total));
    memset(stub_encoder_delta, 0, sizeof(stub_encoder_delta));
    memset(stub_encoder_invalid, 0, sizeof(stub_encoder_invalid));
}

void car_gray_init(void)
{
}

uint8 car_gray_read(void)
{
    return stub_gray_mask;
}

uint8 car_gray_get_mask(void)
{
    return stub_gray_mask;
}

uint8 car_gray_get_error(float *error)
{
    if(stub_gray_valid && (NULL != error))
    {
        *error = stub_gray_error;
    }
    return stub_gray_valid;
}

uint8 car_gray_get_black_count(void)
{
    return stub_gray_black_count;
}

uint8 car_imu_init(void)
{
    stub_imu_ready = 1U;
    return 1U;
}

uint8 car_imu_calibrate(void)
{
    return stub_imu_ready;
}

uint8 car_imu_is_calibrating(void)
{
    return stub_imu_calibrating;
}

uint8 car_imu_calibration_step(void)
{
    if(!stub_imu_calibrating)
    {
        return 1U;
    }
    if(stub_imu_calibration_steps > 0U)
    {
        stub_imu_calibration_steps--;
    }
    if(0U == stub_imu_calibration_steps)
    {
        stub_imu_calibrating = 0U;
        stub_imu_ready = stub_imu_calibration_success;
        return 1U;
    }
    return 0U;
}

void car_imu_update(float dt_s)
{
    (void)dt_s;
}

uint8 car_imu_is_ready(void)
{
    return stub_imu_ready;
}

float car_imu_get_rate(void)
{
    return stub_imu_rate;
}

float car_imu_get_bias(void)
{
    return 0.0f;
}

float car_imu_get_residual(void)
{
    return 0.0f;
}

void car_motor_init(void)
{
}

void car_motor_set(car_motor_id_enum motor, int16 output)
{
    stub_motor_output[motor] = output;
}

void car_motor_set_pair(int16 left, int16 right)
{
    stub_motor_output[CAR_MOTOR_LEFT] = left;
    stub_motor_output[CAR_MOTOR_RIGHT] = right;
}

void car_motor_stop(void)
{
    stub_motor_output[CAR_MOTOR_LEFT] = 0;
    stub_motor_output[CAR_MOTOR_RIGHT] = 0;
}

int16 car_motor_get_output(car_motor_id_enum motor)
{
    return stub_motor_output[motor];
}

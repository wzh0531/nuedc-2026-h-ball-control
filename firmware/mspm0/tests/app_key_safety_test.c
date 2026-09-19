#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "car_app.h"
#include "car_chassis.h"
#include "car_config.h"
#include "car_encoder.h"
#include "car_keys.h"

/*
 * 该测试直接链接正式 car_app.c，以桩函数驱动 KEY4 输入。
 * 重点验证：
 * 1. 运行中消抖后的按下电平立即进入急停，不等待按键松开；
 * 2. 同一次持续按压产生 LONG 事件时，不会把刚进入的 FAULT 复位；
 * 3. 松开后重新长按，才允许人工复位。
 */
static ChassisState stub_state;
static uint8 stub_pending_ticks;
static car_key_event_enum stub_key3_event;
static uint8 stub_key4_pressed;
static car_key_event_enum stub_key4_event;
static uint32 stub_start_count;
static uint32 stub_stop_count;
static uint32 stub_task3_start_count;
static uint8 stub_task3_complete;
static uint32 stub_emergency_count;
static uint32 stub_reset_count;
static uint32 stub_update_count;
static uint32 stub_display_flush_count;
static uint32 stub_display_time_flush_count;
static uint32 stub_time_ms;
static uint8 stub_start_from_command;
static uint8 stub_task2_hold_zero;
static uint32 stub_ball_update_count;

static void test_fail(const char *expression, int line)
{
    fprintf(stderr, "[FAIL] app key safety line %d: %s\n",
        line, expression);
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

static void process_one_tick(void)
{
    stub_pending_ticks = 1U;
    car_app_process();
}

int main(void)
{
    uint32 start_count_before_task3;

    car_app_init();

    /* The app derives hold-zero directly from the authoritative task each tick. */
    stub_state.task = CHASSIS_TASK_2_LAP_STOP_A;
    process_one_tick();
    TEST_ASSERT(1U == stub_task2_hold_zero);
    stub_state.task = CHASSIS_TASK_3_STATIONARY;
    process_one_tick();
    TEST_ASSERT(0U == stub_task2_hold_zero);
    stub_state.task = CHASSIS_TASK_4_A_TO_B;
    process_one_tick();
    TEST_ASSERT(0U == stub_task2_hold_zero);
    stub_state.task = CHASSIS_TASK_5_LAP_CENTER;
    process_one_tick();
    TEST_ASSERT(0U == stub_task2_hold_zero);
    stub_state.task = CHASSIS_TASK_6_LAP_POSITION;
    process_one_tick();
    TEST_ASSERT(0U == stub_task2_hold_zero);
    stub_state.task = CHASSIS_TASK_2_LAP_STOP_A;
    process_one_tick();
    TEST_ASSERT(1U == stub_task2_hold_zero);
    TEST_ASSERT(6U == stub_ball_update_count);
    stub_update_count = 0U;
    stub_display_flush_count = 0U;
    stub_display_time_flush_count = 0U;

    /* READY 下短按 KEY3 必须直接进入 RUNNING。 */
    stub_key3_event = CAR_KEY_EVENT_SHORT;
    process_one_tick();
    TEST_ASSERT(1U == stub_start_count);
    TEST_ASSERT(CHASSIS_MODE_RUNNING == stub_state.mode);

    /* 运行中每 100 ms 只发送 TIME 页，不恢复全屏刷新。 */
    stub_state.run_time_ms = CAR_UI_REFRESH_PERIOD_MS;
    stub_time_ms = CAR_UI_REFRESH_PERIOD_MS;
    process_one_tick();
    TEST_ASSERT(1U == stub_display_time_flush_count);
    TEST_ASSERT(0U == stub_display_flush_count);
    stub_time_ms += 5U;
    process_one_tick();
    TEST_ASSERT(1U == stub_display_time_flush_count);

    stub_state.mode = CHASSIS_MODE_READY;
    stub_update_count = 0U;

    /* 停车调试日志造成的积压只保留一次最新采样，不得误锁超时故障。 */
    stub_pending_ticks = CAR_CONTROL_OVERRUN_LIMIT + 1U;
    car_app_process();
    TEST_ASSERT(0U == stub_emergency_count);
    TEST_ASSERT(CHASSIS_MODE_READY == stub_state.mode);
    TEST_ASSERT(1U == stub_update_count);
    TEST_ASSERT(0U == stub_display_flush_count);

    stub_pending_ticks = CAR_CONTROL_OVERRUN_LIMIT + 1U;
    stub_start_from_command = 1U;
    car_app_process();
    TEST_ASSERT(0U == stub_emergency_count);
    TEST_ASSERT(CHASSIS_MODE_RUNNING == stub_state.mode);
    TEST_ASSERT(2U == stub_update_count);
    TEST_ASSERT(0U == stub_display_flush_count);

    /* 同样的积压若发生在运行中，必须锁定控制超时。 */
    stub_pending_ticks = CAR_CONTROL_OVERRUN_LIMIT + 1U;
    car_app_process();
    TEST_ASSERT(1U == stub_emergency_count);
    TEST_ASSERT(CHASSIS_MODE_FAULT == stub_state.mode);

    /* Task 3 is stationary: discard stale ticks without a false 0x10 fault. */
    stub_emergency_count = 0U;
    stub_update_count = 0U;
    stub_state.fault_bits = 0U;
    stub_state.mode = CHASSIS_MODE_RUNNING;
    stub_state.task = CHASSIS_TASK_3_STATIONARY;
    stub_pending_ticks = CAR_CONTROL_OVERRUN_LIMIT + 1U;
    car_app_process();
    TEST_ASSERT(0U == stub_emergency_count);
    TEST_ASSERT(CHASSIS_MODE_RUNNING == stub_state.mode);
    TEST_ASSERT(1U == stub_update_count);

    /* 清理上述场景，继续验证独立的 KEY4 安全路径。 */
    stub_emergency_count = 0U;
    stub_state.fault_bits = 0U;
    stub_state.mode = CHASSIS_MODE_RUNNING;
    stub_state.task = CHASSIS_TASK_2_LAP_STOP_A;

    /* 已通过按键模块 10 ms 消抖的按下电平，必须在本周期急停。 */
    stub_key4_pressed = 1U;
    process_one_tick();
    TEST_ASSERT(1U == stub_emergency_count);
    TEST_ASSERT(CHASSIS_MODE_FAULT == stub_state.mode);

    /* 持续按住到 1 s 的 LONG 事件必须被本次急停按压锁吞掉。 */
    stub_key4_event = CAR_KEY_EVENT_LONG;
    process_one_tick();
    TEST_ASSERT(0U == stub_reset_count);
    TEST_ASSERT(CHASSIS_MODE_FAULT == stub_state.mode);

    /* 先松开以解除急停按压锁。 */
    stub_key4_pressed = 0U;
    process_one_tick();
    TEST_ASSERT(0U == stub_reset_count);

    /* 排除故障后重新长按，才允许从 FAULT 人工复位。 */
    stub_key4_pressed = 1U;
    stub_key4_event = CAR_KEY_EVENT_LONG;
    process_one_tick();
    TEST_ASSERT(1U == stub_reset_count);
    TEST_ASSERT(CHASSIS_MODE_READY == stub_state.mode);

    /* Task 3 starts from KEY3 and stops once on tracker completion. */
    stub_key4_pressed = 0U;
    stub_state.task = CHASSIS_TASK_3_STATIONARY;
    stub_state.mode = CHASSIS_MODE_READY;
    start_count_before_task3 = stub_start_count;
    stub_key3_event = CAR_KEY_EVENT_SHORT;
    process_one_tick();
    TEST_ASSERT(start_count_before_task3 + 1U == stub_start_count);
    TEST_ASSERT(1U == stub_task3_start_count);
    TEST_ASSERT(CHASSIS_MODE_RUNNING == stub_state.mode);

    stub_task3_complete = 1U;
    process_one_tick();
    TEST_ASSERT(1U == stub_stop_count);
    TEST_ASSERT(CHASSIS_MODE_FINISHED == stub_state.mode);
    process_one_tick();
    TEST_ASSERT(1U == stub_stop_count);

    printf("[PASS] parked backlog gate, runtime timeout and KEY4 safety\n");
    return 0;
}

/* ----------------------------- App dependency stubs ----------------------------- */

void car_command_init(void)
{
}

void car_command_process(void)
{
    if(stub_start_from_command)
    {
        stub_start_from_command = 0U;
        stub_state.mode = CHASSIS_MODE_RUNNING;
    }
}

void car_motor_init(void)
{
}

void car_encoder_init(void)
{
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

void car_gray_init(void)
{
}

uint8 car_gray_get_mask(void)
{
    return 0x18U;
}

void car_keys_init(void)
{
}

void car_keys_scan_5ms(void)
{
}

uint8 car_keys_is_pressed(car_key_id_enum key)
{
    return (CAR_KEY_4 == key) ? stub_key4_pressed : 0U;
}

car_key_event_enum car_keys_take_event(car_key_id_enum key)
{
    car_key_event_enum event = CAR_KEY_EVENT_NONE;

    if(CAR_KEY_3 == key)
    {
        event = stub_key3_event;
        stub_key3_event = CAR_KEY_EVENT_NONE;
    }
    else if(CAR_KEY_4 == key)
    {
        event = stub_key4_event;
        stub_key4_event = CAR_KEY_EVENT_NONE;
    }
    return event;
}

void car_display_init(void)
{
}

void car_display_update(uint8 task, uint8 mode, uint32 runtime_ms,
    uint32 lap_time_ms, float distance_m, float left_speed_mps,
    float right_speed_mps, uint8 gray_mask, float line_error,
    int16 left_pwm, int16 right_pwm, uint32 fault_bits)
{
    (void)task;
    (void)mode;
    (void)runtime_ms;
    (void)lap_time_ms;
    (void)distance_m;
    (void)left_speed_mps;
    (void)right_speed_mps;
    (void)gray_mask;
    (void)line_error;
    (void)left_pwm;
    (void)right_pwm;
    (void)fault_bits;
}

void car_display_flush_step(void)
{
    stub_display_flush_count++;
}

void car_display_flush_time_page(void)
{
    stub_display_time_flush_count++;
}

uint8 car_imu_init(void)
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

void car_scheduler_init(void)
{
}

uint8 car_scheduler_take_control_ticks(void)
{
    uint8 ticks = stub_pending_ticks;

    stub_pending_ticks = 0U;
    return ticks;
}

uint32 car_scheduler_get_time_ms(void)
{
    return stub_time_ms;
}

void chassis_init(void)
{
    memset(&stub_state, 0, sizeof(stub_state));
    stub_state.mode = CHASSIS_MODE_READY;
    stub_state.task = CHASSIS_TASK_2_LAP_STOP_A;
}

uint8 chassis_select_task(ChassisTask task)
{
    stub_state.task = task;
    return 1U;
}

uint8 chassis_start(void)
{
    stub_start_count++;
    stub_state.mode = CHASSIS_MODE_RUNNING;
    return 1U;
}

void chassis_stop(void)
{
    stub_stop_count++;
    stub_state.mode = CHASSIS_MODE_FINISHED;
}

void chassis_update_5ms(void)
{
    stub_update_count++;
}

const ChassisState *chassis_get_state(void)
{
    return &stub_state;
}

void chassis_emergency_stop(uint32 fault_bits)
{
    stub_emergency_count++;
    stub_state.fault_bits |= fault_bits;
    stub_state.mode = CHASSIS_MODE_FAULT;
}

uint8 chassis_reset(void)
{
    stub_reset_count++;
    stub_state.fault_bits = 0U;
    stub_state.mode = CHASSIS_MODE_READY;
    return 1U;
}

void car_ball_tracker_init(void)
{
}

void car_ball_tracker_update_5ms(uint8 task2_hold_zero,
    float vehicle_accel_ref_mps2)
{
    (void)vehicle_accel_ref_mps2;
    stub_task2_hold_zero = task2_hold_zero;
    stub_ball_update_count++;
}

void car_ball_tracker_set_target(float target_pixels)
{
    (void)target_pixels;
}

void car_ball_tracker_task3_start(void)
{
    stub_task3_start_count++;
}

void car_ball_tracker_task3_cancel(void)
{
}

uint8 car_ball_tracker_task3_take_complete(void)
{
    uint8 complete = stub_task3_complete;

    stub_task3_complete = 0U;
    return complete;
}

#include "car_app.h"

#include <stdio.h>

#include "car_ball_tracker.h"
#include "car_chassis.h"
#include "car_command.h"
#include "car_config.h"
#include "car_display.h"
#include "car_encoder.h"
#include "car_gray.h"
#include "car_imu.h"
#include "car_keys.h"
#include "car_motor.h"
#include "car_safety.h"
#include "car_scheduler.h"

static uint32 control_overrun_count;
static uint8 max_pending_control_ticks;
static uint32 previous_ui_refresh_ms;
/*
 * 运行中按下 KEY4 后立即急停，并锁住本次按压直到松手。
 * 否则持续按住 1 s 会产生 LONG 事件，可能把刚进入的 FAULT 又复位。
 */
static uint8 key4_emergency_hold;

static ChassisTask car_app_previous_task(ChassisTask task)
{
    return (task <= CHASSIS_TASK_2_LAP_STOP_A) ?
        CHASSIS_TASK_6_LAP_POSITION : (ChassisTask)((uint8)task - 1U);
}

static ChassisTask car_app_next_task(ChassisTask task)
{
    return (task >= CHASSIS_TASK_6_LAP_POSITION) ?
        CHASSIS_TASK_2_LAP_STOP_A : (ChassisTask)((uint8)task + 1U);
}

static void car_app_handle_keys(void)
{
    const ChassisState *state = chassis_get_state();
    car_key_event_enum key1 = car_keys_take_event(CAR_KEY_1);
    car_key_event_enum key2 = car_keys_take_event(CAR_KEY_2);
    car_key_event_enum key3 = car_keys_take_event(CAR_KEY_3);
    car_key_event_enum key4 = car_keys_take_event(CAR_KEY_4);
    uint8 key4_pressed = car_keys_is_pressed(CAR_KEY_4);

    /*
     * KEY4 不等待松手：连续低电平 10 ms 通过消抖后立即急停。
     * 这条安全路径优先于任务选择、启动和长按复位。
     */
    if(key4_pressed &&
        ((CHASSIS_MODE_RUNNING == state->mode) ||
        (CHASSIS_MODE_PRE_BRAKE == state->mode)))
    {
        key4_emergency_hold = 1U;
        chassis_emergency_stop(CHASSIS_FAULT_EMERGENCY_KEY);
        return;
    }
    if(key4_emergency_hold)
    {
        if(!key4_pressed)
        {
            key4_emergency_hold = 0U;
        }
        return;
    }

    if(CAR_KEY_EVENT_SHORT == key1)
    {
        if(chassis_select_task(car_app_previous_task(state->task)))
        {
            car_ball_tracker_set_target(CAR_BALL_TARGET_DEFAULT_PIXELS);
        }
    }
    if(CAR_KEY_EVENT_SHORT == key2)
    {
        if(chassis_select_task(car_app_next_task(state->task)))
        {
            car_ball_tracker_set_target(CAR_BALL_TARGET_DEFAULT_PIXELS);
        }
    }
    if(CAR_KEY_EVENT_SHORT == key3)
    {
        if((CHASSIS_MODE_READY == state->mode) ||
            (CHASSIS_MODE_FINISHED == state->mode))
        {
            if(chassis_start() &&
                (CHASSIS_TASK_3_STATIONARY == state->task))
            {
                car_ball_tracker_task3_start();
            }
        }
        else if((CHASSIS_MODE_RUNNING == state->mode) &&
            (CHASSIS_TASK_3_STATIONARY == state->task))
        {
            car_ball_tracker_task3_cancel();
            chassis_stop();
        }
    }
    if(CAR_KEY_EVENT_SHORT == key4)
    {
        chassis_emergency_stop(CHASSIS_FAULT_EMERGENCY_KEY);
    }
    if(CAR_KEY_EVENT_LONG == key4)
    {
        if((CHASSIS_MODE_FAULT == state->mode) ||
            (CHASSIS_MODE_FINISHED == state->mode))
        {
            (void)chassis_reset();
        }
        else
        {
            /* 长按期间也必须具备急停语义，不能等松手。 */
            chassis_emergency_stop(CHASSIS_FAULT_EMERGENCY_KEY);
        }
    }
}

static uint8 car_app_update_display(void)
{
    const ChassisState *state = chassis_get_state();
    uint32 now_ms = car_scheduler_get_time_ms();

    if((now_ms - previous_ui_refresh_ms) < CAR_UI_REFRESH_PERIOD_MS)
    {
        return 0U;
    }
    previous_ui_refresh_ms = now_ms;
    car_display_update((uint8)state->task, (uint8)state->mode,
        state->run_time_ms, state->lap_time_ms, state->distance_m,
        state->left_speed_mps, state->right_speed_mps,
        state->gray_mask, state->line_error, state->left_pwm,
        state->right_pwm, state->fault_bits);
    return 1U;
}

void car_app_init(void)
{
    uint8 imu_ok;

    control_overrun_count = 0U;
    max_pending_control_ticks = 0U;
    key4_emergency_hold = 0U;
    /* 让上电后的第一次显示刷新立即发生。 */
    previous_ui_refresh_ms = (uint32)(0U - CAR_UI_REFRESH_PERIOD_MS);
    car_motor_init();
    car_encoder_init();
    car_gray_init();
    car_keys_init();
    car_display_init();
    car_ball_tracker_init();
    imu_ok = car_imu_init();
    chassis_init();
    car_command_init();

    printf("\r\nH chassis firmware V1.0\r\n");
    printf("IMU=%s bias=%.3f residual=%.3f, gray=0x%02X\r\n",
        imu_ok ? "CALIBRATING" : "ERROR", car_imu_get_bias(),
        car_imu_get_residual(), (unsigned int)car_gray_get_mask());
    printf("KEY1/2 select, KEY3 start, KEY4 press stop.\r\n");
    printf("Release KEY4, then hold 1s to reset FAULT/FINISH.\r\n");
    printf("Type HELP for tuning commands.\r\n");
    printf("Ball tracker: BLDC UART1 PA8/PA9, vision UART2 PB16.\r\n");

    /*
     * 调度器必须在阻塞式欢迎日志之后启动。115200 串口打印约数百字节时
     * 足以积压多个 5 ms 标志，若先启动调度会在首次主循环误报控制超时。
     */
    car_scheduler_init();
    (void)car_app_update_display();
}

void car_app_process(void)
{
    const ChassisState *state;
    uint8 pending_ticks;
    uint8 index;
    uint8 was_running_before_command;
    uint8 display_flush_allowed;
    uint8 display_updated;

    state = chassis_get_state();
    was_running_before_command = (uint8)(
        (CHASSIS_MODE_RUNNING == state->mode) ||
        (CHASSIS_MODE_PRE_BRAKE == state->mode));
    car_command_process();
    state = chassis_get_state();
    pending_ticks = car_scheduler_take_control_ticks();
    if(pending_ticks == 0U)
    {
        return;
    }

    if(pending_ticks > max_pending_control_ticks)
    {
        max_pending_control_ticks = pending_ticks;
    }
    if(pending_ticks > 1U)
    {
        control_overrun_count += (uint32)pending_ticks - 1U;
    }
    display_flush_allowed = (uint8)(1U == pending_ticks);
    if(pending_ticks > CAR_CONTROL_OVERRUN_LIMIT)
    {
        /*
         * 只有车辆正在运动时，控制周期积压才需要立即锁故障。
         * READY/FINISHED/FAULT 下的 HELP、STATUS 等阻塞打印不会危及执行器，
         * 此时丢弃陈旧周期并只执行一次最新采样，避免停车调试误报。
         */
        if(was_running_before_command &&
            ((CHASSIS_MODE_RUNNING == state->mode) ||
            (CHASSIS_MODE_PRE_BRAKE == state->mode)) &&
            (CHASSIS_TASK_3_STATIONARY != state->task))
        {
            chassis_emergency_stop(CHASSIS_FAULT_CONTROL_OVERRUN);
        }
        pending_ticks = 1U;
    }

    for(index = 0U; index < pending_ticks; index++)
    {
        car_keys_scan_5ms();
        car_app_handle_keys();
        chassis_update_5ms();
        state = chassis_get_state();
        car_ball_tracker_update_5ms((uint8)(
            CHASSIS_TASK_2_LAP_STOP_A == state->task),
            (uint8)((CHASSIS_TASK_4_A_TO_B == state->task) ||
            (CHASSIS_TASK_5_LAP_CENTER == state->task) ||
            (CHASSIS_TASK_6_LAP_POSITION == state->task)) ?
            state->vehicle_accel_ref_mps2 : 0.0f);
        if(car_ball_tracker_task3_take_complete())
        {
            chassis_stop();
        }
        state = chassis_get_state();
        if(display_flush_allowed &&
            (CHASSIS_MODE_RUNNING != state->mode) &&
            (CHASSIS_MODE_PRE_BRAKE != state->mode))
        {
            car_display_flush_step();
        }
    }
    display_updated = car_app_update_display();
    state = chassis_get_state();
    if(display_updated && display_flush_allowed &&
        ((CHASSIS_MODE_RUNNING == state->mode) ||
        (CHASSIS_MODE_PRE_BRAKE == state->mode)))
    {
        car_display_flush_time_page();
    }
}

void car_app_clear_pending_ticks(void)
{
    (void)car_scheduler_take_control_ticks();
}

uint32 car_app_get_overrun_count(void)
{
    return control_overrun_count;
}

uint8 car_app_get_max_pending_ticks(void)
{
    return max_pending_control_ticks;
}

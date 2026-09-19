#include <stdio.h>
#include <stdlib.h>

#include "car_ball_tracker.h"
#include "car_bldc.h"
#include "car_config.h"
#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"
#include "zf_driver_uart.h"

static int32 last_angle_x10;
static uint32 motor_event_counter;
static uint32 single_zero_event;
static uint32 zero_wait_event;
static uint32 clear_multi_event;

static void test_fail(const char *expression, int line)
{
    fprintf(stderr, "[FAIL] ball task3 line %d: %s\n", line, expression);
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

static void feed_frame_mode(int16 dx, uint8 task2_hold_zero)
{
    car_ball_tracker_parse_vision_byte(0x7BU);
    car_ball_tracker_parse_vision_byte(0x01U);
    car_ball_tracker_parse_vision_byte((uint8)(((uint16)dx >> 8) & 0xFFU));
    car_ball_tracker_parse_vision_byte((uint8)((uint16)dx & 0xFFU));
    car_ball_tracker_parse_vision_byte(0x7DU);
    car_ball_tracker_update_5ms(task2_hold_zero, 0.0f);
}

static void feed_frame(int16 dx)
{
    feed_frame_mode(dx, 0U);
}

int main(void)
{
    uint8 index;
    int32 first_command_x10;

    car_ball_tracker_init();
    TEST_ASSERT(single_zero_event < zero_wait_event);
    TEST_ASSERT(zero_wait_event < clear_multi_event);
    TEST_ASSERT(0 == last_angle_x10);

    /* Task 2 consumes vision frames but never runs any ball PID. */
    feed_frame_mode(40, 1U);
    TEST_ASSERT(0 == car_ball_tracker_get_command_angle_x10());
    TEST_ASSERT(0 == last_angle_x10);
    feed_frame_mode(-80, 1U);
    TEST_ASSERT(0 == car_ball_tracker_get_command_angle_x10());
    TEST_ASSERT(0 == last_angle_x10);

    /* Tasks 4-6 use the unchanged generic PID when hold-zero is false. */
    feed_frame_mode(40, 0U);
    TEST_ASSERT(0 != car_ball_tracker_get_command_angle_x10());
    car_ball_tracker_update_5ms(1U, 0.0f);
    TEST_ASSERT(0 == car_ball_tracker_get_command_angle_x10());
    TEST_ASSERT(0 == last_angle_x10);

    /* Positive chassis start acceleration must lower the rod front. */
    car_ball_tracker_update_5ms(0U, 0.006f);
    TEST_ASSERT(-25 == car_ball_tracker_get_command_angle_x10());
    TEST_ASSERT(-25 == last_angle_x10);

    /* Braking acceleration uses the opposite compensation direction. */
    car_ball_tracker_update_5ms(1U, 0.0f);
    car_ball_tracker_update_5ms(0U, -0.006f);
    TEST_ASSERT(25 == car_ball_tracker_get_command_angle_x10());
    TEST_ASSERT(25 == last_angle_x10);
    car_ball_tracker_update_5ms(1U, 0.15f);
    TEST_ASSERT(0 == car_ball_tracker_get_command_angle_x10());
    TEST_ASSERT(0 == last_angle_x10);

    car_ball_tracker_task3_start();
    TEST_ASSERT(CAR_BALL_TASK3_TO_NEGATIVE ==
        car_ball_tracker_get_task3_phase());
    TEST_ASSERT(CAR_BALL_TASK3_NEGATIVE_TARGET_UNITS ==
        car_ball_tracker_get_target());

    /* Left travel uses its own default PID and fixed 5 ms dt. */
    feed_frame(0);
    first_command_x10 = car_ball_tracker_get_command_angle_x10();
    TEST_ASSERT(first_command_x10 <= -68);
    TEST_ASSERT(first_command_x10 >= -69);

    /* Empty ticks do not alter the fixed PID dt used by the next vision frame. */
    for(index = 0U; index < 7U; index++)
    {
        car_ball_tracker_update_5ms(0U, 0.0f);
    }
    feed_frame(-8);
    TEST_ASSERT(car_ball_tracker_get_command_angle_x10() >= 136);
    TEST_ASSERT(car_ball_tracker_get_command_angle_x10() <= 138);

    for(index = 0U; index < CAR_BALL_TASK3_LEFT_STABLE_FRAMES - 1U; index++)
    {
        feed_frame((int16)CAR_BALL_TASK3_NEGATIVE_TARGET_UNITS);
    }
    TEST_ASSERT(CAR_BALL_TASK3_TO_NEGATIVE ==
        car_ball_tracker_get_task3_phase());

    feed_frame((int16)CAR_BALL_TASK3_NEGATIVE_TARGET_UNITS);
    TEST_ASSERT(CAR_BALL_TASK3_TO_POSITIVE ==
        car_ball_tracker_get_task3_phase());
    TEST_ASSERT(CAR_BALL_TASK3_POSITIVE_TARGET_UNITS ==
        car_ball_tracker_get_target());

    /* Right travel switches to its own default PID after the target reset. */
    feed_frame((int16)CAR_BALL_TASK3_POSITIVE_TARGET_UNITS - 8);
    TEST_ASSERT(car_ball_tracker_get_command_angle_x10() >= 11);
    TEST_ASSERT(car_ball_tracker_get_command_angle_x10() <= 12);
    for(index = 0U; index < CAR_BALL_TASK3_RIGHT_STABLE_FRAMES - 1U; index++)
    {
        feed_frame((int16)(CAR_BALL_TASK3_POSITIVE_TARGET_UNITS -
            CAR_BALL_TASK3_TOLERANCE_UNITS));
    }
    TEST_ASSERT(CAR_BALL_TASK3_TO_POSITIVE ==
        car_ball_tracker_get_task3_phase());
    feed_frame((int16)(CAR_BALL_TASK3_POSITIVE_TARGET_UNITS -
        CAR_BALL_TASK3_TOLERANCE_UNITS));

    TEST_ASSERT(CAR_BALL_TASK3_COMPLETE ==
        car_ball_tracker_get_task3_phase());
    TEST_ASSERT(1U == car_ball_tracker_task3_take_complete());
    TEST_ASSERT(0U == car_ball_tracker_task3_take_complete());
    TEST_ASSERT(last_angle_x10 <= (int32)(CAR_BALL_ANGLE_LIMIT_DEG * 10.0f));
    TEST_ASSERT(last_angle_x10 >= (int32)(-CAR_BALL_ANGLE_LIMIT_DEG * 10.0f));

    printf("[PASS] task3 -5cm -> +5cm completion sequence\n");
    return 0;
}

void uart_write_buffer(uart_index_enum uart_index, const uint8 *buff,
    uint32 len)
{
    (void)uart_index;
    (void)buff;
    (void)len;
}

uint8 uart_query_byte(uart_index_enum uart_index, uint8 *data)
{
    (void)uart_index;
    (void)data;
    return 0U;
}

void uart_set_callback(uart_index_enum uart_index,
    host_uart_callback callback, void *ptr)
{
    (void)uart_index;
    (void)callback;
    (void)ptr;
}

void uart_set_interrupt_config(uart_index_enum uart_index,
    uart_interrupt_config_enum config)
{
    (void)uart_index;
    (void)config;
}

void uart_init(uart_index_enum uart_index, uint32 baud,
    uart_tx_pin_enum tx_pin, uart_rx_pin_enum rx_pin)
{
    (void)uart_index;
    (void)baud;
    (void)tx_pin;
    (void)rx_pin;
}

uint32 interrupt_global_disable(void)
{
    return 0U;
}

void interrupt_global_enable(uint32 state)
{
    (void)state;
}

void interrupt_set_priority(int irqn, uint8 priority)
{
    (void)irqn;
    (void)priority;
}

void system_delay_us(uint32 time_us)
{
    (void)time_us;
}

void system_delay_ms(uint32 time_ms)
{
    if(CAR_BALL_ZERO_SETTLE_MS == time_ms)
    {
        zero_wait_event = ++motor_event_counter;
    }
}

void car_bldc_set_send_function(car_bldc_send_function function)
{
    (void)function;
}

void car_bldc_parse_rx_byte(uint8 data)
{
    (void)data;
}

void car_bldc_disable(uint8 address)
{
    (void)address;
}

void car_bldc_enable(uint8 address)
{
    (void)address;
}

void car_bldc_set_mode(uint8 address, uint16 mode)
{
    (void)address;
    (void)mode;
}

void car_bldc_set_speed(uint8 address, int16 rpm)
{
    (void)address;
    (void)rpm;
}

void car_bldc_set_acceleration(uint8 address, uint16 acceleration_rps2)
{
    (void)address;
    (void)acceleration_rps2;
}

void car_bldc_set_multi_angle(uint8 address, int32 angle_x10)
{
    (void)address;
    last_angle_x10 = angle_x10;
}

void car_bldc_set_single_angle(uint8 address, uint16 angle_x10)
{
    (void)address;
    if(0U == angle_x10)
    {
        single_zero_event = ++motor_event_counter;
    }
}

void car_bldc_clear_multi_angle(uint8 address)
{
    (void)address;
    clear_multi_event = ++motor_event_counter;
}

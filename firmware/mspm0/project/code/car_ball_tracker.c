#include "car_ball_tracker.h"

#include <math.h>

#include "car_bldc.h"
#include "car_board.h"
#include "car_config.h"
#include "car_pid.h"
#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"
#include "zf_driver_uart.h"

#define CAR_VISION_FRAME_HEADER              (0x7BU)
#define CAR_VISION_FRAME_TAIL                (0x7DU)
#define CAR_VISION_PAYLOAD_LENGTH            (3U)

typedef enum
{
    CAR_VISION_RX_WAIT_HEADER = 0,
    CAR_VISION_RX_PAYLOAD,
    CAR_VISION_RX_WAIT_TAIL,
}car_vision_rx_state_enum;

static car_pid_struct ball_position_pid;
static car_pid_struct ball_task3_left_pid;
static car_pid_struct ball_task3_right_pid;
static volatile uint8 vision_tracking;
static volatile int16 vision_dx;
static volatile uint8 vision_data_ready;
static volatile uint32 vision_frame_count;
static car_vision_rx_state_enum vision_rx_state;
static uint8 vision_rx_buffer[CAR_VISION_PAYLOAD_LENGTH];
static uint8 vision_rx_index;
static float ball_target_pixels;
static float ball_pid_output_deg;
static float ball_previous_accel_mps2;
static int32 ball_command_angle_x10;
static uint8 ball_task2_hold_active;
static car_ball_task3_phase_enum ball_task3_phase;
static uint8 ball_task3_stable_frames;
static uint8 ball_task3_complete_pending;

static float car_ball_tracker_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float car_ball_tracker_clamp(float value, float limit)
{
    if(value > limit)
    {
        return limit;
    }
    if(value < -limit)
    {
        return -limit;
    }
    return value;
}

static float car_ball_tracker_accel_feedforward(float accel_mps2)
{
    float jerk_mps3;
    float preview_accel_mps2 = accel_mps2;
    float angle_deg;

    jerk_mps3 = (accel_mps2 - ball_previous_accel_mps2) /
        CAR_CONTROL_DT_S;
    jerk_mps3 = car_ball_tracker_clamp(jerk_mps3,
        CAR_JERK_LIMIT_MPS3);
    ball_previous_accel_mps2 = accel_mps2;

    /* Lead only while acceleration magnitude is building. */
    if((accel_mps2 * jerk_mps3) > 0.0f)
    {
        preview_accel_mps2 += jerk_mps3 * CAR_BALL_ACCEL_FF_PREVIEW_S;
    }
    preview_accel_mps2 = car_ball_tracker_clamp(preview_accel_mps2,
        CAR_BALL_ACCEL_FF_MAX_ACCEL_MPS2);
    angle_deg = CAR_BALL_ACCEL_FF_SIGN * CAR_BALL_ACCEL_FF_GAIN *
        atanf(preview_accel_mps2 / CAR_GRAVITY_MPS2) * CAR_RAD_TO_DEG;
    return car_ball_tracker_clamp(angle_deg, CAR_BALL_ACCEL_FF_MAX_DEG);
}

static void car_ball_tracker_apply_target(float target)
{
    ball_target_pixels = target;
    car_pid_reset(&ball_position_pid);
    car_pid_reset(&ball_task3_left_pid);
    car_pid_reset(&ball_task3_right_pid);
}

static void car_ball_tracker_update_task3(int16 dx)
{
    float expected_target;
    uint8 required_stable_frames;

    if(CAR_BALL_TASK3_TO_POSITIVE == ball_task3_phase)
    {
        expected_target = CAR_BALL_TASK3_POSITIVE_TARGET_UNITS;
        required_stable_frames = CAR_BALL_TASK3_RIGHT_STABLE_FRAMES;
    }
    else if(CAR_BALL_TASK3_TO_NEGATIVE == ball_task3_phase)
    {
        expected_target = CAR_BALL_TASK3_NEGATIVE_TARGET_UNITS;
        required_stable_frames = CAR_BALL_TASK3_LEFT_STABLE_FRAMES;
    }
    else
    {
        return;
    }

    if(car_ball_tracker_abs((float)dx - expected_target) >
        CAR_BALL_TASK3_TOLERANCE_UNITS)
    {
        ball_task3_stable_frames = 0U;
        return;
    }

    ball_task3_stable_frames++;
    if(ball_task3_stable_frames < required_stable_frames)
    {
        return;
    }
    ball_task3_stable_frames = 0U;

    if(CAR_BALL_TASK3_TO_NEGATIVE == ball_task3_phase)
    {
        ball_task3_phase = CAR_BALL_TASK3_TO_POSITIVE;
        car_ball_tracker_apply_target(CAR_BALL_TASK3_POSITIVE_TARGET_UNITS);
    }
    else
    {
        ball_task3_phase = CAR_BALL_TASK3_COMPLETE;
        ball_task3_complete_pending = 1U;
    }
}

static void car_ball_tracker_bldc_send(const uint8 *data, uint8 length)
{
    uart_write_buffer(CAR_BOARD_BLDC_UART, data, length);
}

static void car_ball_tracker_bldc_uart_callback(uint32 event, void *ptr)
{
    uint8 data;

    (void)ptr;
    if(UART_INTERRUPT_STATE_RX != event)
    {
        return;
    }
    while(uart_query_byte(CAR_BOARD_BLDC_UART, &data))
    {
        car_bldc_parse_rx_byte(data);
    }
}

static void car_ball_tracker_vision_uart_callback(uint32 event, void *ptr)
{
    uint8 data;

    (void)ptr;
    if(UART_INTERRUPT_STATE_RX != event)
    {
        return;
    }
    while(uart_query_byte(CAR_BOARD_VISION_UART, &data))
    {
        car_ball_tracker_parse_vision_byte(data);
    }
}

static void car_ball_tracker_motor_delay(void)
{
    system_delay_us(CAR_BALL_COMMAND_GAP_US);
}

static void car_ball_tracker_motor_init(void)
{
    car_bldc_disable(CAR_BALL_MOTOR_ADDRESS);
    car_ball_tracker_motor_delay();
    car_bldc_set_acceleration(CAR_BALL_MOTOR_ADDRESS,
        CAR_BALL_MOTOR_ACCELERATION_RPS2);
    car_ball_tracker_motor_delay();
    car_bldc_enable(CAR_BALL_MOTOR_ADDRESS);
    car_ball_tracker_motor_delay();
    car_bldc_set_mode(CAR_BALL_MOTOR_ADDRESS,
        CAR_BLDC_MODE_SINGLE_POSITION);
    car_ball_tracker_motor_delay();
    car_bldc_set_speed(CAR_BALL_MOTOR_ADDRESS,
        CAR_BALL_MOTOR_SPEED_RPM);
    car_ball_tracker_motor_delay();
    car_bldc_set_single_angle(CAR_BALL_MOTOR_ADDRESS, 0U);
    system_delay_ms(CAR_BALL_ZERO_SETTLE_MS);
    car_bldc_clear_multi_angle(CAR_BALL_MOTOR_ADDRESS);
    car_ball_tracker_motor_delay();
    car_bldc_set_mode(CAR_BALL_MOTOR_ADDRESS,
        CAR_BLDC_MODE_MULTI_POSITION_L);
    car_ball_tracker_motor_delay();
    car_bldc_set_multi_angle(CAR_BALL_MOTOR_ADDRESS, 0);
    car_ball_tracker_motor_delay();
}

void car_ball_tracker_init(void)
{
    vision_tracking = 0U;
    vision_dx = 0;
    vision_data_ready = 0U;
    vision_frame_count = 0U;
    vision_rx_state = CAR_VISION_RX_WAIT_HEADER;
    vision_rx_index = 0U;
    ball_target_pixels = CAR_BALL_TARGET_DEFAULT_PIXELS;
    ball_pid_output_deg = 0.0f;
    ball_previous_accel_mps2 = 0.0f;
    ball_command_angle_x10 = 0;
    ball_task2_hold_active = 1U;
    ball_task3_phase = CAR_BALL_TASK3_IDLE;
    ball_task3_stable_frames = 0U;
    ball_task3_complete_pending = 0U;

    car_pid_init(&ball_position_pid, CAR_BALL_KP_DEFAULT,
        CAR_BALL_KI_DEFAULT, CAR_BALL_KD_DEFAULT,
        CAR_BALL_INTEGRAL_LIMIT, CAR_BALL_ANGLE_LIMIT_DEG);
    car_pid_init(&ball_task3_left_pid, CAR_BALL_TASK3_LEFT_KP,
        CAR_BALL_TASK3_LEFT_KI, CAR_BALL_TASK3_LEFT_KD,
        CAR_BALL_INTEGRAL_LIMIT, CAR_BALL_ANGLE_LIMIT_DEG);
    car_pid_init(&ball_task3_right_pid, CAR_BALL_TASK3_RIGHT_KP,
        CAR_BALL_TASK3_RIGHT_KI, CAR_BALL_TASK3_RIGHT_KD,
        CAR_BALL_INTEGRAL_LIMIT, CAR_BALL_ANGLE_LIMIT_DEG);

    uart_init(CAR_BOARD_BLDC_UART, CAR_BALL_UART_BAUDRATE,
        CAR_BOARD_BLDC_UART_TX, CAR_BOARD_BLDC_UART_RX);
    uart_set_callback(CAR_BOARD_BLDC_UART,
        car_ball_tracker_bldc_uart_callback, NULL);
    uart_set_interrupt_config(CAR_BOARD_BLDC_UART,
        UART_INTERRUPT_CONFIG_RX_ENABLE);
    interrupt_set_priority(UART1_INT_IRQn, CAR_BALL_UART_IRQ_PRIORITY);

    uart_init(CAR_BOARD_VISION_UART, CAR_BALL_UART_BAUDRATE,
        CAR_BOARD_VISION_UART_TX, CAR_BOARD_VISION_UART_RX);
    uart_set_callback(CAR_BOARD_VISION_UART,
        car_ball_tracker_vision_uart_callback, NULL);
    uart_set_interrupt_config(CAR_BOARD_VISION_UART,
        UART_INTERRUPT_CONFIG_RX_ENABLE);
    interrupt_set_priority(UART2_INT_IRQn, CAR_BALL_UART_IRQ_PRIORITY);

    car_bldc_set_send_function(car_ball_tracker_bldc_send);
    car_ball_tracker_motor_init();
}

void car_ball_tracker_parse_vision_byte(uint8 data)
{
    switch(vision_rx_state)
    {
        case CAR_VISION_RX_WAIT_HEADER:
            if(CAR_VISION_FRAME_HEADER == data)
            {
                vision_rx_index = 0U;
                vision_rx_state = CAR_VISION_RX_PAYLOAD;
            }
            break;

        case CAR_VISION_RX_PAYLOAD:
            vision_rx_buffer[vision_rx_index++] = data;
            if(vision_rx_index >= CAR_VISION_PAYLOAD_LENGTH)
            {
                vision_rx_state = CAR_VISION_RX_WAIT_TAIL;
            }
            break;

        case CAR_VISION_RX_WAIT_TAIL:
            if(CAR_VISION_FRAME_TAIL == data)
            {
                vision_tracking = (0x01U == vision_rx_buffer[0]) ? 1U : 0U;
                vision_dx = (int16)(((uint16)vision_rx_buffer[1] << 8) |
                    (uint16)vision_rx_buffer[2]);
                vision_data_ready = 1U;
                vision_frame_count++;
                vision_rx_state = CAR_VISION_RX_WAIT_HEADER;
            }
            else if(CAR_VISION_FRAME_HEADER == data)
            {
                vision_rx_index = 0U;
                vision_rx_state = CAR_VISION_RX_PAYLOAD;
            }
            else
            {
                vision_rx_state = CAR_VISION_RX_WAIT_HEADER;
            }
            break;

        default:
            vision_rx_state = CAR_VISION_RX_WAIT_HEADER;
            vision_rx_index = 0U;
            break;
    }
}

void car_ball_tracker_update_5ms(uint8 task2_hold_zero,
    float vehicle_accel_ref_mps2)
{
    uint32 interrupt_state;
    uint8 data_ready;
    uint8 tracking;
    int16 dx;
    float output_deg;
    int32 command_angle_x10;
    car_pid_struct *active_pid;

    interrupt_state = interrupt_global_disable();
    data_ready = vision_data_ready;
    tracking = vision_tracking;
    dx = vision_dx;
    vision_data_ready = 0U;
    interrupt_global_enable(interrupt_state);

    if(task2_hold_zero)
    {
        if(!ball_task2_hold_active)
        {
            ball_task2_hold_active = 1U;
            car_ball_tracker_task3_cancel();
            car_ball_tracker_apply_target(CAR_BALL_TARGET_DEFAULT_PIXELS);
            ball_pid_output_deg = 0.0f;
            ball_previous_accel_mps2 = 0.0f;
            ball_command_angle_x10 = 0;
            car_bldc_set_multi_angle(CAR_BALL_MOTOR_ADDRESS, 0);
            car_ball_tracker_motor_delay();
        }
        return;
    }
    ball_task2_hold_active = 0U;

    if(data_ready && tracking)
    {
        if(CAR_BALL_TASK3_TO_NEGATIVE == ball_task3_phase)
        {
            active_pid = &ball_task3_left_pid;
        }
        else if(CAR_BALL_TASK3_IDLE != ball_task3_phase)
        {
            active_pid = &ball_task3_right_pid;
        }
        else
        {
            active_pid = &ball_position_pid;
        }

        ball_pid_output_deg = car_pid_update(active_pid,
            ball_target_pixels, (float)dx, CAR_CONTROL_DT_S, 0.0f);
        car_ball_tracker_update_task3(dx);
    }

    output_deg = ball_pid_output_deg +
        car_ball_tracker_accel_feedforward(vehicle_accel_ref_mps2);
    output_deg = car_ball_tracker_clamp(output_deg,
        CAR_BALL_ANGLE_LIMIT_DEG);
    command_angle_x10 = (int32)(output_deg * 10.0f);
    if(command_angle_x10 != ball_command_angle_x10)
    {
        ball_command_angle_x10 = command_angle_x10;
        car_bldc_set_multi_angle(CAR_BALL_MOTOR_ADDRESS,
            ball_command_angle_x10);
        car_ball_tracker_motor_delay();
    }
}

void car_ball_tracker_set_pid(float kp, float ki, float kd)
{
    car_pid_set_gains(&ball_position_pid, kp, ki, kd);
}

void car_ball_tracker_set_task3_pid(float kp, float ki, float kd)
{
    car_pid_set_gains(&ball_task3_left_pid, kp, ki, kd);
    car_pid_set_gains(&ball_task3_right_pid, kp, ki, kd);
}

void car_ball_tracker_set_task3_left_pid(float kp, float ki, float kd)
{
    car_pid_set_gains(&ball_task3_left_pid, kp, ki, kd);
}

void car_ball_tracker_set_task3_right_pid(float kp, float ki, float kd)
{
    car_pid_set_gains(&ball_task3_right_pid, kp, ki, kd);
}

void car_ball_tracker_set_target(float target_pixels)
{
    car_ball_tracker_task3_cancel();
    car_ball_tracker_apply_target(target_pixels);
}

void car_ball_tracker_task3_start(void)
{
    ball_task3_phase = CAR_BALL_TASK3_TO_NEGATIVE;
    ball_task3_stable_frames = 0U;
    ball_task3_complete_pending = 0U;
    car_ball_tracker_apply_target(CAR_BALL_TASK3_NEGATIVE_TARGET_UNITS);
}

void car_ball_tracker_task3_cancel(void)
{
    ball_task3_phase = CAR_BALL_TASK3_IDLE;
    ball_task3_stable_frames = 0U;
    ball_task3_complete_pending = 0U;
    car_pid_reset(&ball_position_pid);
    car_pid_reset(&ball_task3_left_pid);
    car_pid_reset(&ball_task3_right_pid);
}

uint8 car_ball_tracker_task3_take_complete(void)
{
    uint8 complete = ball_task3_complete_pending;

    ball_task3_complete_pending = 0U;
    return complete;
}

uint8 car_ball_tracker_is_tracking(void)
{
    return vision_tracking;
}

int16 car_ball_tracker_get_dx(void)
{
    return vision_dx;
}

float car_ball_tracker_get_target(void)
{
    return ball_target_pixels;
}

int32 car_ball_tracker_get_command_angle_x10(void)
{
    return ball_command_angle_x10;
}

uint32 car_ball_tracker_get_frame_count(void)
{
    return vision_frame_count;
}

car_ball_task3_phase_enum car_ball_tracker_get_task3_phase(void)
{
    return ball_task3_phase;
}

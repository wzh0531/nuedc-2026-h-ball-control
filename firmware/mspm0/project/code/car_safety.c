#include "car_safety.h"

#include "car_config.h"
#include "car_encoder.h"
#include "car_imu.h"
#include "car_motor.h"

static uint32 fault_latch;
static uint8 left_no_edge_ticks;
static uint8 right_no_edge_ticks;
static uint8 line_lost_ticks;
static uint32 previous_invalid_left;
static uint32 previous_invalid_right;

static int16 car_safety_abs_output(int16 value)
{
    return (value < 0) ? (int16)-value : value;
}

void car_safety_init(void)
{
    car_safety_reset();
}

void car_safety_reset(void)
{
    fault_latch = CHASSIS_FAULT_NONE;
    left_no_edge_ticks = 0U;
    right_no_edge_ticks = 0U;
    line_lost_ticks = 0U;
    previous_invalid_left = car_encoder_get_invalid(CAR_ENCODER_LEFT);
    previous_invalid_right = car_encoder_get_invalid(CAR_ENCODER_RIGHT);
}

static void car_safety_update_stall(void)
{
    if((car_safety_abs_output(car_motor_get_output(CAR_MOTOR_LEFT)) >
        CAR_MOTOR_STALL_PWM_THRESHOLD) &&
        (0 == car_encoder_get_delta(CAR_ENCODER_LEFT)))
    {
        if(left_no_edge_ticks < 255U)
        {
            left_no_edge_ticks++;
        }
    }
    else
    {
        left_no_edge_ticks = 0U;
    }

    if((car_safety_abs_output(car_motor_get_output(CAR_MOTOR_RIGHT)) >
        CAR_MOTOR_STALL_PWM_THRESHOLD) &&
        (0 == car_encoder_get_delta(CAR_ENCODER_RIGHT)))
    {
        if(right_no_edge_ticks < 255U)
        {
            right_no_edge_ticks++;
        }
    }
    else
    {
        right_no_edge_ticks = 0U;
    }

    if(left_no_edge_ticks >= CAR_STALL_NO_EDGE_TICKS)
    {
        fault_latch |= CHASSIS_FAULT_LEFT_STALL;
    }
    if(right_no_edge_ticks >= CAR_STALL_NO_EDGE_TICKS)
    {
        fault_latch |= CHASSIS_FAULT_RIGHT_STALL;
    }
}

static void car_safety_update_encoder_signal(void)
{
    uint32 invalid_left = car_encoder_get_invalid(CAR_ENCODER_LEFT);
    uint32 invalid_right = car_encoder_get_invalid(CAR_ENCODER_RIGHT);

    if(((invalid_left - previous_invalid_left) > CAR_ENCODER_INVALID_LIMIT) ||
        ((invalid_right - previous_invalid_right) > CAR_ENCODER_INVALID_LIMIT))
    {
        fault_latch |= CHASSIS_FAULT_ENCODER_SIGNAL;
    }
    previous_invalid_left = invalid_left;
    previous_invalid_right = invalid_right;
}

uint32 car_safety_update(uint8 line_valid)
{
    if(line_valid)
    {
        line_lost_ticks = 0U;
    }
    else if(line_lost_ticks < 255U)
    {
        line_lost_ticks++;
    }

    if(line_lost_ticks >= CAR_LINE_FAULT_TICKS)
    {
        fault_latch |= CHASSIS_FAULT_LINE_LOST;
    }
    if(!car_imu_is_ready())
    {
        fault_latch |= CHASSIS_FAULT_IMU;
    }

    car_safety_update_stall();
    car_safety_update_encoder_signal();
    return fault_latch;
}

void car_safety_latch(uint32 fault_bits)
{
    fault_latch |= fault_bits;
}

uint32 car_safety_get_faults(void)
{
    return fault_latch;
}

uint8 car_safety_get_line_lost_ticks(void)
{
    return line_lost_ticks;
}

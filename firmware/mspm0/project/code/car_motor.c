#include "car_motor.h"

#include "car_board.h"
#include "car_config.h"

static int16 motor_output[CAR_MOTOR_COUNT];
static uint8 reverse_zero_ticks[CAR_MOTOR_COUNT];

static int16 car_motor_limit(int32 value)
{
    if(value > CAR_MOTOR_OUTPUT_LIMIT)
    {
        value = CAR_MOTOR_OUTPUT_LIMIT;
    }
    else if(value < -CAR_MOTOR_OUTPUT_LIMIT)
    {
        value = -CAR_MOTOR_OUTPUT_LIMIT;
    }
    return (int16)value;
}

static pwm_channel_enum car_motor_pwm(car_motor_id_enum motor)
{
    return (CAR_MOTOR_LEFT == motor) ? CAR_BOARD_PWM_LEFT : CAR_BOARD_PWM_RIGHT;
}

static void car_motor_write_direction(car_motor_id_enum motor, int16 output)
{
    gpio_pin_enum in1 = (CAR_MOTOR_LEFT == motor) ?
        CAR_BOARD_MOTOR_LEFT_IN1 : CAR_BOARD_MOTOR_RIGHT_IN1;
    gpio_pin_enum in2 = (CAR_MOTOR_LEFT == motor) ?
        CAR_BOARD_MOTOR_LEFT_IN2 : CAR_BOARD_MOTOR_RIGHT_IN2;

    if(output > 0)
    {
        gpio_high(in1);
        gpio_low(in2);
    }
    else if(output < 0)
    {
        gpio_low(in1);
        gpio_high(in2);
    }
    else
    {
        /* IN1=IN2=0 配合 PWM=0 作为本项目故障/静止状态。 */
        gpio_low(in1);
        gpio_low(in2);
    }
}

static void car_motor_write_zero(car_motor_id_enum motor)
{
    pwm_set_duty(car_motor_pwm(motor), 0U);
    car_motor_write_direction(motor, 0);
    motor_output[motor] = 0;
}

void car_motor_init(void)
{
    gpio_init(CAR_BOARD_MOTOR_LEFT_IN1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CAR_BOARD_MOTOR_LEFT_IN2, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CAR_BOARD_MOTOR_RIGHT_IN1, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gpio_init(CAR_BOARD_MOTOR_RIGHT_IN2, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pwm_init(CAR_BOARD_PWM_LEFT, CAR_MOTOR_PWM_FREQ_HZ, 0U);
    pwm_init(CAR_BOARD_PWM_RIGHT, CAR_MOTOR_PWM_FREQ_HZ, 0U);
    reverse_zero_ticks[CAR_MOTOR_LEFT] = 0U;
    reverse_zero_ticks[CAR_MOTOR_RIGHT] = 0U;
    car_motor_stop();
}

void car_motor_set(car_motor_id_enum motor, int16 output)
{
    int32 signed_output;
    int16 logical_output;
    uint32 duty;

    if(motor >= CAR_MOTOR_COUNT)
    {
        return;
    }

    logical_output = car_motor_limit(output);
    signed_output = logical_output;
    signed_output *= (CAR_MOTOR_LEFT == motor) ?
        CAR_MOTOR_LEFT_SIGN : CAR_MOTOR_RIGHT_SIGN;
    output = car_motor_limit(signed_output);

    /*
     * 禁止直接反转。检测到符号变化后，本次强制归零；后续至少保持
     * CAR_MOTOR_REVERSE_ZERO_TICKS 个完整 5 ms 调用周期。
     */
    if(((motor_output[motor] > 0) && (logical_output < 0)) ||
        ((motor_output[motor] < 0) && (logical_output > 0)))
    {
        reverse_zero_ticks[motor] = CAR_MOTOR_REVERSE_ZERO_TICKS;
        car_motor_write_zero(motor);
        return;
    }

    if(reverse_zero_ticks[motor] > 0U)
    {
        reverse_zero_ticks[motor]--;
        car_motor_write_zero(motor);
        return;
    }

    if(0 == logical_output)
    {
        car_motor_write_zero(motor);
        return;
    }

    duty = (uint32)((output < 0) ? -output : output);
    car_motor_write_direction(motor, output);
    pwm_set_duty(car_motor_pwm(motor), duty);
    motor_output[motor] = logical_output;
}

void car_motor_set_pair(int16 left, int16 right)
{
    car_motor_set(CAR_MOTOR_LEFT, left);
    car_motor_set(CAR_MOTOR_RIGHT, right);
}

void car_motor_stop(void)
{
    reverse_zero_ticks[CAR_MOTOR_LEFT] = 0U;
    reverse_zero_ticks[CAR_MOTOR_RIGHT] = 0U;
    car_motor_write_zero(CAR_MOTOR_LEFT);
    car_motor_write_zero(CAR_MOTOR_RIGHT);
}

int16 car_motor_get_output(car_motor_id_enum motor)
{
    return (motor < CAR_MOTOR_COUNT) ? motor_output[motor] : 0;
}

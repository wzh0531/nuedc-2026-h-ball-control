#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "car_board.h"
#include "car_config.h"
#include "car_encoder.h"
#include "car_gray.h"
#include "car_keys.h"
#include "car_motor.h"
#include "zf_common_interrupt.h"
#include "zf_driver_exti.h"

#define HOST_PIN_COUNT       (64U)
#define HOST_PWM_COUNT       (2U)

static uint8 gpio_level[HOST_PIN_COUNT];
static void_callback_uint32_ptr exti_callback[HOST_PIN_COUNT];
static void *exti_context[HOST_PIN_COUNT];
static uint32 pwm_frequency[HOST_PWM_COUNT];
static uint32 pwm_duty[HOST_PWM_COUNT];
static uint8 interrupt_priority;
static uint8 gray_serial_raw;
static uint8 gray_serial_index;
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

static void host_trigger_pin(gpio_pin_enum pin, uint8 level)
{
    gpio_level[pin] = level;
    if(NULL != exti_callback[pin])
    {
        exti_callback[pin](0U, exti_context[pin]);
    }
}

static void host_left_forward_cycle(void)
{
    /*
     * Measured vehicle-forward sequence is 00->01->11->10->00; the left
     * installation therefore needs CAR_ENCODER_LEFT_SIGN=-1.
     */
    host_trigger_pin(CAR_BOARD_ENCODER_LEFT_B, 1U);
    host_trigger_pin(CAR_BOARD_ENCODER_LEFT_A, 1U);
    host_trigger_pin(CAR_BOARD_ENCODER_LEFT_B, 0U);
    host_trigger_pin(CAR_BOARD_ENCODER_LEFT_A, 0U);
}

static void host_right_forward_cycle(void)
{
    /*
     * Measured vehicle-forward sequence is 00->10->11->01->00; the right
     * installation therefore needs CAR_ENCODER_RIGHT_SIGN=1.
     */
    host_trigger_pin(CAR_BOARD_ENCODER_RIGHT_A, 1U);
    host_trigger_pin(CAR_BOARD_ENCODER_RIGHT_B, 1U);
    host_trigger_pin(CAR_BOARD_ENCODER_RIGHT_A, 0U);
    host_trigger_pin(CAR_BOARD_ENCODER_RIGHT_B, 0U);
}

static void test_encoder_x4_counting(void)
{
    uint16 cycle;

    memset(gpio_level, 0, sizeof(gpio_level));
    memset(exti_callback, 0, sizeof(exti_callback));
    car_encoder_init();
    car_encoder_reset();
    TEST_ASSERT("encoder IRQ priority", 0U == interrupt_priority);

    for(cycle = 0U; cycle < 364U; cycle++)
    {
        host_left_forward_cycle();
        host_right_forward_cycle();
    }
    car_encoder_sample();
    TEST_ASSERT("left 13PPR x4 x28", 1456 ==
        car_encoder_get_delta(CAR_ENCODER_LEFT));
    TEST_ASSERT("right 13PPR x4 x28", 1456 ==
        car_encoder_get_delta(CAR_ENCODER_RIGHT));
    TEST_ASSERT("left total", 1456 ==
        car_encoder_get_total(CAR_ENCODER_LEFT));
    TEST_ASSERT("right total", 1456 ==
        car_encoder_get_total(CAR_ENCODER_RIGHT));

    car_encoder_sample();
    TEST_ASSERT("sample clears left pending", 0 ==
        car_encoder_get_delta(CAR_ENCODER_LEFT));
    TEST_ASSERT("sample clears right pending", 0 ==
        car_encoder_get_delta(CAR_ENCODER_RIGHT));
    test_pass("AB quadrature x4 count and measured forward signs");
}

static void test_encoder_invalid_transition(void)
{
    memset(gpio_level, 0, sizeof(gpio_level));
    car_encoder_reset();

    /* 00->11 skips a legal neighboring Gray state and must be counted invalid. */
    gpio_level[CAR_BOARD_ENCODER_LEFT_A] = 1U;
    gpio_level[CAR_BOARD_ENCODER_LEFT_B] = 1U;
    exti_callback[CAR_BOARD_ENCODER_LEFT_A](
        0U, exti_context[CAR_BOARD_ENCODER_LEFT_A]);
    TEST_ASSERT("invalid transition detected", 1U ==
        car_encoder_get_invalid(CAR_ENCODER_LEFT));
    test_pass("encoder invalid Gray-code transition");
}

static void test_motor_limit_and_reverse_guard(void)
{
    memset(gpio_level, 0, sizeof(gpio_level));
    memset(pwm_frequency, 0, sizeof(pwm_frequency));
    memset(pwm_duty, 0, sizeof(pwm_duty));
    car_motor_init();

    TEST_ASSERT("left PWM 20kHz", CAR_MOTOR_PWM_FREQ_HZ ==
        pwm_frequency[CAR_BOARD_PWM_LEFT]);
    TEST_ASSERT("right PWM 20kHz", CAR_MOTOR_PWM_FREQ_HZ ==
        pwm_frequency[CAR_BOARD_PWM_RIGHT]);

    car_motor_set(CAR_MOTOR_LEFT, 9000);
    TEST_ASSERT("motor output clamps to 65 percent",
        CAR_MOTOR_OUTPUT_LIMIT ==
        car_motor_get_output(CAR_MOTOR_LEFT));
    TEST_ASSERT("motor PWM clamps to 65 percent",
        (uint32)CAR_MOTOR_OUTPUT_LIMIT ==
        pwm_duty[CAR_BOARD_PWM_LEFT]);
    TEST_ASSERT("forward IN1 high",
        1U == gpio_level[CAR_BOARD_MOTOR_LEFT_IN1]);
    TEST_ASSERT("forward IN2 low",
        0U == gpio_level[CAR_BOARD_MOTOR_LEFT_IN2]);

    car_motor_set(CAR_MOTOR_LEFT, -3000);
    TEST_ASSERT("reverse request first zero", 0 ==
        car_motor_get_output(CAR_MOTOR_LEFT));
    TEST_ASSERT("reverse request first PWM zero", 0U ==
        pwm_duty[CAR_BOARD_PWM_LEFT]);
    TEST_ASSERT("reverse request first direction stop",
        (0U == gpio_level[CAR_BOARD_MOTOR_LEFT_IN1]) &&
        (0U == gpio_level[CAR_BOARD_MOTOR_LEFT_IN2]));

    car_motor_set(CAR_MOTOR_LEFT, -3000);
    TEST_ASSERT("configured zero interval held", 0 ==
        car_motor_get_output(CAR_MOTOR_LEFT));
    TEST_ASSERT("configured zero interval PWM", 0U ==
        pwm_duty[CAR_BOARD_PWM_LEFT]);

    car_motor_set(CAR_MOTOR_LEFT, -3000);
    TEST_ASSERT("reverse applies after guard", -3000 ==
        car_motor_get_output(CAR_MOTOR_LEFT));
    TEST_ASSERT("reverse duty", 3000U ==
        pwm_duty[CAR_BOARD_PWM_LEFT]);
    TEST_ASSERT("reverse IN1 low",
        0U == gpio_level[CAR_BOARD_MOTOR_LEFT_IN1]);
    TEST_ASSERT("reverse IN2 high",
        1U == gpio_level[CAR_BOARD_MOTOR_LEFT_IN2]);

    car_motor_stop();
    TEST_ASSERT("stop clears both PWM",
        (0U == pwm_duty[CAR_BOARD_PWM_LEFT]) &&
        (0U == pwm_duty[CAR_BOARD_PWM_RIGHT]));
    TEST_ASSERT("stop clears all directions",
        (0U == gpio_level[CAR_BOARD_MOTOR_LEFT_IN1]) &&
        (0U == gpio_level[CAR_BOARD_MOTOR_LEFT_IN2]) &&
        (0U == gpio_level[CAR_BOARD_MOTOR_RIGHT_IN1]) &&
        (0U == gpio_level[CAR_BOARD_MOTOR_RIGHT_IN2]));
    test_pass("TB6612 limit, direction and reverse zero guard");
}

static void host_set_gray_black_mask(uint8 desired_black_mask)
{
    /*
     * The measured auxiliary board sends 0=black and 1=white. Its first
     * serial bit maps directly to black_mask bit0, the physical-left sensor.
     */
    gray_serial_raw = (uint8)(~desired_black_mask);
    gray_serial_index = 0U;
}

static void test_gray_serial_and_weighting(void)
{
    static const float expected_single_error[8] =
    {
        -1.0f, -5.0f / 7.0f, -3.0f / 7.0f, -1.0f / 7.0f,
         1.0f / 7.0f,  3.0f / 7.0f,  5.0f / 7.0f,  1.0f
    };
    uint8 index;
    float error = 99.0f;

    host_set_gray_black_mask(0U);
    car_gray_init();
    TEST_ASSERT("all-white mask", 0U == car_gray_get_mask());
    TEST_ASSERT("all-white count", 0U == car_gray_get_black_count());
    TEST_ASSERT("all-white is invalid", !car_gray_get_error(&error));

    for(index = 0U; index < 8U; index++)
    {
        host_set_gray_black_mask((uint8)(1U << index));
        (void)car_gray_read();
        TEST_ASSERT("single gray mask",
            (uint8)(1U << index) == car_gray_get_mask());
        TEST_ASSERT("single gray count", 1U ==
            car_gray_get_black_count());
        TEST_ASSERT("single gray valid", car_gray_get_error(&error));
        TEST_ASSERT("single gray weighted error",
            (error - expected_single_error[index] < 0.00001f) &&
            (expected_single_error[index] - error < 0.00001f));
    }

    host_set_gray_black_mask(0x18U);
    (void)car_gray_read();
    TEST_ASSERT("center double valid", car_gray_get_error(&error));
    TEST_ASSERT("center double zero error",
        (error < 0.00001f) && (error > -0.00001f));

    host_set_gray_black_mask(0xFFU);
    (void)car_gray_read();
    TEST_ASSERT("all-black mask", 0xFFU == car_gray_get_mask());
    TEST_ASSERT("all-black count", 8U == car_gray_get_black_count());
    TEST_ASSERT("all-black valid", car_gray_get_error(&error));
    TEST_ASSERT("all-black zero error",
        (error < 0.00001f) && (error > -0.00001f));
    test_pass("gray serial polarity, order and eight-point weighting");
}

static void test_key_debounce_and_long_press(void)
{
    uint16 tick;

    TEST_ASSERT("extension board KEY3 uses PB18",
        B18 == CAR_BOARD_KEY_3);
    car_keys_init();

    /* A one-tick pulse is shorter than the 10 ms debounce threshold. */
    gpio_level[CAR_BOARD_KEY_1] = 0U;
    car_keys_scan_5ms();
    TEST_ASSERT("key not pressed before debounce",
        !car_keys_is_pressed(CAR_KEY_1));
    gpio_level[CAR_BOARD_KEY_1] = 1U;
    car_keys_scan_5ms();
    TEST_ASSERT("key bounce rejected", CAR_KEY_EVENT_NONE ==
        car_keys_take_event(CAR_KEY_1));

    gpio_level[CAR_BOARD_KEY_1] = 0U;
    car_keys_scan_5ms();
    car_keys_scan_5ms();
    TEST_ASSERT("key pressed after debounce",
        car_keys_is_pressed(CAR_KEY_1));
    gpio_level[CAR_BOARD_KEY_1] = 1U;
    car_keys_scan_5ms();
    TEST_ASSERT("key pressed clears on release",
        !car_keys_is_pressed(CAR_KEY_1));
    TEST_ASSERT("key short press", CAR_KEY_EVENT_SHORT ==
        car_keys_take_event(CAR_KEY_1));
    TEST_ASSERT("key event is one-shot", CAR_KEY_EVENT_NONE ==
        car_keys_take_event(CAR_KEY_1));

    gpio_level[CAR_BOARD_KEY_4] = 0U;
    for(tick = 0U; tick < 200U; tick++)
    {
        car_keys_scan_5ms();
    }
    TEST_ASSERT("key long press at one second", CAR_KEY_EVENT_LONG ==
        car_keys_take_event(CAR_KEY_4));
    TEST_ASSERT("long-held key remains pressed",
        car_keys_is_pressed(CAR_KEY_4));
    gpio_level[CAR_BOARD_KEY_4] = 1U;
    car_keys_scan_5ms();
    TEST_ASSERT("long press does not emit short on release",
        CAR_KEY_EVENT_NONE == car_keys_take_event(CAR_KEY_4));
    test_pass("four-key debounce, pressed level and long-press event");
}

int main(void)
{
    test_encoder_x4_counting();
    test_encoder_invalid_transition();
    test_motor_limit_and_reverse_guard();
    test_gray_serial_and_weighting();
    test_key_debounce_and_long_press();
    printf("[PASS] all %u encoder/motor driver tests\n", test_count);
    return 0;
}

/* ----------------------------- Host BSP stubs ----------------------------- */

void gpio_init(gpio_pin_enum pin, gpio_dir_enum dir,
    gpio_level_enum level, gpio_mode_enum mode)
{
    (void)dir;
    (void)mode;
    gpio_level[pin] = (uint8)level;
}

uint8 gpio_get_level(gpio_pin_enum pin)
{
    if(CAR_BOARD_GRAY_DATA == pin)
    {
        uint8 value = (uint8)((gray_serial_raw >> gray_serial_index) & 0x01U);

        if(gray_serial_index < 7U)
        {
            gray_serial_index++;
        }
        return value;
    }
    return gpio_level[pin];
}

void gpio_high(gpio_pin_enum pin)
{
    gpio_level[pin] = 1U;
}

void gpio_low(gpio_pin_enum pin)
{
    gpio_level[pin] = 0U;
}

void pwm_init(pwm_channel_enum channel, uint32 frequency, uint32 duty)
{
    pwm_frequency[channel] = frequency;
    pwm_duty[channel] = duty;
}

void pwm_set_duty(pwm_channel_enum channel, uint32 duty)
{
    pwm_duty[channel] = duty;
}

void exti_init(gpio_pin_enum pin, exti_trigger_enum trigger,
    void_callback_uint32_ptr callback, void *ptr)
{
    TEST_ASSERT("EXTI configured both edges", EXTI_TRIGGER_BOTH == trigger);
    exti_callback[pin] = callback;
    exti_context[pin] = ptr;
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
    interrupt_priority = priority;
}

void system_delay_us(uint32 time_us)
{
    (void)time_us;
}

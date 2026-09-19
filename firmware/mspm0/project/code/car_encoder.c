#include "car_encoder.h"

#include "car_board.h"
#include "car_config.h"
#include "zf_common_interrupt.h"
#include "zf_driver_exti.h"

/*
 * 合法 Gray-code 相邻转换返回 ±1，未变化返回 0，跨两位跳变返回 2。
 * 正负方向最后再乘左右编码器方向系数，便于架空测试时只改配置。
 */
static const int8 quadrature_table[16] =
{
     0, -1,  1,  2,
     1,  0,  2, -1,
    -1,  2,  0,  1,
     2,  1, -1,  0,
};

static volatile int32 encoder_pending[CAR_ENCODER_COUNT];
static volatile uint32 encoder_invalid[CAR_ENCODER_COUNT];
static volatile uint8 encoder_previous_ab[CAR_ENCODER_COUNT];
static int16 encoder_delta[CAR_ENCODER_COUNT];
static int32 encoder_total[CAR_ENCODER_COUNT];

static uint8 car_encoder_read_ab(car_encoder_id_enum encoder)
{
    gpio_pin_enum pin_a = (CAR_ENCODER_LEFT == encoder) ?
        CAR_BOARD_ENCODER_LEFT_A : CAR_BOARD_ENCODER_RIGHT_A;
    gpio_pin_enum pin_b = (CAR_ENCODER_LEFT == encoder) ?
        CAR_BOARD_ENCODER_LEFT_B : CAR_BOARD_ENCODER_RIGHT_B;

    return (uint8)((gpio_get_level(pin_a) ? 2U : 0U) |
        (gpio_get_level(pin_b) ? 1U : 0U));
}

static void car_encoder_decode_edge(car_encoder_id_enum encoder)
{
    uint8 current_ab = car_encoder_read_ab(encoder);
    uint8 table_index = (uint8)((encoder_previous_ab[encoder] << 2U) | current_ab);
    int8 step = quadrature_table[table_index];

    encoder_previous_ab[encoder] = current_ab;
    if(2 == step)
    {
        encoder_invalid[encoder]++;
    }
    else
    {
        encoder_pending[encoder] += step;
    }
}

static void car_encoder_left_callback(uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;
    car_encoder_decode_edge(CAR_ENCODER_LEFT);
}

static void car_encoder_right_callback(uint32 event, void *ptr)
{
    (void)event;
    (void)ptr;
    car_encoder_decode_edge(CAR_ENCODER_RIGHT);
}

void car_encoder_init(void)
{
    exti_init(CAR_BOARD_ENCODER_LEFT_A, EXTI_TRIGGER_BOTH,
        car_encoder_left_callback, NULL);
    exti_init(CAR_BOARD_ENCODER_LEFT_B, EXTI_TRIGGER_BOTH,
        car_encoder_left_callback, NULL);
    exti_init(CAR_BOARD_ENCODER_RIGHT_A, EXTI_TRIGGER_BOTH,
        car_encoder_right_callback, NULL);
    exti_init(CAR_BOARD_ENCODER_RIGHT_B, EXTI_TRIGGER_BOTH,
        car_encoder_right_callback, NULL);
    /* 编码器边沿优先级最高，避免高速时漏计数。 */
    interrupt_set_priority(GPIOA_INT_IRQn, 0U);
    car_encoder_reset();
}

void car_encoder_sample(void)
{
    int32 left;
    int32 right;
    uint32 interrupt_state = interrupt_global_disable();

    left = encoder_pending[CAR_ENCODER_LEFT];
    right = encoder_pending[CAR_ENCODER_RIGHT];
    encoder_pending[CAR_ENCODER_LEFT] = 0;
    encoder_pending[CAR_ENCODER_RIGHT] = 0;
    interrupt_global_enable(interrupt_state);

    left *= CAR_ENCODER_LEFT_SIGN;
    right *= CAR_ENCODER_RIGHT_SIGN;
    encoder_delta[CAR_ENCODER_LEFT] = (int16)left;
    encoder_delta[CAR_ENCODER_RIGHT] = (int16)right;
    encoder_total[CAR_ENCODER_LEFT] += left;
    encoder_total[CAR_ENCODER_RIGHT] += right;
}

int16 car_encoder_get_delta(car_encoder_id_enum encoder)
{
    return (encoder < CAR_ENCODER_COUNT) ? encoder_delta[encoder] : 0;
}

int32 car_encoder_get_total(car_encoder_id_enum encoder)
{
    return (encoder < CAR_ENCODER_COUNT) ? encoder_total[encoder] : 0;
}

uint32 car_encoder_get_invalid(car_encoder_id_enum encoder)
{
    uint32 value = 0U;
    uint32 interrupt_state;

    if(encoder >= CAR_ENCODER_COUNT)
    {
        return 0U;
    }
    interrupt_state = interrupt_global_disable();
    value = encoder_invalid[encoder];
    interrupt_global_enable(interrupt_state);
    return value;
}

void car_encoder_reset(void)
{
    uint32 interrupt_state = interrupt_global_disable();

    encoder_pending[CAR_ENCODER_LEFT] = 0;
    encoder_pending[CAR_ENCODER_RIGHT] = 0;
    encoder_delta[CAR_ENCODER_LEFT] = 0;
    encoder_delta[CAR_ENCODER_RIGHT] = 0;
    encoder_total[CAR_ENCODER_LEFT] = 0;
    encoder_total[CAR_ENCODER_RIGHT] = 0;
    encoder_invalid[CAR_ENCODER_LEFT] = 0U;
    encoder_invalid[CAR_ENCODER_RIGHT] = 0U;
    encoder_previous_ab[CAR_ENCODER_LEFT] =
        car_encoder_read_ab(CAR_ENCODER_LEFT);
    encoder_previous_ab[CAR_ENCODER_RIGHT] =
        car_encoder_read_ab(CAR_ENCODER_RIGHT);
    interrupt_global_enable(interrupt_state);
}

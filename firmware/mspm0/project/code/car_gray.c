#include "car_gray.h"

#include "car_board.h"
#include "car_config.h"
#include "zf_driver_delay.h"

static uint8 gray_black_mask;
static uint8 gray_black_count;

#if CAR_GRAY_REVERSE_ORDER
static uint8 car_gray_reverse_bits(uint8 value)
{
    uint8 result = 0U;
    uint8 index;

    for(index = 0U; index < 8U; index++)
    {
        result |= (uint8)(((value >> index) & 0x01U) << (7U - index));
    }
    return result;
}
#endif

static uint8 car_gray_count_bits(uint8 value)
{
    uint8 count = 0U;

    while(value != 0U)
    {
        count += (uint8)(value & 0x01U);
        value >>= 1U;
    }
    return count;
}

void car_gray_init(void)
{
    gpio_init(CAR_BOARD_GRAY_DATA, GPI, GPIO_LOW, GPI_FLOATING_IN);
    gpio_init(CAR_BOARD_GRAY_CLOCK, GPO, GPIO_LOW, GPO_PUSH_PULL);
    gray_black_mask = 0U;
    gray_black_count = 0U;
    (void)car_gray_read();
}

uint8 car_gray_read(void)
{
    uint8 raw_value = 0U;
    uint8 index;

    gpio_low(CAR_BOARD_GRAY_CLOCK);
    for(index = 0U; index < 8U; index++)
    {
        gpio_high(CAR_BOARD_GRAY_CLOCK);
        system_delay_us(CAR_GRAY_CLOCK_DELAY_US);
        gpio_low(CAR_BOARD_GRAY_CLOCK);

        /* 感为例程按第一个脉冲写入 bit0。 */
        if(gpio_get_level(CAR_BOARD_GRAY_DATA))
        {
            raw_value |= (uint8)(1U << index);
        }
        system_delay_us(CAR_GRAY_CLOCK_DELAY_US);
    }

#if CAR_GRAY_REVERSE_ORDER
    raw_value = car_gray_reverse_bits(raw_value);
#endif

#if (0U == CAR_GRAY_BLACK_LEVEL)
    gray_black_mask = (uint8)(~raw_value);
#else
    gray_black_mask = raw_value;
#endif
    gray_black_count = car_gray_count_bits(gray_black_mask);
    return gray_black_mask;
}

uint8 car_gray_get_mask(void)
{
    return gray_black_mask;
}

uint8 car_gray_get_error(float *error)
{
    static const int8 weight[8] = {-7, -5, -3, -1, 1, 3, 5, 7};
    int16 weighted_sum = 0;
    uint8 index;

    if((NULL == error) || (0U == gray_black_count))
    {
        return 0U;
    }

    for(index = 0U; index < 8U; index++)
    {
        if((gray_black_mask & (uint8)(1U << index)) != 0U)
        {
            weighted_sum += weight[index];
        }
    }

    /* 归一化到约 -1..+1，方便不同探头组合使用同一组外环参数。 */
    *error = (float)weighted_sum / ((float)gray_black_count * 7.0f);
    return 1U;
}

uint8 car_gray_get_black_count(void)
{
    return gray_black_count;
}

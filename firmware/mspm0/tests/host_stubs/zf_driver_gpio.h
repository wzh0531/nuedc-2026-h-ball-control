#ifndef _HOST_STUB_ZF_DRIVER_GPIO_H_
#define _HOST_STUB_ZF_DRIVER_GPIO_H_

#include "zf_common_typedef.h"

/* Host-only pin identifiers. A and B groups use different numeric ranges. */
typedef enum
{
    A0 = 0,
    A1 = 1,
    A7 = 7,
    A12 = 12,
    A17 = 17,
    A25 = 25,
    A27 = 27,
    B0 = 32,
    B2 = 34,
    B3 = 35,
    B4 = 36,
    B5 = 37,
    B6 = 38,
    B7 = 39,
    B8 = 40,
    B9 = 41,
    B10 = 42,
    B11 = 43,
    B12 = 44,
    B14 = 46,
    B15 = 47,
    B16 = 48,
    B18 = 50,
    B22 = 54,
    B24 = 56,
}gpio_pin_enum;

typedef enum
{
    GPI = 0,
    GPO = 1,
}gpio_dir_enum;

typedef enum
{
    GPIO_LOW = 0,
    GPIO_HIGH = 1,
}gpio_level_enum;

typedef enum
{
    GPI_FLOATING_IN = 0,
    GPI_PULL_UP = 1,
    GPO_PUSH_PULL = 2,
}gpio_mode_enum;

void gpio_init(gpio_pin_enum pin, gpio_dir_enum dir,
    gpio_level_enum level, gpio_mode_enum mode);
uint8 gpio_get_level(gpio_pin_enum pin);
void gpio_high(gpio_pin_enum pin);
void gpio_low(gpio_pin_enum pin);

#endif

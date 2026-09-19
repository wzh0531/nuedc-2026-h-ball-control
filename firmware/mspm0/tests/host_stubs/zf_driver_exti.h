#ifndef _HOST_STUB_ZF_DRIVER_EXTI_H_
#define _HOST_STUB_ZF_DRIVER_EXTI_H_

#include "zf_common_typedef.h"
#include "zf_driver_gpio.h"

typedef enum
{
    EXTI_TRIGGER_RISING = 1,
    EXTI_TRIGGER_FALLING = 2,
    EXTI_TRIGGER_BOTH = 3,
}exti_trigger_enum;

void exti_init(gpio_pin_enum pin, exti_trigger_enum trigger,
    void_callback_uint32_ptr callback, void *ptr);

#endif

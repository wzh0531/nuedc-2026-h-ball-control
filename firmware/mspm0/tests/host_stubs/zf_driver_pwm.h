#ifndef _HOST_STUB_ZF_DRIVER_PWM_H_
#define _HOST_STUB_ZF_DRIVER_PWM_H_

#include "zf_common_typedef.h"

typedef enum
{
    PWM_TIM_A0_CH0_B14 = 0,
    PWM_TIM_A0_CH1_A7 = 1,
}pwm_channel_enum;

void pwm_init(pwm_channel_enum channel, uint32 frequency, uint32 duty);
void pwm_set_duty(pwm_channel_enum channel, uint32 duty);

#endif

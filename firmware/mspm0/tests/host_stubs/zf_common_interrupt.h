#ifndef _HOST_STUB_ZF_COMMON_INTERRUPT_H_
#define _HOST_STUB_ZF_COMMON_INTERRUPT_H_

#include "zf_common_typedef.h"

#define GPIOA_INT_IRQn  (0)
#define UART1_INT_IRQn  (1)
#define UART2_INT_IRQn  (2)

uint32 interrupt_global_disable(void);
void interrupt_global_enable(uint32 state);
void interrupt_set_priority(int irqn, uint8 priority);

#endif

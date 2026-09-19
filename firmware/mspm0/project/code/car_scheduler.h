#ifndef _CAR_SCHEDULER_H_
#define _CAR_SCHEDULER_H_

/* L2 BSP - TIMG12 1 ms 时基，ISR 回调只累计时间并置任务计数。 */
#include "zf_common_typedef.h"

void car_scheduler_init(void);
uint8 car_scheduler_take_control_ticks(void);
uint32 car_scheduler_get_time_ms(void);

#endif

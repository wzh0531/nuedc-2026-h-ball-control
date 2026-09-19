#ifndef _CAR_APP_H_
#define _CAR_APP_H_

/* L6 App - 底盘初始化和主循环编排。 */
#include "zf_common_typedef.h"

void car_app_init(void);
void car_app_process(void);
void car_app_clear_pending_ticks(void);
uint32 car_app_get_overrun_count(void);
uint8 car_app_get_max_pending_ticks(void);

#endif

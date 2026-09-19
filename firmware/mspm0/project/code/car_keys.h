#ifndef _CAR_KEYS_H_
#define _CAR_KEYS_H_

/* L2 BSP - 扩展板四按键消抖与短按/长按事件。 */
#include "zf_common_typedef.h"

typedef enum
{
    CAR_KEY_1 = 0,
    CAR_KEY_2,
    CAR_KEY_3,
    CAR_KEY_4,
    CAR_KEY_COUNT,
}car_key_id_enum;

typedef enum
{
    CAR_KEY_EVENT_NONE = 0,
    CAR_KEY_EVENT_SHORT,
    CAR_KEY_EVENT_LONG,
}car_key_event_enum;

void car_keys_init(void);
void car_keys_scan_5ms(void);
/* 返回已连续按下至少 10 ms 的消抖电平，供急停等电平型功能使用。 */
uint8 car_keys_is_pressed(car_key_id_enum key);
car_key_event_enum car_keys_take_event(car_key_id_enum key);

#endif

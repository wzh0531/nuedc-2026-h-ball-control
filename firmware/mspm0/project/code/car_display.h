#ifndef _CAR_DISPLAY_H_
#define _CAR_DISPLAY_H_

/*
 * L3 Driver - 扩展板 0.96 英寸 SSD1306 I2C OLED。
 * update 只更新内存帧缓冲，flush_step 每次只发送一页，避免阻塞 5 ms 控制。
 */
#include "zf_common_typedef.h"

void car_display_init(void);
void car_display_update(uint8 task, uint8 mode, uint32 runtime_ms,
    uint32 lap_time_ms, float distance_m, float left_speed_mps,
    float right_speed_mps, uint8 gray_mask, float line_error,
    int16 left_pwm, int16 right_pwm, uint32 fault_bits);
void car_display_flush_step(void);
/* 运行中只发送 TIME 所在页，避免全屏软件 I2C 刷新干扰控制周期。 */
void car_display_flush_time_page(void);

#endif

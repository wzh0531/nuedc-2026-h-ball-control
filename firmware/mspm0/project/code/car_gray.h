#ifndef _CAR_GRAY_H_
#define _CAR_GRAY_H_

/* L3 Driver - 感为八路灰度辅助板 CLK/DAT 串行驱动。 */
#include "zf_common_typedef.h"

void car_gray_init(void);
uint8 car_gray_read(void);
uint8 car_gray_get_mask(void);
uint8 car_gray_get_error(float *error);
uint8 car_gray_get_black_count(void);

#endif
